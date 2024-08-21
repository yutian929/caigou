# 创建class WaveshareNode, 并注册节点 waveshare_node
#1.附加函数：创建waveshare_coords话题发布者，发布坐标信息
#2.创建srv_basic_arm_cmd服务端，接收基础机械臂指令,并解析执行
#3.创建srv_whether_arm_reahable服务端，接收rs坐标系下的坐标，判断机械臂是否能够到达
#4.创建串口通讯，与arduino nano通讯，得知气泵气压状态

import rclpy
from rclpy.node import Node
from caigou_interfaces.srv import BasicArmCmd, WhetherArmReachable, Bridge
from caigou_interfaces.msg import ArmCoords
import time
import json
import numpy as np
import transforms3d as tfs
import serial
import serial.tools.list_ports




class WaveshareNode(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        # 创建srv_basic_arm_cmd服务端
        self.srv_basic_arm_cmd_server = self.create_service(BasicArmCmd, 'srv_basic_arm_cmd',
                                                            self.basic_arm_cmd_callback)
        # 创建srv_whether_arm_reahable服务端
        self.srv_whether_arm_reahable_server = self.create_service(WhetherArmReachable, 'srv_whether_arm_reachable',
                                                                   self.whether_arm_reachable_callback)

        # 与机械臂控制板通信
        waveshare_vendor_id = 0x10c4  # 十六进制的 Vendor ID
        waveshare_product_id = 0xea60  # 十六进制的 Product ID
        waveshare_serial_port = self.find_serial_port(waveshare_vendor_id, waveshare_product_id)
        self.ser = serial.Serial(waveshare_serial_port, 115200, timeout=1)
        self.get_logger().info(f'INFO --- Connected with waveshare serial port: {waveshare_serial_port}')
        if waveshare_serial_port is None:
            self.get_logger().error(f"ERROR --- No waveshare serial port found! wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww")
            return

        # 与arduino板创建串口连接
        arduino_vendor_id = 0x1a86  # 十六进制的 Vendor ID
        arduino_product_id = 0x7523  # 十六进制的 Product ID
        arduino_serial_port = self.find_serial_port(arduino_vendor_id, arduino_product_id)
        self.serial_arduino = serial.Serial(arduino_serial_port, 9600, timeout=1)
        self.get_logger().info(f'INFO --- Connected with arduino serial port: {arduino_serial_port}')
        if arduino_serial_port is None:
            self.get_logger().error(f"ERROR --- No arduino serial port found! aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
            return

        try:
            with open("save_Hcg.json", "r") as f:
                self.hcg = json.load(f)  # 读取保存的Hcg,也就是相机坐标系到机械臂坐标系的变换矩阵，为4x4的齐次矩阵
            self.get_logger().info(f"INFO --- Hcg loaded successfully!\n{self.hcg}")
        except FileNotFoundError:
            self.get_logger().error("ERROR --- No save_Hcg.json file found!")
            self.hcg = np.array(
                [[0.008696904615170808, -0.1010059265819499, -0.9948478107959209, -0.000992849030701995],
                 [-0.9999146966719763, -0.010573864008121214, -0.007667644966187326, 0.0527789136622589],
                 [-0.009744907875621989, 0.9948296317436768, -0.1010892703269548, -0.045632540710289445],
                 [0.0, 0.0, 0.0, 1.0]])
        self.rectify_x = 0.0  # m
        self.rectify_y = 0.0
        self.rectify_z = 0.0

        # 全局配置文件
        self.config_file_path = "/home/caigou/workspace/caigou_ws/config.json"
        self.config = {}  # 包含机械臂配置参数"arm_reachable_range": [0.12, 0.35], "ideal_package_xy": [0.18, 0], "package_release_xyz": [0.0, 0.15, 0.05]
        # 加载配置文件，不需要判断是否完成init
        self.read_config()

        self.pump_status = None  # 气泵状态
        self.pump_grab_threshold = 10  # 吸取阈值

        self.horizontal_t_pos = 3.1415926  # rad
        self.headdown_t_pos = 4.1

        self.company_info = {}
        self.no_error_occur = True

        self.init()
        time.sleep(1)
        self.pos_center_horizontal()
        time.sleep(1)
        self.pos_center_headdown()

        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')

        # DEBUG启动坐标发布, hand-eye
        # self.pub_coords()
        # self.arm_release()

        # self.pos_target(-0.0904325470328331, -0.0002774444583337754, 0.2998945713043213, 4.522175312042236)
        # time.sleep(2)

        # self.pos_center_horizontal()
        # self.pos_target(0.4, 0, 0 + 0.05)  # 抓取前先抬高5cm
        # self.pump_grab()  # 打开气泵，吸取
        # self.pos_shift('z', 'down')  # z轴减少
        # self.wait_until_pump_grabbed()
        # self.pos_shift('z', 'stop')  # z轴停止
        # self.pos_center_horizontal()
        # time.sleep(3)

        # self.pos_shelf_horizontal()


    def whether_arm_reachable_callback(self, request, response):
        self.get_logger().info('INFO --- Received whether_arm_reachable request')
        # 不需要实时更新配置文件
        rs_x = request.rs_x
        rs_y = request.rs_y
        rs_z = request.rs_z
        # rs坐标系转换为机械臂坐标系
        arm_x, arm_y, arm_z = self.rs2arm(rs_x, rs_y, rs_z)
        # 人工修正
        arm_x += self.rectify_x
        arm_y += self.rectify_y
        arm_z += self.rectify_z
        self.get_logger().info(
            f'INFO --- rs_x: {rs_x}, rs_y: {rs_y}, rs_z: {rs_z} -> arm_x: {arm_x}, arm_y: {arm_y}, arm_z: {arm_z}')
        reachable, x_shift, y_shift = self.whether_arm_reachable(arm_x, arm_y, arm_z)
        response.arm_reachable = reachable
        response.arm_x_shift = float(x_shift)
        response.arm_y_shift = float(y_shift)
        self.get_logger().info(f'INFO --- arm_reachable: {reachable}, x_shift: {x_shift}, y_shift: {y_shift}')
        return response
        # DEBUG
        # response.arm_reachable = True
        # response.arm_x_shift = 0
        # response.arm_y_shift = 0
        # return response

    def basic_arm_cmd_callback(self, request, response):
        # 加载最新的配置文件
        self.read_config()
        self.no_error_occur  =True
        # 解析request
        basic_arm_cmd = json.loads(request.basic_arm_cmd_req)

        if len(basic_arm_cmd) == 0:
            self.get_logger().error("ERROR --- no cmd")
            response.basic_arm_cmd_res = False
            return response
        self.get_logger().info(
            f'INFO --- Received basic_arm_cmd = {basic_arm_cmd}, total steps is {len(basic_arm_cmd)}')
        try:
            for i in range(len(basic_arm_cmd)):
                self.solve_every_cmd(basic_arm_cmd[i])
                # time.sleep(5)
            if self.no_error_occur:
                response.basic_arm_cmd_res = True
            else:
                response.basic_arm_cmd_res = False
        except Exception as e:
            self.get_logger().error(f"Arm_action Error --- {e}")
            response.basic_arm_cmd_res = False

        return response

    def solve_every_cmd(self, cmd):
        action_type, obj_name, x, y, z = cmd  # 单位都是m

        # 首先判断是rs坐标系还是机械臂坐标系,如果是rs坐标系则转换为机械臂坐标系,并进行人工修正
        if action_type[:2] == "rs":
            x, y, z = self.rs2arm(x, y, z)  # 只进行坐标转换
            # 人工修正
            x += self.rectify_x
            y += self.rectify_y
            z += self.rectify_z

        self.get_logger().info(f'INFO --- action_type: {action_type}, obj_name: {obj_name}, x: {x}, y: {y}, z: {z}')

        if action_type == "rs_grab" or action_type == "arm_grab":  # 抓取
            self.pos_center_horizontal()
            self.pos_target(x, y, z + 0.05)  # 抓取前先抬高5cm
            self.pump_grab()  # 打开气泵，吸取
            self.pos_shift('z', 'down')  # z轴减少
            self.wait_until_pump_grabbed()
            time.sleep(0.3)
            self.pos_shift('z', 'stop')  # z轴停止
            # self.send_cmd({"T": 104, "x": 100, "y": 0, "z": 100, "t": self.horizontal_t_pos, "spd": 0.5})
            self.pos_target(0.1, 0, 0.2)
            time.sleep(2)


        elif action_type == 'arm_release_package':
            release_x, release_y, release_z = self.config["package_release_xyz"]
            # temp
            self.pos_target(0, 0.2, release_z+0.05)
            time.sleep(1.5)
            self.pos_target(release_x, release_y, release_z)
            time.sleep(2)
            self.pos_target(release_x, release_y, release_z-0.09)
            time.sleep(1)
            self.pump_release()
            time.sleep(1)
            # self.pos_center_headdown()
        
        elif action_type == "pos_center_headdown":
            self.pos_center_headdown()

        elif action_type == "arm_place":  # 放置
            self.pos_center_horizontal()
            self.pos_target(x, y, z)
            time.sleep(3)
            self.pump_release()
            time.sleep(1)
            self.pos_center_headdown()
            self.get_logger().info('INFO --- Placing over!')

        elif action_type == "save_temp_pos":  # 保存临时位置
            temp_x, temp_y, temp_z, temp_rx, temp_ry, temp_rz = self.get_coords_indeed()
            self.get_logger().info(
                f'INFO --- Saving temp pos: ({temp_x},{temp_y},{temp_z},{temp_rx},{temp_ry},{temp_rz})')
            self.temp_x = temp_x
            self.temp_y = temp_y
            self.temp_z = temp_z
            self.temp_rx = temp_rx
            self.temp_ry = temp_ry
            self.temp_rz = temp_rz

        elif action_type == "pos_shelf_horizontal":
            self.pos_shelf_horizontal(int(obj_name))

        elif action_type == "pos_shift":
            self.pos_shift(obj_name, x)

        elif action_type == "pos_read_qrcode":
            # 首先运动到放包裹的位置,让末端执行器正视下方
            # x: -0.0904325470328331
            # y: -0.0002774444583337754
            # z: 0.2998945713043213
            # rx: 0.0
            # ry: 4.522175312042236
            # rz: -3.1385247707366943
            # release_x, release_y, release_z = self.config["package_release_xyz"]
            self.pos_target(-0.0904325470328331, 0, 0.2998945713043213, 4.522175312042236)
            time.sleep(2)

        elif action_type == "put_package":
            # 首先拿起包裹
            release_x, release_y, release_z = self.config["package_release_xyz"]
            self.pos_target(0, 0.2, release_z+0.05)  # 抓取前先抬高5cm
            time.sleep(0.5)
            self.pos_target(release_x, release_y, release_z)  
            time.sleep(0.5)
            self.pump_grab()  # 打开气泵，吸取
            self.pos_shift('z', 'down', 10)  # z轴减少
            self.wait_until_pump_grabbed()
            time.sleep(0.2)
            self.pos_shift('z', 'stop')  # z轴停止
            # 运动到temp位置
            # self.pos_target(self.temp_x, self.temp_y, self.temp_z)
            # time.sleep(3)
            # 放置到目标位置
            final_x = self.temp_x + x
            final_y = self.temp_y + y
            final_z = self.temp_z + z
            # release, but final z
            self.pos_target(release_x, release_y, final_z)
            time.sleep(1)
            # temp, but final z
            self.pos_target(self.temp_x, self.temp_y, final_z)
            time.sleep(1.5)
            # temp1
            temp_x1 = self.temp_x + x/2
            temp_y1 = self.temp_y + y/2
            temp_z1 = final_z
            self.pos_target(temp_x1, temp_y1, temp_z1)
            time.sleep(1.1)
            # temp2
            temp_x2 = self.temp_x + x*3/4
            temp_y2 = self.temp_y + y*3/4
            temp_z2 = final_z
            self.pos_target(temp_x2, temp_y2, temp_z2)
            time.sleep(0.8)
            # final
            self.pos_target(final_x, final_y, final_z)
            time.sleep(0.8)
            self.pos_target(final_x, final_y, final_z-0.1)
            time.sleep(0.5)
            # 放下包裹
            self.pump_release()
            time.sleep(0.2)
            self.pos_target(final_x, final_y, final_z+0.05)
            time.sleep(1.5)
            # 回到temp, but final z
            self.pos_target(self.temp_x, self.temp_y, final_z)
            time.sleep(1)
            # 回到初始位置
            self.pos_center_headdown()
            self.get_logger().info('INFO --- Putting package over!')

        elif action_type == "pickup_grab_package":  # 从车上抓取包裹
            # 首先拿起包裹
            release_x, release_y, release_z = self.config["package_release_xyz"]
            # self.pos_target(0, 0.2, release_z + 0.05)  # 抓取前先抬高5cm
            # time.sleep(0.5)
            self.pos_target(release_x, release_y, release_z)
            time.sleep(0.5)
            self.pump_grab()  # 打开气泵，吸取
            self.pos_shift('z', 'down', 10)  # z轴减少
            self.wait_until_pump_grabbed()
            time.sleep(0.5)
            self.pos_shift('z', 'stop')  # z轴停止
            self.pos_target(release_x, release_y, release_z+0.05)
            time.sleep(0.5)
            # 放到正前方
            self.pos_target(0, 0.2, release_z + 0.05)
            time.sleep(1)
            self.pos_target(0.2, 0, release_z)
            time.sleep(1)

        elif action_type == "pickup_release_package":  # 放到揽货区
            if obj_name in self.company_info:
                self.company_info[obj_name] += 1
            else:
                self.company_info[obj_name] = 1
            release_x, release_y, release_z = self.config["package_release_xyz"]
            y_offest = 0.15 if self.company_info[obj_name] % 2 == 0 else -0.15
            self.pos_target(0.3, y_offest, release_z - 0.40)
            time.sleep(2)
            self.pump_release()
            time.sleep(0.5)
            self.pos_center_headdown()

        # DEBUG    
        elif action_type == "debug" or action_type == "rs_debug":  # 调试, 照做就行
            # self.get_logger().info(f'INFO --- Debugging... pos: {x}, {y}, {z}')
            self.pos_target(x, y, z)
            time.sleep(3)
            self.pos_center_headdown()
            self.arm_release()

        elif action_type == "arm_release":
            self.arm_release()
        else:
            pass

    def whether_arm_reachable(self, arm_x, arm_y, arm_z):  # 判断机械臂是否能够到达，能到达的话返回True，否则返回False+需要平移的x，y距离
        arm_reachable_range = self.config["arm_reachable_range"]
        ideal_package_xy = self.config["ideal_package_xy"]
        dis = np.sqrt(arm_x ** 2 + arm_y ** 2 + arm_z ** 2)
        xy_dis = np.sqrt(arm_x ** 2 + arm_y ** 2)
        self.get_logger().info(f'INFO --- arm_reachable_range: {arm_reachable_range}, now dis: {dis}, xy_dis:{xy_dis}')
        if arm_reachable_range[0] <= xy_dis <= arm_reachable_range[1]:
            return True, 0, 0
        else:
            x_shift = arm_x - ideal_package_xy[0]
            y_shift = arm_y - ideal_package_xy[1]
            return False, x_shift, y_shift

    def wait_until_pump_grabbed(self, threshold_time=5):
        start_time = time.time()
        while True:
            if time.time() - start_time > threshold_time:  # 如果超过5秒都没抓到东西，那肯定是出问题了
                self.get_logger().error(f'ERROR --- Pump grab failed!!! over {threshold_time}s !!!')
                self.no_error_occur = False
                return False
            try:
                pump_status = self.read_once_pumpAB()
                self.get_logger().info(f'INFO --- pump_status = {pump_status}')
                if pump_status is not None:
                    if pump_status["pumpA"] < self.pump_grab_threshold or pump_status[
                        "pumpB"] < self.pump_grab_threshold:
                        self.get_logger().info('INFO --- Pump grab successfully!')
                        self.get_logger().info('INFO --- Grabbing over!')
                        return True
                    else:
                        time.sleep(0.1)
                        self.get_logger().info(f'INFO --- Pump grab failed, pump_status = {pump_status}, retrying...')
            except Exception as e:
                self.get_logger().error(f"ERROR --- {e}")

    def rs2arm(self, x, y, z):
        # rs坐标系转换为机械臂坐标系
        target_in_cam_4x4 = self.get_matrix_eular_radu(x, y, z, 0, 0, 0)  # realsense看到的目标位姿, 4x4齐次矩阵
        cam_in_end = self.hcg  # 相机坐标系到机械臂坐标系的变换矩阵, 4x4齐次矩阵
        end_in_base = self.get_matrix_eular_radu(*self.get_coords_indeed())  # 机械臂末端位姿, 4x4齐次矩阵

        base2target_4x4 = np.dot(end_in_base, cam_in_end)
        base2target_4x4 = np.dot(base2target_4x4, target_in_cam_4x4)
        arm_x, arm_y, arm_z = base2target_4x4[:3, 3]

        return arm_x, arm_y*0.9, arm_z

    def get_matrix_eular_radu(self, x, y, z, rx, ry, rz):
        rmat = tfs.euler.euler2mat(rx, ry, rz)
        rmat = tfs.affines.compose(np.squeeze(np.asarray((x, y, z))), rmat, [1, 1, 1])
        return rmat

    def send_cmd(self, dict_cmd):
        # self.get_logger().info(f'INFO --- Sending cmd (dict): {dict_cmd}')
        cmd = json.dumps(dict_cmd) + "\n"
        self.ser.write(cmd.encode())
        self.ser.flush()

    def close(self):
        self.ser.close()
        print('Serial port closed.')

    def reset(self):
        self.send_cmd({"T": 100})

    def init(self):
        init_data_EoAT = {"T": 2, "pos": 3, "ea": 30, "eb": 55}
        self.send_cmd(init_data_EoAT)

    def read_config(self):
        with open(self.config_file_path, "r") as f:
            self.config = json.load(f)
        # self.get_logger().info(f'INFO --- Read config: {self.config}')

    def find_serial_port(self, vendor_id, product_id):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if port.vid == vendor_id and port.pid == product_id:
                print(f"Found target device on port {port.device}")
                return port.device
        return None

    def arm_release(self):  # 解开扭矩锁
        cmd = {"T": 210, "cmd": 0}
        self.send_cmd(cmd)

    def pub_coords(self):  # 启动坐标发布
        self.get_logger().info('INFO --- Start publishing coords...')
        self.msg_waveshare_coords_pub = self.create_publisher(ArmCoords, 'msg_arm_coords', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 0.05s发布一次坐标信息

    def timer_callback(self):
        x, y, z, rx, ry, rz = self.get_coords_indeed()  # 单位m,rad
        msg = ArmCoords()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.rx = rx
        msg.ry = ry
        msg.rz = rz
        # DEBUG
        self.get_logger().info(f'INFO --- Publish coords: ({msg.x},{msg.y},{msg.z},{msg.rx},{msg.ry},{msg.rz})')
        self.msg_waveshare_coords_pub.publish(msg)

    def read_feedback(self, eclipsed_time=0.2):
        time.sleep(0.2)
        start_time = time.time()
        while time.time() - start_time < eclipsed_time:
            if self.ser.in_waiting:
                raw_data = self.ser.read(self.ser.in_waiting).decode().strip()
                if raw_data:
                    raw_data_list = raw_data.split('\n')
                    raw_data = raw_data_list[-1]
                    try:
                        data = json.loads(raw_data)
                        # self.get_logger().info(f'DEBUG --- Received data: {data}')
                        return data
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f'Failed to decode JSON: {e}')
                    except Exception as e:
                        self.get_logger().error(f'Unexpected error: {e}')
        self.get_logger().error('ERROR --- No feedback received!')
        self.ser.flush()
        return None

    def get_coords(self):  # 获取当前机械臂末端位姿 x,y,z,t,b
        self.send_cmd({"T": 105})
        feedback = self.read_feedback()
        if feedback is None:
            self.get_logger().error('ERROR --- Feedback is None')
            return None, None, None, None, None
        try:
            # self.get_logger().info(f'INFO --- Feedback: {feedback}')
            x = feedback['x']  # mm
            y = feedback['y']
            z = feedback['z']
            t = feedback['t']
            b = feedback['b']
        except KeyError as e:
            self.get_logger().error(f'ERROR --- Missing expected key in feedback: {e}')
            return None, None, None, None, None  # or some default values or re-request data
        return x / 1000, y / 1000, z / 1000, t, b

    def get_coords_indeed(self):  # 保证获取到机械臂末端位姿 x,y,z,rx,ry,rz
        fucking_waveshare_serial_times = 0
        temp_x, temp_y, temp_z, temp_t, temp_b = None, None, None, None, None
        while not all([temp_x, temp_y, temp_z, temp_t, temp_b]):  # 保证获取到所有数据，且必须全部不为None
            temp_x, temp_y, temp_z, temp_t, temp_b = self.get_coords()
            fucking_waveshare_serial_times += 1
            if fucking_waveshare_serial_times == 10:
                # 0.0968509552,-0.000891430199,0.09003179116,0.0,3.93312674,-0.009203885
                temp_x = 0.0968509552
                temp_y = -0.000891430199
                temp_z = 0.09003179116
                temp_t = self.headdown_t_pos
                temp_b = -0.009203885
                self.get_logger().info(f'INFO --- fucking_waveshare_serial !!!!')
        x = temp_x
        y = temp_y
        z = temp_z
        rx = 0.0
        ry = temp_t
        rz = temp_b
        
        return x, y, z, rx, ry, rz

    def pos_mid_horizontal(self):
        self.send_cmd({"T": 104, "x": 0, "y": 100, "z": 100, "t": self.horizontal_t_pos, "spd": 0.5})

    def pos_center_headdown(self):
        self.send_cmd({"T": 104, "x": 100, "y": 0, "z": 100, "t": self.headdown_t_pos, "spd": 0.5})

    def pos_center_horizontal(self):
        self.send_cmd({"T": 104, "x": 100, "y": 0, "z": 100, "t": self.horizontal_t_pos, "spd": 0.5})

    def pos_shelf_horizontal(self, shelf_id='useless'):
        x = 0
        y = 130
        z = -43
        t = 3.34
        # if shelf_id == 1:  # 0, 0.12, 0.18, 3.52
        #     self.send_cmd({"T": 104, "x": 0, "y": 250, "z": 20, "t": self.horizontal_t_pos, "spd": 0.5})
        # elif shelf_id == 2:  # 0, 0.15, -0.05, 3.86
        #     self.send_cmd({"T": 104, "x": 0, "y": 250, "z": 20, "t": self.horizontal_t_pos, "spd": 0.5})
        self.send_cmd({"T": 104, "x": x, "y": y, "z": z, "t": t, "spd": 0.5})
        time.sleep(3)

    def pos_shift(self, axis, cmd, spd=3):  # 机械臂末端坐标平移
        # axis(1,2,3,4对应x,y,z,t) cmd(0停止1增加2减少) spd(0~20)
        if axis == 'x':
            axis = 1
        elif axis == 'y':
            axis = 2
        elif axis == 'z':
            axis = 3
        elif axis == 't':
            axis = 4

        if cmd == 'up':
            cmd = 1
        elif cmd == 'down':
            cmd = 2
        elif cmd == 'stop':
            cmd = 0

        cmd_send = {"T": 123, "m": 1, "axis": axis, "cmd": cmd, "spd": spd}
        self.get_logger().info(f'INFO --- Sending cmd: {cmd_send}')
        self.send_cmd(cmd_send)
        # time.sleep(0.05)
        # self.send_cmd({"T":103,"axis":4,"pos":self.horizontal_t_pos,"spd":0.25})
        time.sleep(0.05)


    def pos_target(self, x, y, z, t=None, spd=0.5):  # 机械臂末端坐标移动到目标位置, 默认保持水平
        if t is None:
            t = self.horizontal_t_pos
            cmd = {"T": 104, "x": x * 1000, "y": y * 1000, "z": z * 1000, "t": t, "spd": spd}
            self.send_cmd(cmd)
        else:
            cmd = {"T": 104, "x": x * 1000, "y": y * 1000, "z": z * 1000, "t": t, "spd": spd}
            self.send_cmd(cmd)

    def pump_grab(self):  # 后续引入反馈检测，判断是否成功吸取
        pwm = 200  # 12V -> 6V
        cmd = {"T": 114, "led": pwm}
        self.send_cmd(cmd)
        # time.sleep(5)

    def pump_release(self):  # 后续引入反馈检测，判断是否成功放下
        pwm = 0
        cmd = {"T": 114, "led": pwm}
        self.send_cmd(cmd)
        # time.sleep(5)

    def read_once_pumpAB(self):
        """读取一次气泵状态."""
        self.serial_arduino.flushInput()
        try:
            line = self.serial_arduino.readline().decode()
            eles = line.split()  # ['pumpA', '29', 'pumpB', '33']
            # 找到pumpA，pumpB的值
            if len(eles) == 4:
                if eles[0] == "pumpA":
                    pumpA = int(eles[1])
                    if eles[2] == "pumpB":
                        pumpB = int(eles[3])
                        return {"pumpA": pumpA, "pumpB": pumpB}
                        # self.get_logger().info(f"INFO --- pump_status = {self.pump_status}")

        except Exception as e:
            print(f"Error reading from serial port: {str(e)}")
        return None


def main(args=None):
    rclpy.init(args=args)
    waveshare_node = WaveshareNode("waveshare_node")
    try:
        rclpy.spin(waveshare_node)
    except KeyboardInterrupt:
        pass
    finally:
        waveshare_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
