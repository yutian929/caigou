# 创建class SortNode, 并注册节点 sort_node
# 1.创建msg_status订阅器，用于接收状态码，
# 2.创建srv_task_finished客户端，当初始化完成时，发送给ai_status_judge_node节点
# 3.创建srv_bridge客户端与小车底盘通信
# 4.创建srv_cobot客户端，BasicArmCmd服务接口，与arm进行互动
# 5.创建srv_package_track客服端，跟realsense要包裹追踪的信息

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from caigou_interfaces.srv import TTS, TaskSchedule, Bridge, BasicArmCmd, WhetherArmReachable, PackageTrack, ParcelInfo, ShelfInfo, DataBase
import json
import time
import numpy as np
from functools import partial


class SortNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # 创建客户端
        self.srv_tts_client = self.create_client(TTS, 'srv_tts')
        self.srv_task_schedule_client = self.create_client(TaskSchedule, 'srv_task_schedule')
        self.srv_bridge_client = self.create_client(Bridge, 'srv_bridge')
        self.srv_basic_arm_cmd_client = self.create_client(BasicArmCmd, 'srv_basic_arm_cmd')
        self.srv_whether_arm_reahable_client = self.create_client(WhetherArmReachable, 'srv_whether_arm_reachable')
        self.srv_package_track_client = self.create_client(PackageTrack, 'srv_package_track')
        self.srv_parcel_info_client = self.create_client(ParcelInfo, 'srv_parcel_info')
        self.srv_shelf_info_client = self.create_client(ShelfInfo, 'srv_shelf_info')
        self.srv_database_client = self.create_client(DataBase, 'srv_database')

        # 创建msg_status订阅器
        self.msg_status_sub = self.create_subscription(String, 'msg_status', self.msg_status_callback, 10)

        # 没有包裹的次数，超过10次就认为都分拣完了
        self.no_package_count = 0
        self.former_package_track_rsxyzcls = []

        # 配置文件
        self.config_file_path = "/home/caigou/workspace/caigou_ws/config.json"

        # sort任务的配置, 在sort_start状态时读取
        self.config = {}

        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')

        # # # DEBUG
        # self.config = self.read_config()
        # time.sleep(15)
        # self.taget_warehouse = (1, 1, 2)
        # self.task_schedule_feedback("sort_goto_cabinets_finished")

    def msg_status_callback(self, msg):
        # self.get_logger().info(f'INFO --- received msg_status = {msg.data}')
        if 'sort' in msg.data:  # 判断发出的status是否是sort类，如果不是可以直接忽略了
            self.get_logger().info(f'INFO --- received msg_status = {msg.data}, validating whether initialized')

            config = self.read_config()
            if not config['init']:  # 没有初始化，无法执行后续任务
                self.not_initialized()
                return
            self.config = config  # 读取最新全部配置

            self.global_status = msg.data  # 记录当前的状态

            # 判断应该是sort的哪一步了
            if msg.data == 'sort_start':  # 最初的收到人为指令
                self.sort_start()
            elif msg.data == 'sort_goto_unloading_area':  # 前往卸货区->sort_goto_unloading_area_finished
                self.sort_goto_unloading_area_start()
            elif msg.data == 'sort_package_tracking':  # 追踪捡起包裹->sort_get_package_finished
                self.sort_package_tracking_start()
            elif msg.data == 'sort_goto_cabinets':  # 前往快递柜->sort_goto_cabinets_finished
                self.sort_goto_cabinets_start()
            elif msg.data == 'sort_put_package':  # 开始上架包裹->sort_put_package_finished
                self.sort_put_package_start()
            elif msg.data == 'sort_wait_for_new_package':  # 等待新包裹->sort_package_tracking
                self.sort_package_wait_for_new_package()
            else:
                return
        else:
            self.global_status = "free"
            return

    def sort_start(self):
        self.tell_to_tts('开始分拣上架', self.sort_goto_unloading_area_start)

    def sort_goto_unloading_area_start(self, future='just_wait'):
        self.no_package_count = 0
        self.tell_to_tts('开始前往分拣区', self.sort_goto_unloading_area_1)

    def sort_goto_unloading_area_1(self, future='just_wait'):  # 确定开始分拣了，先通过bridge给小车发送最终要去的地方xy和角度
        self.tell_to_car_base({"set_pose": [self.config['sort_unloading_area_xy'][0],
                                            self.config['sort_unloading_area_xy'][1],
                                            self.config['sort_unloading_area_angle']]},
                              self.sort_goto_unloading_area_1_cb)

    def sort_goto_unloading_area_1_cb(self, future):  # 小车给出了回复，下面校验一下是否正确到达指定位置以及角度
        response = future.result()
        self.get_logger().info(f'INFO --- sort_goto_unloading_area_1 response.bridge_res = {response.bridge_res}')
        try:
            bridge_res = json.loads(response.bridge_res)  # 一个字典，正常应该是{"set_pose": [latest_x,y,angle]}
            latest_x, latest_y, latest_angle = bridge_res['set_pose']
            # if abs(latest_x - self.config['sort_unloading_area_xy'][0]) <= self.config['xy_tolerance'] and abs(
            #         latest_y - self.config['sort_unloading_area_xy'][1]) <= self.config['xy_tolerance']:
            #     if abs(latest_angle - self.config['sort_unloading_area_angle']) <= self.config[
            #         'angle_tolerance'] or abs(latest_angle - self.config['sort_unloading_area_angle']) >= (
            #             360 - self.config['angle_tolerance']):
            #         self.task_schedule_feedback("sort_goto_unloading_area_finished")
            #     else:
            #         self.get_logger().error(
            #             'WARN --- sort_goto_unloading_area_1_cb latest_angle exceed max angle tolerance, retry')
            #         self.sort_goto_unloading_area_1()
            # else:
            #     self.get_logger().error(
            #         'WARN --- sort_goto_unloading_area_1_cb latest_location exceed max xy tolerance, retry')
            #     self.sort_goto_unloading_area_1()
            self.task_schedule_feedback("sort_goto_unloading_area_finished")
        except Exception as e:
            self.get_logger().error(f'ERROR --- sort_goto_unloading_area_1 response.bridge_res is not json, {str(e)}')
            time.sleep(1)
            self.sort_goto_unloading_area_1()

    def sort_package_tracking_start(self):  # 发送语音，开始分拣上架
        self.tell_to_tts('开始分拣', self.sort_package_tracking_1)  # 语音播报完成后进入分拣过程

    def sort_package_tracking_1(self,
                                future='useless'):  # 语音播报完成，此时状态是self.status = 'sort_package_tracking', realsense 开始追踪
        # 跟realsenese要最新的包裹跟踪信息
        while not self.srv_package_track_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for package_track service')
        request = PackageTrack.Request()
        request.package_track_req = 'package_track_rsxyzcls_list'  # 要arm坐标系下的xyz坐标，以及包裹的class id
        self.srv_package_track_client.call_async(request).add_done_callback(self.sort_package_tracking_1_cb)

    def sort_package_tracking_1_cb(self, future):  # realsense给出了最新的包裹信息，找到距离最近的包裹，发送请求是否能够抓取
        response = future.result()
        self.get_logger().info(
            f'INFO --- sort_package_tracking_1 response.package_track_res = {response.package_track_res}')
        package_track_rsxyzcls = json.loads(response.package_track_res)  # 一个列表的列表[[arm_x, arm_y, arm_z, cls, id]...]

        if len(package_track_rsxyzcls) == 0:  # 如果没有包裹，超过10次就直接结束
            if self.no_package_count >= 10:
                self.sort_finished()
                return
            time.sleep(0.5)
            self.no_package_count += 1
            self.sort_package_tracking_1()
        else:
            if not self.whether_stable_frame(package_track_rsxyzcls):  # 如果包裹信息不稳定，重新获取
                self.sort_package_tracking_1()
                return
            self.no_package_count = 0  # 有包裹，重置no_package_count，找到最近的包裹
            min_distance = 9999999
            nearest_package = []
            for package in package_track_rsxyzcls:
                distance = np.sqrt(package[0] ** 2 + package[1] ** 2 + package[2] ** 2)
                if distance < min_distance:
                    min_distance = distance
                    nearest_package = package
            if len(nearest_package) == 0:
                self.get_logger().error('ERROR --- sort_package_tracking_3 no nearest package')
                self.sort_package_tracking_1()
                return

            # 判断最近的包裹是否在机械臂可抓取的范围self.arm_reachable_range = (min, max)
            self.get_logger().info(
                f'INFO --- sort_package_tracking_1 nearest_package = {nearest_package}, min_distance = {min_distance}')
            self.sort_package_tracking_2(nearest_package)

    def sort_package_tracking_2(self, nearest_package):
        # 判断是否在机械臂可抓取的范围
        req = WhetherArmReachable.Request()
        req.rs_x = float(nearest_package[0])
        req.rs_y = float(nearest_package[1])
        req.rs_z = float(nearest_package[2])
        while not self.srv_whether_arm_reahable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for whether_arm_reahable service')
        callback = partial(self.sort_package_tracking_2_cb, nearest_package)
        self.srv_whether_arm_reahable_client.call_async(req).add_done_callback(callback)
        self.get_logger().info(f'INFO --- sort_package_tracking_2 whether_arm_reahable req = {req}')

    def sort_package_tracking_2_cb(self, nearest_package, future):  # waveshare arm 给出回复是否可抓，如果能抓就直接抓，不能就bridge移动小车
        response = future.result()
        reachable = response.arm_reachable
        if reachable:  # 在机械臂可抓范围内，直接抓取
            self.sort_get_package_1(nearest_package)
        else:  # 不在机械臂可抓范围内，需要结合当前小车的位置&角度，和我需要的小车shift的x&y，来计算新的目标位置
            arm_x_shift = response.arm_x_shift
            arm_y_shift = response.arm_y_shift
            self.sort_package_tracking_3_1(arm_x_shift, arm_y_shift)

    def sort_package_tracking_3_1(self, arm_x_shift, arm_y_shift):  # 先get一下小车的当前位置xy和角度
        callback = partial(self.sort_package_tracking_3_1_cb, arm_x_shift, arm_y_shift)
        self.tell_to_car_base({"get_pose": 0}, callback)

    def sort_package_tracking_3_1_cb(self, arm_x_shift, arm_y_shift, future):  # 小车给出了回复，校验一下是不是 get_pose
        response = future.result()
        self.get_logger().info(f'INFO --- sort_package_tracking_3_1 response.bridge_res = {response.bridge_res}')
        bridge_res = json.loads(response.bridge_res)  # 一个字典，正常应该是{"get_location": [latest_x,y]}
        if "get_pose" in bridge_res:
            car_latest_x, car_latest_y, car_latest_angle = bridge_res["get_pose"]
            self.sort_package_tracking_3_2(arm_x_shift, arm_y_shift, car_latest_x, car_latest_y, car_latest_angle)
        else:
            self.get_logger().error('ERROR --- sort_package_tracking_3_1_cb response.bridge_res is not get_location')
            self.sort_package_tracking_3_1(arm_x_shift, arm_y_shift)

    def sort_package_tracking_3_2(self, arm_x_shift, arm_y_shift, car_latest_x, car_latest_y,
                                  car_latest_angle):  # 计算新的目标位置
        # 此时arm_x_shift, arm_y_shift是相对于机械臂的偏移量，单位m
        # car_latest_x, car_latest_y是小车当前的位置，单位m
        # car_latest_angle是小车当前的角度，单位°
        angle_rad = np.deg2rad(car_latest_angle)
        x_offset_car = arm_x_shift * np.cos(angle_rad) - arm_y_shift * np.sin(angle_rad)
        y_offset_car = arm_x_shift * np.sin(angle_rad) + arm_y_shift * np.cos(angle_rad)
        target_x_car = car_latest_x + x_offset_car
        target_y_car = car_latest_y + y_offset_car
        # angle_offset_car = np.arctan2(y_offset_car, x_offset_car)
        # angle_offset_car = np.rad2deg(angle_offset_car)
        callback = partial(self.sort_package_tracking_3_2_cb, self.config["sort_unloading_area_angle"], target_y_car)
        self.tell_to_car_base(
            {"set_pose_delta": [x_offset_car, y_offset_car, self.config["sort_unloading_area_angle"], 1]}, callback)  # 现在只需要发世界坐标系的delta就行

    def sort_package_tracking_3_2_cb(self, target_x_car, target_y_car, future):  # 小车给出了回复，下面校验一下是否到达了新的目标位置
        response = future.result()
        self.get_logger().info(f'INFO --- sort_package_tracking_3_2_cb response.bridge_res = {response.bridge_res}')
        bridge_res = json.loads(response.bridge_res)  # 一个字典，正常应该是{"set_location": [latest_x,y]}
        if "set_pose_delta" in bridge_res:
            latest_x, latest_y, latest_angle = bridge_res["set_pose_delta"]
            # if abs(latest_x - target_x_car) <= self.config['xy_tolerance'] and abs(latest_y - target_y_car) <= \
            #         self.config['xy_tolerance']:
            #     if abs(latest_angle - self.config['sort_unloading_area_angle']) <= self.config[
            #         'angle_tolerance'] or abs(latest_angle - self.config['sort_unloading_area_angle']) >= (
            #             360 - self.config['angle_tolerance']):
            #         self.sort_package_tracking_1()  # 位置，角度都已经就位了，重新开始识别包裹，运动，抓取……
            #     else:
            #         self.get_logger().error(
            #             'WARN --- sort_package_tracking_3_2_cb latest_angle exceed max angle tolerance, retry')
            #         self.sort_package_tracking_1()
            # else:
            #     self.get_logger().error(
            #         'WARN --- sort_package_tracking_3_2_cb latest_location exceed max xy tolerance, retry')
            #     self.sort_package_tracking_1()
            self.sort_package_tracking_1()
        else:
            self.get_logger().error('ERROR --- sort_package_tracking_3_2_cb response.bridge_res is not set_location')
            self.sort_package_tracking_1()

    def sort_get_package_1(self, nearest_package):  # 通过arm抓取包裹
        package_x, package_y, package_z, package_cls = nearest_package
        release_package_x, release_package_y, release_package_z = self.config['package_release_xyz']
        self.tell_to_arm([['rs_grab', package_cls, package_x, package_y, package_z],
                          ["arm_release_package", package_cls, release_package_x, release_package_y,
                           release_package_z]],
                         self.sort_get_package_1_cb)

    def sort_get_package_1_cb(self,  future='just_wait'):  # arm给出了回复,有可能没抓到
        response = future.result()
        self.get_logger().info(
            f'INFO --- sort_get_package_1_cb response.basic_arm_cmd_res = {response.basic_arm_cmd_res}')
        if not response.basic_arm_cmd_res:
            self.get_logger().error('ERROR --- sort_get_package_1_cb response.basic_arm_cmd_res is False')
            self.tell_to_arm([['pos_center_headdown', 'x', 0, 0, 0]])  # 机械臂归位
            time.sleep(3)
            self.sort_package_tracking_1()
            return
        self.sort_get_package_3()

    def sort_get_package_3(self):
        # reverse for 2 s
        self.tell_to_car_base({"set_step": ['s', 0.1]})
        time.sleep(2)
        self.tell_to_car_base({"set_step": ['t', 0.0]})
        self.task_schedule_feedback("sort_get_package_finished")

    def sort_goto_cabinets_start(self):  # 发送语音，开始前往快递柜
        self.tell_to_database({"ask_for_free_warehouse": 0}, self.sort_goto_cabinets_0)
    
    def sort_goto_cabinets_0(self, future):
        response = future.result()
        self.get_logger().info(f'INFO --- sort_goto_cabinets_0 response.database_res = {response.database_res}')
        database_res_dict = json.loads(response.database_res)  # {"ask_for_free_warehouse": [cabinet_id, shelf_id, warehouse_id]}
        if 'ask_for_free_warehouse' in database_res_dict:
            self.taget_warehouse = database_res_dict['ask_for_free_warehouse']
            if self.taget_warehouse == 0:
                self.get_logger().info('INFO --- sort_goto_cabinets_0 no free warehouse ????')
                return
            self.tell_to_tts('开始前往快递柜', self.sort_goto_cabinets_1)
        else:
            self.get_logger().error('ERROR --- sort_goto_cabinets_0 response.database_res is not ask_for_free_warehouse')
            time.sleep(1)
            self.sort_goto_cabinets_start()

    def sort_goto_cabinets_1(self, future='just_wait'):  # 确定开始前往快递柜了
        id_cabinet = str(self.taget_warehouse[0])
        id_shelf = str(self.taget_warehouse[1])
        id_warehouse = str(self.taget_warehouse[2])
        target_x = self.config['sort_cabinets_xy'][id_cabinet][0] + self.config["sort_warehouse_distance"] * (
                    self.taget_warehouse[2] - 1)
        target_y = self.config['sort_cabinets_xy'][id_cabinet][1]
        target_angle = self.config['sort_cabinets_angle'][id_cabinet]
        # self.tell_to_car_base({"set_pose": [target_x, target_y, target_angle]}, self.sort_goto_cabinets_1_cb)
        self.tell_to_car_base({"set_pose": [target_x, target_y, target_angle]})  # 先不校验，直接发,下面开始读在车上的包裹二维码
        self.sort_goto_cabinets_2()
    
    def sort_goto_cabinets_2(self):  # 告诉机械臂要读车上的二维码
        self.tell_to_arm([["pos_read_qrcode", str(self.taget_warehouse[2]), 0, 0, 0]], self.sort_goto_cabinets_3)
    
    def sort_goto_cabinets_3(self, future='just_wait'):  # 机械臂运动到读二维码位置后的回复，下面校验一下是否读到了二维码
        while not self.srv_parcel_info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for srv_parcel_info service')
        request = ParcelInfo.Request()
        request.parcel_info_req = "parcel_info"
        self.get_logger().info(f'INFO --- sort_goto_cabinets_3 request.parcel_info_req = {request.parcel_info_req}')
        self.srv_parcel_info_client.call_async(request).add_done_callback(self.sort_goto_cabinets_3_cb)
    
    def sort_goto_cabinets_3_cb(self, future):  # srv_parcel_info_client给出了回复，是一个字典self.parcel_info = {"parcel_info": [string_data, qr_center_x, qr_center_y, qr_rs_d]}
        response = future.result()
        self.get_logger().info(f'INFO --- sort_goto_cabinets_3_cb response.parcel_info_res = {response.parcel_info_res}')
        parcel_info = json.loads(response.parcel_info_res)  # {"parcel_info": [string_data, qr_center_x, qr_center_y, qr_rs_d]}
        if len(parcel_info):  # 有parcel_info，说明看到了qr_code位置
            string_data, qr_center_x, qr_center_y, qr_rs_d = parcel_info["parcel_info"]
            self.parcel_code = string_data
            self.tell_to_arm([["pos_shelf_horizontal", str(self.taget_warehouse[2]), 0, 0, 0]])  # 让机械臂运动到看架子的姿态
            self.get_logger().info(f"INFO --- sort_goto_cabinets_3_cb tell_to_arm pos_shelf_horizontal***")
            self.sort_goto_cabinets_4()  # 已经知道parcel_code了，可以专心问小车是否到位了
        else:
            self.parcel_code = None  # 没有看到二维码，就不知道parcel_code了，只能再读一次
            self.get_logger().warn('WARN --- sort_goto_cabinets_3_cb response.parcel_info_res has no parcel_info')
            # self.tell_to_arm([['pos_shift', 'z', 'up', 0, 0]])  # 机械臂上升
            # time.sleep(0.1)
            # self.tell_to_arm([['pos_shift', 'z', 'stop', 0, 0]])  # 机械臂下降
            # self.sort_goto_cabinets_4()
            self.sort_goto_cabinets_3()  # 再读一次二维码
    
    def sort_goto_cabinets_4(self):  # 试过读取一次二维码后，问小车是否到位了
        self.tell_to_car_base({"get_state": 0}, self.sort_goto_cabinets_4_cb)
    
    def sort_goto_cabinets_4_cb(self, future):  # 小车给出了回复，下面校验一下是否到达指定位置
        response = future.result()
        self.get_logger().info(f'INFO --- sort_goto_cabinets_4_cb response.bridge_res = {response.bridge_res}')
        bridge_res = json.loads(response.bridge_res)  # 一个字典，正常应该是{"get_state": 0/1}
        if "get_state" in bridge_res:
            if bridge_res["get_state"] == 1:  # 小车到位了
                if self.parcel_code:  # 有parcel_code，说明看到了qr_code位置
                    self.task_schedule_feedback("sort_goto_cabinets_finished")
                else:  # 没有parcel_code，说明没有看到qr_code位置, 那就随机一个吧
                    self.get_logger().warn('WARN --- sort_goto_cabinets_4_cb no parcel_code, use random')
                    self.parcel_code = '00000' + str(self.taget_warehouse[2])
                    self.task_schedule_feedback("sort_goto_cabinets_finished")
            else:  # 小车没到位，重来一遍
                if self.parcel_code:  # 有parcel_code, 就只要等小车就行了
                    self.get_logger().warn('WARN --- sort_goto_cabinets_4_cb get_state is not 1, wait')
                    time.sleep(0.5)
                else:  # 没有parcel_code，说明没有看到qr_code位置, 那就立马重来一遍
                    self.get_logger().warn('WARN --- sort_goto_cabinets_4_cb get_state is not 1, also no parcel_code, retry')
                    self.sort_goto_cabinets_3()

    def sort_put_package_start(self):  # 发送语音，开始上架包裹
        self.tell_to_tts('开始上架', self.sort_put_package_1)
        self.no_shelf_qr_info_cnt = 0

    def sort_put_package_1(self, future="just_wait"):  # 到达指定地点开始上架包裹，首先要让机械臂升到对应层的高度
        id_shelf = str(self.taget_warehouse[2])
        self.tell_to_arm([["pos_shelf_horizontal", id_shelf, 0, 0, 0]], self.sort_put_package_1_cb)

    def sort_put_package_1_cb(self, future="just_wait"):  # 机械臂升到指定高度后的回复，由于需要机械臂运动到指定高度才能做下一步，所以要加一个过程
        self.sort_put_package_2()

    def sort_put_package_2(self):  # 机械臂升到指定高度后，通过srv_shelf_info_client获取taget_warehouse_qr的中心坐标，然后开始校准
        while not self.srv_shelf_info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for srv_shelf_info service')
        request = ShelfInfo.Request()
        request.shelf_info_req = "shelf_info"
        self.get_logger().info(f'INFO --- sort_put_package_2 request.shelf_info_req = {request.shelf_info_req}')
        self.srv_shelf_info_client.call_async(request).add_done_callback(self.sort_put_package_2_cb)

    def sort_put_package_2_cb(self, future):  # srv_shelf_info_client给出了回复，是一个字典self.shelf_info = 
        # {"qr_code_info": QRCode_info, "has_parcel": classify_results}, QRCode_info  =[string_data, qr_center_x, qr_center_y, qr_rs_d], classify_results = "has/no_parcel"
        response = future.result()
        self.get_logger().info(f'INFO --- sort_put_package_2_cb response.shelf_info_res = {response.shelf_info_res}')
        shelf_info = json.loads(
            response.shelf_info_res)  # {"qr_code_info": QRCode_info, "has_parcel": classify_results}
        if len(shelf_info):  # 有qr_code_info，说明看到了qr_code位置
            qr_code_info = shelf_info["qr_code_info"]
            classify_result = shelf_info["has_parcel"]
            # self.tell_to_car_base({"set_step": 't'})  # 先把小车底盘暂停，稳住
            self.sort_put_package_3(qr_code_info, classify_result)  # 开始教调
        else:
            self.no_shelf_qr_info_cnt += 1
            if self.no_shelf_qr_info_cnt == 10:  # 10次都没有看到qr_code位置，就调整小车的位置
                self.tell_to_car_base({"set_step": ['s', 0.1]})
                time.sleep(1.0)
                self.tell_to_car_base({"set_step": ['t', 0.0]})  # 小车停止
            elif self.no_shelf_qr_info_cnt == 20:
                self.tell_to_car_base({"set_step": ['w', 0.1]})
                time.sleep(1.5)
                self.tell_to_car_base({"set_step": ['t', 0.0]})  # 小车停止
            
            self.get_logger().error('ERROR --- sort_put_package_2_cb response.shelf_info_res has no qr_code_info')
            self.tell_to_arm([['pos_shift', 'z', 'stop', 0, 0]])  # 机械臂稳住
            self.tell_to_car_base({"set_step": ['t', 0.0]})  # 小车停止
            time.sleep(0.2)
            self.sort_put_package_2()

    def sort_put_package_3(self, qr_code_info, classify_result):  # 机械臂已经到达指定高度了，可以开始校准
        string_data, qr_center_x, qr_center_y, qr_rs_d = qr_code_info

        # 由于高度定死，所以，不要需要在乎y，只需要调整x和d

        # 再调整x
        qr_center_x_rate = qr_center_x / self.config["camera_width"]
        low_bar, high_bar = self.config["sort_put_calibrate_rate_x_range"]
        if low_bar <= qr_center_x_rate <= high_bar:
            # 小车停止
            self.tell_to_car_base({"set_step": ['t', 0.0]})
            # 最后调整d
            low_bar, high_bar = self.config["sort_put_calibrate_d_range"]
            if low_bar <= qr_rs_d <= high_bar:
                # xyd位置都已经就位了
                self.get_logger().info('INFO --- sort_put_package_3 calibrate ok !!!')
                self.sort_put_package_4(classify_result)  # 二维码的中心已经位于相机图片的正中心，而且面积大小也非常ok，可以开始检测是否有空位置放置包裹了
            else:
                self.get_logger().error(
                    'WARN --- sort_put_package_3 qr_rs_d exceed max sort_put_calibrate_d_range, adjusting')
                if qr_rs_d < low_bar:
                    self.get_logger().info(
                        f'INFO --- sort_put_package_3 qr_rs_d = {qr_rs_d} < low_bar, move backward')
                    self.tell_to_arm([['pos_shift', 'y', 'down', 0, 0]])
                else:
                    self.get_logger().info(
                        f'INFO --- sort_put_package_3 qr_rs_d = {qr_rs_d} > high_bar, move forward')
                    self.tell_to_arm([['pos_shift', 'y', 'up', 0, 0]])
                time.sleep(0.05)
                self.tell_to_arm([['pos_shift', 'y', 'stop', 0, 0]])
                time.sleep(0.05)
                self.sort_put_package_2()
                return
        else:
            self.get_logger().error(
                'WARN --- sort_put_package_3 qr_center_x_rate exceed max sort_put_calibrate_rate_x_range, adjusting')
            if qr_center_x_rate < low_bar:
                self.get_logger().info(
                    f'INFO --- sort_put_package_3 qr_center_x_rate = {qr_center_x_rate} < low_bar, let car move backward')
                self.tell_to_car_base({"set_step": ['s', 0.1]})
            else:
                self.get_logger().info(
                    f'INFO --- sort_put_package_3 qr_center_x_rate = {qr_center_x_rate} > high_bar, let car move forward')
                self.tell_to_car_base({"set_step": ['w', 0.1]})
            time.sleep(0.08)
            self.tell_to_car_base({"set_step": ['t', 0.0]})  # 小车停止
            self.sort_put_package_2()
            return

    def sort_put_package_4(self, classify_result):  # 机械臂已经校准完毕，可以开始检测是否有空位置放置包裹了

        self.get_logger().info('INFO --- sort_put_package_4 no matter has or not has parcel, i have the fucking database ! I can put package !')
        self.sort_put_package_5()
        # # DEBUG
        # time.sleep(10)
        # self.sort_put_package_2()

    def sort_put_package_5(self):  # 一切就绪，可以放包裹了
        self.tell_to_car_base({"set_step": ['t', 0.0]})  # 稳住小车
        self.tell_to_arm([['pos_shift', 'y', 'stop', 0, 0], ['pos_shift', 'z', 'stop', 0, 0]])  # 稳住机械臂
        time.sleep(0.5)
        self.tell_to_arm([['save_temp_pos', 'None', 0, 0, 0]], self.sort_put_package_5_cb)  # 保存当前位置

    def sort_put_package_5_cb(self, future):  # 机械臂保存当前位置后的回复
        response = future.result()
        self.get_logger().info(
            f'INFO --- sort_put_package_5_cb response.basic_arm_cmd_res = {response.basic_arm_cmd_res}')
        if response.basic_arm_cmd_res:
            self.sort_put_package_6()
        else:
            self.get_logger().error('ERROR --- sort_put_package_5_cb response.basic_arm_cmd_res is False')
            self.sort_put_package_5()

    def sort_put_package_6(self):  # 机械臂保存当前位置后，可以开始放包裹了
        self.tell_to_arm([['put_package', 'temp', 0, 0.25, 0.25]],
                         self.sort_put_package_6_cb)  # 机械臂放包裹，在temp的基础上x+0，y+0.2，z+0.3

    def sort_put_package_6_cb(self, future):  # 机械臂放包裹后的回复，下面校验一下是否放包裹成功
        response = future.result()
        self.get_logger().info(f'INFO --- sort_put_package_6 response.basic_arm_cmd_res = {response.basic_arm_cmd_res}')
        if response.basic_arm_cmd_res:
            self.sort_put_package_7()
        else:
            self.get_logger().error('ERROR --- sort_put_package_6 response.basic_arm_cmd_res is False')
            self.sort_put_package_6()

    def sort_put_package_7(self):  # 放完包裹后让小车倒退或前进
        # temp_x = self.config['sort_warehouse_distance']*(2-self.taget_warehouse[2])
        # temp_y = -0.2
        # temp_angle = 0.0 + self.config["sort_cabinets_angle"][str(self.taget_warehouse[0])]
        # self.tell_to_car_base({"set_pose_delta": [temp_x, temp_y, temp_angle, 1]}, self.sort_put_package_7_cb)
        if self.taget_warehouse[2] <= 3:  # 3以内，向后退
            move_distance = self.config['sort_warehouse_distance']*(self.taget_warehouse[2]) + 0.23
            move_speed = 0.3
            move_time = move_distance / move_speed
            self.tell_to_car_base({"set_step": ['s', move_speed]})
            time.sleep(move_time)
            self.tell_to_car_base({"set_step": ['t', 0.0]})
            self.sort_put_package_8()
        else:  # 3以上，向前进
            move_distance = self.config['sort_warehouse_distance']*(6 - self.taget_warehouse[2]) + 0.3
            move_speed = 0.3
            move_time = move_distance / move_speed
            self.tell_to_car_base({"set_step": ['w', move_speed]})
            time.sleep(move_time)
            self.tell_to_car_base({"set_step": ['t', 0.0]})
            self.sort_put_package_8()

    
    def sort_put_package_8(self):
        self.tell_to_database({"renew_database": [self.taget_warehouse[0], self.taget_warehouse[1], self.taget_warehouse[2], self.parcel_code]})  # 目前先定死快递单号是"000001"，之后可以读取二维码获得
        self.sort_goto_unloading_area_start()


    def sort_finished(self):
        # 全部分拣完成后发送语音
        while not self.srv_tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for tts service')
        self.get_logger().info('INFO --- sort_finished all packages finished !!!')
        tts_request = TTS.Request()
        tts_request.tts_req = '已分拣完全部包裹'
        self.srv_tts_client.call_async(tts_request).add_done_callback(self.sort_finished_tts_callback)

    def sort_finished_tts_callback(self, future):
        response = future.result()
        self.get_logger().info(f'INFO --- sort_finished_tts_callback response.tts_res = {response.tts_res}')
        self.task_schedule_feedback('sort_finished')  # 语音播报完，向task_schedule_server发送反馈"sort_finished"
        # 下面进入时刻等待新的包裹丢入

    def sort_package_wait_for_new_package(self):
        if self.global_status != 'sort_wait_for_new_package':  # 如果不是等待新包裹状态，就不用等了
            self.get_logger().info(f'INFO --- sort_package_wait_for_new_package global_status is {self.global_status}, return')
            return
        
        # 跟realsenese要最新的包裹信息
        while not self.srv_package_track_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for package_track service')
        request = PackageTrack.Request()
        request.package_track_req = 'package_track_rsxyzcls_list'  # 要arm坐标系下的xyz坐标，以及包裹的class id
        self.srv_package_track_client.call_async(request).add_done_callback(self.sort_package_wait_for_new_package_cb)
    
    def sort_package_wait_for_new_package_cb(self, future):  # realsense给出了最新的包裹信息，看是否有包裹
        if self.global_status != 'sort_wait_for_new_package':  # 如果不是等待新包裹状态，就不用等了
            self.get_logger().info(f'INFO --- sort_package_wait_for_new_package global_status is {self.global_status}, return')
            return
        response = future.result()
        self.get_logger().info(
            f'INFO --- sort_package_wait_for_new_package_cb response.package_track_res = {response.package_track_res}')
        package_track_rsxyzcls = json.loads(response.package_track_res)  # 一个列表的列表[[rs_x, rs_y, rs_z, cls]...]
        if len(package_track_rsxyzcls) == 0:  # 如果没有包裹，而且状态仍然是等待新包裹，就继续等
            if self.global_status == 'sort_wait_for_new_package':
                time.sleep(1)
                self.sort_package_wait_for_new_package()
            else:  # 已经不需要再等待新包裹了
                return
        else:  # 一旦有包裹，就开始分拣
            self.sort_package_tracking_1()




    def not_initialized(self):
        # 全部分拣完成后发送语音
        while not self.srv_tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for tts service')

        tts_request = TTS.Request()
        tts_request.tts_req = '系统检测到您尚未初始化，请先初始化系统。'
        self.srv_tts_client.call_async(tts_request).add_done_callback(self.not_initialized_tts_callback)

    def not_initialized_tts_callback(self, future):
        response = future.result()
        self.get_logger().info(f'INFO --- not_initialized_tts_callback response.tts_res = {response.tts_res}')

        self.task_schedule_feedback('not_initialized')  # 语音播报完，向task_schedule_server发送反馈"not_initialized"

    def read_config(self):
        with open(self.config_file_path, 'r') as json_file:
            data = json.load(json_file)
        return data

    def task_schedule_feedback(self, feedback):  # 给task_schedule_server发送反馈
        while not self.srv_task_schedule_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('srv_task_schedule not available, waiting again...')

        task_schedule_request = TaskSchedule.Request()  # 向srv_task_schedule发送反馈
        task_schedule_request.task_schedule_req = feedback
        self.srv_task_schedule_client.call_async(task_schedule_request)

    def tell_to_car_base(self, msg_dict, callback_func=None):  # 给小车底座发送信息，可以不需要回复
        while not self.srv_bridge_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for bridge service')
        request = Bridge.Request()
        request.bridge_req = json.dumps(msg_dict)
        self.get_logger().info(f'INFO --- tell_to_car_base request.bridge_req = {request.bridge_req}')
        if callback_func is not None:
            self.srv_bridge_client.call_async(request).add_done_callback(callback_func)
        else:
            self.srv_bridge_client.call_async(request)

    def tell_to_arm(self, msg_llist, callback_func=None):  # 给机械臂发送信息，可以不需要回复
        while not self.srv_basic_arm_cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for arm service')
        request = BasicArmCmd.Request()
        request.basic_arm_cmd_req = json.dumps(msg_llist)
        self.get_logger().info(f'INFO --- tell_to_arm request.basic_arm_cmd_req = {request.basic_arm_cmd_req}')
        if callback_func is not None:
            self.srv_basic_arm_cmd_client.call_async(request).add_done_callback(callback_func)
        else:
            self.srv_basic_arm_cmd_client.call_async(request)

    def tell_to_tts(self, msg_str, callback_func=None):
        while not self.srv_tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for tts service')
        request = TTS.Request()
        request.tts_req = msg_str
        self.get_logger().info(f'INFO --- tell_to_tts request.tts_req = {request.tts_req}')
        if callback_func is not None:
            self.srv_tts_client.call_async(request).add_done_callback(callback_func)
        else:
            self.srv_tts_client.call_async(request)
    
    def tell_to_database(self, msg_dict, callback_func=None):
        while not self.srv_database_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for database service')
        request = DataBase.Request()
        request.database_req = json.dumps(msg_dict)
        self.get_logger().info(f'INFO --- tell_to_database request.database_req = {request.database_req}')
        if callback_func is not None:
            self.srv_database_client.call_async(request).add_done_callback(callback_func)
        else:
            self.srv_database_client.call_async(request)
    
    def whether_stable_frame(self, track_rsxyzcls, threshold=0.05):  # 判断当前的包裹信息是否稳定
        if len(self.former_package_track_rsxyzcls):  # [[rs_x, rs_y, rs_z, cls]...]
            if len(self.former_package_track_rsxyzcls) == len(track_rsxyzcls):  # 首先包裹数量要一样
                # 统计前一次x,y,z方向的总和，和后一次x,y,z方向上的总和，两者之差不能大于阈值
                former_sum_x, former_sum_y, former_sum_z = 0, 0, 0
                for former_package in self.former_package_track_rsxyzcls:
                    former_sum_x += former_package[0]
                    former_sum_y += former_package[1]
                    former_sum_z += former_package[2]
                sum_x, sum_y, sum_z = 0, 0, 0
                for package in track_rsxyzcls:
                    sum_x += package[0]
                    sum_y += package[1]
                    sum_z += package[2]
                
                delta_x = abs(former_sum_x - sum_x)
                delta_y = abs(former_sum_y - sum_y)
                delta_z = abs(former_sum_z - sum_z)

                if delta_x < threshold and delta_y < threshold and delta_z < threshold:
                    self.get_logger().info(f'INFO --- whether_stable_frame in threshold delat_x = {delta_x}, delta_y = {delta_y}, delta_z = {delta_z}')
                    self.former_package_track_rsxyzcls = []
                    return True
                else:
                    self.get_logger().warn('WARN --- whether_stable_frame out of threshold')
                    self.former_package_track_rsxyzcls = track_rsxyzcls
                    return False
            else:  
                self.get_logger().warn('WARN --- whether_stable_frame length is not equal')
                self.former_package_track_rsxyzcls = track_rsxyzcls
                return False
        else:
            self.get_logger().warn('WARN --- whether_stable_frame former_package_track_rsxyzcls is empty')
            self.former_package_track_rsxyzcls = track_rsxyzcls
            return False

def main(args=None):
    rclpy.init(args=args)
    sort_node = SortNode('sort_node')
    try:
        rclpy.spin(sort_node)
    except KeyboardInterrupt:
        pass
    finally:
        sort_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
