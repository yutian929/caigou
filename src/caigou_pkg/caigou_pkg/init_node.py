# 创建class InitNode, 并注册节点 init_node
# 1.创建msg_status订阅器，用于接收状态码，当状态码为init时，顺序执行初始化过程
# 2.创建srv_task_finished客户端，当初始化完成时，发送给ai_status_judge_node节点

# 重新配写config.json, 或者程序员操作位置
import json


def programmer_set_config(data=None):
    if not data:
        data = {  # 单位都是m
            "init": True,  # 是否完成初始化

            "camera_height": 480,
            "camera_width": 640,

            "arm_reachable_range": (0.25, 0.43),  # 机械臂可达范围xy,arm坐标系下，单位m
            "ideal_package_xy": (0.37, 0.0),  # 理想的包裹位置，arm坐标系下，单位m
            "package_release_xyz": (-0.25, 0.0, 0.20),  # 抓取包裹后的释放位置，arm坐标系下，单位m

            "sort_unloading_area_xy": (0.85, -0.53),  # 快递卸货区的坐标
            "sort_unloading_area_angle": -90.0,  # 快递卸货区小车的角度

            "sort_cabinets_xy": {"1": (0.3, 1.88), },  # 快递分拣柜的坐标
            "sort_warehouse_distance": 0.28,  # 快递分拣柜上每个仓之间的距离
            "sort_cabinets_angle": {"1": 0.0, },  # 快递分拣柜小车的角度
            "sort_cabinets_height": {"1": 0.27, "2": 0.13, "3": -0.1, },  # 快递分拣柜不同层的高度

            "sort_put_calibrate_rate_x_range": (0.4, 0.6),  # 分拣放置校准时x方向的范围
            "sort_put_calibrate_rate_y_range": (0.4, 0.6),  # 分拣放置校准时y方向的范围
            "sort_put_calibrate_d_range": (0.15, 0.23),  # 分拣放置校准时d方向的范围,单位m

            "angle_tolerance": 10,  # 角度误差容忍度
            "xy_tolerance": 0.1,  # 坐标误差容忍度

            "pickup_loading_area_xy":(0.1, 0.6),  # 用户寄件区
            "pickup_loading_area_angle": 180.0,
            "pickup_unloading_area_xy": {"A": (1.8, -0.1), "B":(1.8, 0.6)},  # 公司揽货区
            "pickup_unloading_area_angle": {"A": 0.0, "B": 0.0},
            
            # "qr_position_tolerance_x": 50,  # 二维码定位时xy误差容忍度，单位pixel,
            # "qr_position_tolerance_y": 20,
            # "qr_position_standard_size": 110*110,  # 正常定位时的二维码大小，单位pixel^2
            # "qr_position_tolerance_size": 4000,  # 二维码大小误差容忍度，单位pixel^2

            "others": {},
        }

    # 文件名
    file_name = "config.json"

    # 将字典写入JSON文件
    with open(file_name, 'w') as json_file:
        json.dump(data, json_file)

    print("Successfully written to", file_name)


programmer_set_config()  # 初始化时，先写入config.json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from caigou_interfaces.srv import TTS, TaskSchedule
import time


class InitNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # 创建srv_tts客户端
        self.srv_tts_client = self.create_client(TTS, 'srv_tts')

        # 创建srv_task_schedule客户端
        self.srv_task_schedule_client = self.create_client(TaskSchedule, 'srv_task_schedule')

        # 创建msg_status订阅器
        self.msg_status_sub = self.create_subscription(String, 'msg_status', self.msg_status_callback, 10)

        # 配置文件
        self.config_file_path = "/home/caigou/workspace/caigou_ws/config.json"

        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')

    def msg_status_callback(self, msg):
        # self.get_logger().info(f'INFO --- received msg_status = {msg.data}')
        if 'init_start' in msg.data:  # 只需要判断发出的status是否是init_start，其他的不用管
            self.get_logger().info(f'INFO --- received msg_status = {msg.data}, start init process')
            self.init_start()

    def init_start(self):  # 发送语音，开始初始化
        while not self.srv_tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for tts service')

        tts_request = TTS.Request()
        tts_request.tts_req = '开始初始化'
        self.srv_tts_client.call_async(tts_request).add_done_callback(self.init_process)  # 语音播报完成后进入初始化过程

    def init_process(self, future):
        time.sleep(1)  # 初始化过程，先pass
        programmer_set_config()

        data = self.read_config()  # 重读一遍，看写入是否有误
        self.get_logger().info(f'INFO --- read config file: \n{data}')

        self.init_finished()

    def init_finished(self):
        while not self.srv_tts_client.wait_for_service(timeout_sec=1.0):  # 初始化完成后发送语音
            self.get_logger().info('INFO --- waiting for tts service')
        tts_request = TTS.Request()
        tts_request.tts_req = '初始化完成'
        self.srv_tts_client.call_async(tts_request).add_done_callback(self.init_finished_tts_callback)

    def init_finished_tts_callback(self, future):
        response = future.result()
        self.get_logger().info(f'INFO --- init_finished_tts_callback response.tts_res = {response.tts_res}')
        self.task_schedule_feedback('init_finished')  # 发送反馈init_finished给task_schedule_server

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


def main(args=None):
    rclpy.init(args=args)
    init_node = InitNode('init_node')
    try:
        rclpy.spin(init_node)
    except KeyboardInterrupt:
        pass
    finally:
        init_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
