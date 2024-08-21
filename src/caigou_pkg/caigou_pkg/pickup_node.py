# 创建class PickupNode, 并注册节点 pickup_node
# 1.创建msg_status订阅器，用于接收状态码，
# 2.创建srv_task_finished客户端，当初始化完成时，发送给ai_status_judge_node节点
# 3.创建srv_bridge客户端与小车底盘通信
# 4.创建srv_basic_arm_cmd_client客户端，BasicArmCmd服务接口，与arm进行互动
# 5.创建srv_package_track客服端，跟realsense要包裹追踪的信息

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from caigou_interfaces.srv import TTS, TaskSchedule, Bridge, BasicArmCmd, WhetherArmReachable, PackageTrack, ParcelInfo, \
    ShelfInfo, DataBase
import json
import time
import numpy as np
from functools import partial


class PickupNode(Node):
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

        # pickup任务的配置, 在pickup_xxx状态时读取
        self.config = {}

        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')

    def msg_status_callback(self, msg):
        # self.get_logger().info(f'INFO --- received msg_status = {msg.data}')
        if 'pickup' in msg.data:  # 判断发出的status是否是pickup类，如果不是可以直接忽略了
            self.get_logger().info(f'INFO --- received msg_status = {msg.data}, validating whether initialized')

            config = self.read_config()
            if not config['init']:  # 没有初始化，无法执行后续任务
                self.not_initialized()
                return
            self.config = config  # 读取最新全部配置

            self.global_status = msg.data  # 记录当前的状态

            # 判断应该是pickup的哪一步了
            if msg.data == 'pickup_start':  # 最初的收到人为指令
                self.pickup_start()
            elif msg.data == 'pickup_goto_loading_area':  # 前往寄件区->pickup_goto_loading_area_finished
                self.pickup_goto_loading_area_start()
            elif msg.data == 'pickup_package_tracking':  # 追踪捡起包裹->pickup_package_tracking_finished
                self.pickup_package_tracking_start()
            elif msg.data == 'pickup_goto_unloading_area':  # 前往揽货区->pickup_goto_unloading_area_finished
                self.pickup_goto_unloading_area_start()
            elif msg.data == 'pickup_put_package':  # 开始放下包裹->pickup_put_package_finished
                self.pickup_put_package_start()
            elif msg.data == 'pickup_wait_for_new_package':  # 等待新寄件包裹
                self.pickup_package_wait_for_new_package()
            else:
                return
        else:
            self.global_status = "free"

    def pickup_start(self):  # 收到人为指令，开始处理寄件
        self.express_company_name = None
        self.tell_to_tts('开始处理寄件包裹', self.pickup_goto_loading_area_start)  # 真实语音：下面开始处理寄件包裹

    def pickup_goto_loading_area_start(self, future='just_wait'):  # 确定开始处理寄件了，先去寄件区pickup_loading_area
        self.no_package_count = 0
        self.tell_to_tts('开始前往寄件区', self.pickup_goto_loading_area_1)  # 真实语音：下面开始前往寄件区

    def pickup_goto_loading_area_1(self, future='just_wait'):  # 确定开始处理寄件了，先通过bridge给小车发送最终要去的地方xy和角度
        self.tell_to_car_base({"set_pose": [self.config['pickup_loading_area_xy'][0],
                                            self.config['pickup_loading_area_xy'][1],
                                            self.config['pickup_loading_area_angle']]},
                              self.pickup_goto_loading_area_1_cb)

    def pickup_goto_loading_area_1_cb(self, future):  # 小车给出了回复，下面简单校验一下
        response = future.result()
        self.get_logger().info(f'INFO --- pickup_goto_loading_area_1_cb response.bridge_res = {response.bridge_res}')
        try:
            bridge_res = json.loads(response.bridge_res)  # 一个字典，正常应该是{"set_pose": [latest_x,y,angle]}
            latest_x, latest_y, latest_angle = bridge_res['set_pose']
            self.task_schedule_feedback("pickup_goto_loading_area_finished")
        except Exception as e:
            self.get_logger().error(
                f'ERROR --- pickup_goto_loading_area_1_cb response.bridge_res is not json, {str(e)}')
            time.sleep(1)
            self.pickup_goto_loading_area_1_()

    def pickup_package_tracking_start(self):  # 小车到了
        self.tell_to_tts('开始搬运寄件包裹', self.pickup_package_tracking_1)  # 真实语音：开始搬运包裹

    def pickup_package_tracking_1(self, future="just_wait"):  # 开始调用摄像头，获取包裹信息
        # 跟realsenese要最新的包裹跟踪信息
        while not self.srv_package_track_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for package_track service')
        request = PackageTrack.Request()
        request.package_track_req = 'package_track_rsxyzcls_list'  # 要arm坐标系下的xyz坐标，以及包裹的class
        self.srv_package_track_client.call_async(request).add_done_callback(self.pickup_package_tracking_1_cb)

    def pickup_package_tracking_1_cb(self, future):  # realsense给出了最新的包裹信息，找到距离最近的包裹，发送请求是否能够抓取
        response = future.result()
        self.get_logger().info(
            f'INFO --- pickup_package_tracking_1 response.package_track_res = {response.package_track_res}')
        package_track_rsxyzcls = json.loads(response.package_track_res)  # 一个列表的列表[[arm_x, arm_y, arm_z, cls, id]...]

        if len(package_track_rsxyzcls) == 0:  # 如果没有包裹，超过10次就直接结束
            if self.no_package_count >= 10:
                self.pickup_finished()
                return
            time.sleep(0.2)
            self.no_package_count += 1
            self.pickup_package_tracking_1()
        else:
            if not self.whether_stable_frame(package_track_rsxyzcls):
                self.pickup_package_tracking_1()
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
                self.get_logger().error('ERROR --- pickup_package_tracking_1 no nearest package')
                self.pickup_package_tracking_1()
                return

            # 判断最近的包裹是否在机械臂可抓取的范围self.arm_reachable_range = (min, max)
            self.get_logger().info(
                f'INFO --- pickup_package_tracking_1 nearest_package = {nearest_package}, min_distance = {min_distance}')
            self.pickup_package_tracking_2(nearest_package)

    def pickup_package_tracking_2(self, nearest_package):  # 问一下机械臂能不能抓
        # 判断最近的包裹是否在机械臂可抓取的范围self.arm_reachable_range = (min, max)
        while not self.srv_whether_arm_reahable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for whether_arm_reachable service')
        req = WhetherArmReachable.Request()
        req.rs_x = float(nearest_package[0])
        req.rs_y = float(nearest_package[1])
        req.rs_z = float(nearest_package[2])
        callback = callback = partial(self.pickup_package_tracking_2_cb, nearest_package)
        self.srv_whether_arm_reahable_client.call_async(req).add_done_callback(callback)

    def pickup_package_tracking_2_cb(self, nearest_package, future):  # 机械臂的回答
        response = future.result()
        reachable = response.arm_reachable
        if reachable:  # 在机械臂可抓范围内，直接抓取
            self.pickup_get_package_1(nearest_package)
        else:  # 不在机械臂可抓范围内，需要结合当前小车的位置&角度，和我需要的小车shift的x&y，来计算新的目标位置
            arm_x_shift = response.arm_x_shift
            arm_y_shift = response.arm_y_shift
            self.pickup_package_tracking_3_1(arm_x_shift, arm_y_shift)

    def pickup_package_tracking_3_1(self, arm_x_shift, arm_y_shift):  # 不能直接抓，要结合小车位姿，进行计算调整
        callback = partial(self.pickup_package_tracking_3_1_cb, arm_x_shift, arm_y_shift)
        self.tell_to_car_base({"get_pose": 0}, callback)

    def pickup_package_tracking_3_1_cb(self, arm_x_shift, arm_y_shift, future):  # 小车给出了回复，校验一下是不是 get_pose
        response = future.result()
        self.get_logger().info(f'INFO --- pickup_package_tracking_3_1_cb response.bridge_res = {response.bridge_res}')
        bridge_res = json.loads(response.bridge_res)  # 一个字典，正常应该是{"get_location": [latest_x,y]}
        if "get_pose" in bridge_res:
            car_latest_x, car_latest_y, car_latest_angle = bridge_res["get_pose"]
            self.pickup_package_tracking_3_2(arm_x_shift, arm_y_shift, car_latest_x, car_latest_y, car_latest_angle)
        else:
            self.get_logger().error('ERROR --- pickup_package_tracking_3_1_cb response.bridge_res is not get_location')
            self.pickup_package_tracking_3_1(arm_x_shift, arm_y_shift)

    def pickup_package_tracking_3_2(self, arm_x_shift, arm_y_shift, car_latest_x, car_latest_y,
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
        callback = partial(self.pickup_package_tracking_3_2_cb, self.config["pickup_loading_area_angle"],
                           target_y_car)
        self.tell_to_car_base(
            {"set_pose_delta": [x_offset_car, y_offset_car, self.config["pickup_loading_area_angle"], 1]},
            callback)  # 现在只需要发世界坐标系的delta就行

    def pickup_package_tracking_3_2_cb(self, target_x_car, target_y_car, future):  # 小车给出了回复，下面校验一下是否到达了新的目标位置
        response = future.result()
        self.get_logger().info(f'INFO --- pickup_package_tracking_3_2_cb response.bridge_res = {response.bridge_res}')
        bridge_res = json.loads(response.bridge_res)  # 一个字典，正常应该是{"set_location": [latest_x,y]}
        if "set_pose_delta" in bridge_res:  # 到了新位置，从头来一遍，跟相机要新的包裹信息
            self.pickup_package_tracking_1()
        else:
            self.get_logger().error('ERROR --- pickup_package_tracking_3_2_cb response.bridge_res is not set_location')
            self.pickup_package_tracking_1()

    def pickup_get_package_1(self, nearest_package):  # 可以直接抓取
        package_x, package_y, package_z, package_cls = nearest_package
        release_package_x, release_package_y, release_package_z = self.config['package_release_xyz']
        self.tell_to_arm([['rs_grab', package_cls, package_x, package_y, package_z],
                          ["arm_release_package", package_cls, release_package_x, release_package_y,
                           release_package_z]],
                         self.pickup_get_package_1_cb)

    def pickup_get_package_1_cb(self, future="just_wait"):  # arm给出了回复,默认抓完了
        response = future.result()
        self.get_logger().info(f'INFO --- pickup_get_package_1_cb response.basic_arm_cmd_res = {response.basic_arm_cmd_res}')
        if not response.basic_arm_cmd_res:
            self.get_logger().error('ERROR --- pickup_get_package_1_cb response.basic_arm_cmd_res is False')
            self.tell_to_arm([['pos_center_headdown', 'x', 0, 0, 0]])  # 机械臂归位
            time.sleep(3)
            self.pickup_package_tracking_1()  # 抓取失败，重新来一遍
            return
        else:
            self.pickup_get_package_2()  # 让小车后退一点

    def pickup_get_package_2(self):  # 抓完了，后退一点，同时让机械臂运动到读qrcode的位置
        self.tell_to_car_base({"set_step": ['s', 0.1]})
        self.tell_to_arm([["pos_read_qrcode", 'pickup', 0, 0, 0]])
        time.sleep(2)
        self.tell_to_car_base({"set_step": ['t', 0.0]})
        self.task_schedule_feedback("pickup_get_package_finished")

    def pickup_goto_unloading_area_start(self):  # 这里要先知道包裹单号，问数据库是哪个公司，所以先跟摄像头要二维码信息
        self.tell_to_tts("开始前往揽货区")  # 真实语音：开始前往揽货区
        self.pickup_goto_unloading_area_1()

    def pickup_goto_unloading_area_1(self):  # 跟摄像头要包裹信息
        while not self.srv_parcel_info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for srv_parcel_info service')
        request = ParcelInfo.Request()
        request.parcel_info_req = "parcel_info"
        self.get_logger().info(
            f'INFO --- pickup_goto_unloading_area_1 request.parcel_info_req = {request.parcel_info_req}')
        self.srv_parcel_info_client.call_async(request).add_done_callback(self.pickup_goto_unloading_area_1_cb)

    def pickup_goto_unloading_area_1_cb(self, future):
        response = future.result()
        self.get_logger().info(
            f'INFO --- pickup_goto_unloading_area_1_cb response.parcel_info_res = {response.parcel_info_res}')
        parcel_info = json.loads(
            response.parcel_info_res)  # {"parcel_info": [string_data, qr_center_x, qr_center_y, qr_rs_d]}
        if len(parcel_info):  # 有parcel_info，说明看到了qr_code位置
            string_data, qr_center_x, qr_center_y, qr_rs_d = parcel_info["parcel_info"]
            self.parcel_code = string_data
            if self.parcel_code[0] == '0':
                self.pickup_goto_unloading_area_2()
            else:
                self.get_logger().error('ERROR --- pickup_goto_unloading_area_1_cb response.parcel_info_res is not 0xxxx')
                time.sleep(0.1)
                self.pickup_goto_unloading_area_1()
                return

            # DEBUG
            # if not self.express_company_name:
            #     self.express_company_name = 'A'
            # else:
            #     if self.express_company_name == 'A':
            #         self.express_company_name = 'B'
            #     else:
            #         self.express_company_name = 'A'
            # self.pickup_goto_unloading_area_3()

        else:
            self.parcel_code = None  # 没有看到二维码，就不知道parcel_code了，只能，再读一次
            self.get_logger().warn(
                'WARN --- pickup_goto_unloading_area_1_cb response.parcel_info_res has no parcel_info')
            self.pickup_goto_unloading_area_1()  # 再读一次二维码

    def pickup_goto_unloading_area_2(self):  # 跟database要parcel code对应的公司称，是A还是B？
        self.tell_to_database({"ask_for_express_company_name": self.parcel_code},
                              self.pickup_goto_unloading_area_2_cb)  # parcel_code是一个字符串string

    def pickup_goto_unloading_area_2_cb(self, future):  # 收到database回的消息了
        response = future.result()
        self.get_logger().info(
            f'INFO --- pickup_goto_unloading_area_2_cb response.database_res = {response.database_res}')
        database_res_dict = json.loads(
            response.database_res)  # {"ask_for_express_company_name": 'A'/'B'}
        if 'ask_for_express_company_name' in database_res_dict:
            self.express_company_name = database_res_dict['ask_for_express_company_name']  #
            if self.express_company_name == 'A' or self.express_company_name == 'B':
                self.pickup_goto_unloading_area_3()
            else:
                self.get_logger().error(
                    'ERROR --- pickup_goto_unloading_area_2_cb response.database_res is not A or B')
                time.sleep(0.1)
                self.pickup_goto_unloading_area_2()  # 重新读一次二维码
        else:
            self.get_logger().error(
                'ERROR --- pickup_goto_unloading_area_2_cb response.database_res is not ask_for_express_company_name')
            time.sleep(1)
            self.pickup_goto_unloading_area_2()

    def pickup_goto_unloading_area_3(self):  # 告诉小车要去哪个公司
        target_unloading_area_xy = self.config["pickup_unloading_area_xy"][self.express_company_name]
        target_unloading_area_angle = self.config["pickup_unloading_area_angle"][self.express_company_name]
        self.tell_to_car_base(
            {"set_pose": [target_unloading_area_xy[0], target_unloading_area_xy[1], target_unloading_area_angle]})
        # 不用等待小车的回应，直接去抓车上的包裹，等车一到，丢下就行
        self.pickup_goto_unloading_area_4()  # 控制机械臂抓取车上的包裹

    def pickup_goto_unloading_area_4(self):  # 小车在运动的过程中，机械臂来抓取车上的包裹
        self.tell_to_arm([["pickup_grab_package", self.express_company_name, 0, 0, 0]],
                         self.pickup_goto_unloading_area_4_cb)

    def pickup_goto_unloading_area_4_cb(self, future="just_wait"):  # 机械臂已经抓取到了，可以问小车有没有到了
        self.pickup_goto_unloading_area_5()

    def pickup_goto_unloading_area_5(self):  # 询问小车的状态
        self.tell_to_car_base({"get_state": 0}, self.pickup_goto_unloading_area_5_cb)

    def pickup_goto_unloading_area_5_cb(self, future):
        response = future.result()
        self.get_logger().info(f'INFO --- pickup_goto_unloading_area_5_cb response.bridge_res = {response.bridge_res}')
        bridge_res = json.loads(response.bridge_res)  # 一个字典，正常应该是{"get_state": 0/1}
        if "get_state" in bridge_res:
            if bridge_res["get_state"] == 1:  # 小车到位了
                self.task_schedule_feedback("pickup_goto_unloading_area_finished")
            else:  # 小车没到位，重来一遍
                time.sleep(0.5)
                self.get_logger().warn(
                    'WARN --- pickup_goto_unloading_area_5_cb get_state is not 1, also no parcel_code, retry')
                self.pickup_goto_unloading_area_5()

    def pickup_put_package_start(self):  # 发送语音，开始放置包裹
        self.tell_to_tts('开始放置包裹', self.pickup_put_package_1)  # 真实语音：开始放置包裹

    def pickup_put_package_1(self, future="just_wait"):  # 告诉机械臂放包裹
        self.tell_to_arm([["pickup_release_package", self.express_company_name, 0, 0, 0]], self.pickup_put_package_1_cb)

    def pickup_put_package_1_cb(self, future="just_wait"):  # 丢完了，让车后退2s，重头再来一遍
        self.tell_to_car_base({"set_step": ['s', 0.1]})
        time.sleep(2)
        self.tell_to_car_base({"set_step": ['t', 0.0]})
        self.pickup_goto_loading_area_start()

    def pickup_finished(self):  # 超过十次没检测到包裹，就认为没有了，进入等待新的寄件流程
        self.tell_to_tts("已处理完全部寄件包裹", self.pickup_finished_cb)  # 真实语音： 已处理完全部包裹

    def pickup_finished_cb(self, future="just_wait"):
        self.task_schedule_feedback("pickup_finished")

    def pickup_package_wait_for_new_package(self):  # 等待新的寄件包裹
        if self.global_status != 'pickup_wait_for_new_package':
            self.get_logger().warn(f'WARN --- pickup_package_wait_for_new_package global_status = {self.global_status}')
            return
        # 跟realsenese要最新的包裹信息
        while not self.srv_package_track_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for package_track service')
        request = PackageTrack.Request()
        request.package_track_req = 'package_track_rsxyzcls_list'  # 要arm坐标系下的xyz坐标，以及包裹的class id
        self.srv_package_track_client.call_async(request).add_done_callback(self.pickup_package_wait_for_new_package_cb)

    def pickup_package_wait_for_new_package_cb(self, future):  # realsense给出了最新的包裹信息，看看有没有新的包裹
        if self.global_status != 'pickup_wait_for_new_package':
            self.get_logger().warn(f'WARN --- pickup_package_wait_for_new_package global_status = {self.global_status}')
            return
        response = future.result()
        self.get_logger().info(
            f'INFO --- pickup_package_wait_for_new_package_cb response.package_track_res = {response.package_track_res}')
        package_track_rsxyzcls = json.loads(response.package_track_res)  # 一个列表的列表[[rs_x, rs_y, rs_z, cls]...]
        if len(package_track_rsxyzcls) == 0:
            if self.global_status == 'pickup_wait_for_new_package':
                time.sleep(1)
                self.pickup_package_wait_for_new_package()
            else:  # 如果状态不对了，就不再等待了
                self.get_logger().warn(
                    f'WARN --- pickup_package_wait_for_new_package global_status = {self.global_status}')
                return
        else:  # 出现了新的包裹，重头再来一遍
            self.get_logger().info(
                f'INFO --- pickup_package_wait_for_new_package new package_track_rsxyzcls = {package_track_rsxyzcls}')
            self.pickup_package_tracking_1()

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
    pickup_node = PickupNode('pickup_node')
    try:
        rclpy.spin(pickup_node)
    except KeyboardInterrupt:
        pass
    finally:
        pickup_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
