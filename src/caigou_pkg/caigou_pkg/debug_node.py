# 编写一个简单的debug节点，测试srv_database的服务
# 1. 该节点会调用srv_database服务，发送一个json格式的请求，然后打印返回的json格式的数据

import rclpy
from rclpy.node import Node
from caigou_interfaces.srv import DataBase
import json
import time

class DebugNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # 创建srv_database客户端
        self.srv_database_client = self.create_client(DataBase, 'srv_database')
        while not self.srv_database_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service srv_database not available, waiting again...')
        
        self.warehouse_idx = []
        self.debug_srv_database_1()

        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')

    def debug_srv_database_1(self):
        req = DataBase.Request()
        req.database_req = json.dumps({"ask_for_free_warehouse": 0})
        self.get_logger().info(f'INFO --- req.database_req: {req.database_req}')
        future = self.srv_database_client.call_async(req)
        future.add_done_callback(self.debug_srv_database_1_cb)

    def debug_srv_database_1_cb(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'INFO --- response.database_res: {response.database_res}')
            ori_data = json.loads(response.database_res)
            self.warehouse_idx = ori_data['ask_for_free_warehouse']
            if len(self.warehouse_idx) == 0:
                self.get_logger().info(f'INFO --- no free warehouse')
                time.sleep(3)
                self.debug_srv_database_1()
        except Exception as e:
            self.get_logger().error(f'ERROR --- Service call failed: {str(e)}')
        
        time.sleep(3)
        self.debug_srv_database_2()
    
    def debug_srv_database_2(self):
        req = DataBase.Request()
        req.database_req = json.dumps({"renew_database": [self.warehouse_idx[0], self.warehouse_idx[1], self.warehouse_idx[2], f"00000{self.warehouse_idx[2]}"]})
        self.get_logger().info(f'INFO --- req.database_req: {req.database_req}')
        future = self.srv_database_client.call_async(req)
        future.add_done_callback(self.debug_srv_database_2_cb)
    
    def debug_srv_database_2_cb(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'INFO --- response.database_res: {response.database_res}')
        except Exception as e:
            self.get_logger().error(f'ERROR --- Service call failed: {str(e)}')
        
        time.sleep(3)
        self.debug_srv_database_3()

    def debug_srv_database_3(self):
        req = DataBase.Request()
        req.database_req = json.dumps({"renew_status": f"free{self.warehouse_idx[2]}"})
        self.get_logger().info(f'INFO --- req.database_req: {req.database_req}')
        future = self.srv_database_client.call_async(req)
        future.add_done_callback(self.debug_srv_database_3_cb)

    def debug_srv_database_3_cb(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'INFO --- response.database_res: {response.database_res}')
        except Exception as e:
            self.get_logger().error(f'ERROR --- Service call failed: {str(e)}')

        time.sleep(3)
        self.debug_srv_database_1()


    def stop(self):
        self.get_logger().info(f'INFO --- {self.get_name()} stopped ^.^')


def main(args=None):
    rclpy.init(args=args)
    debug_node = DebugNode('debug_node')
    try:
        rclpy.spin(debug_node)
    except KeyboardInterrupt:
        pass
    finally:
        debug_node.stop()
        debug_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from caigou_interfaces.srv import PackageTrack, ShelfInfo
# import json
# import numpy as np

# # #DEBUG
# import time
# import os
# from caigou_interfaces.srv import BasicArmCmd
# # import cv2.aruco as aruco  # 4.5.5.64：opencv-python-4.5.5.64 opencv-contrib-python-4.5.5.64
# # from scipy.spatial.transform import Rotation as R

# class DebugNode(Node):
#     def __init__(self, node_name):
#         super().__init__(node_name)

#         # 创建ShelfInfo客户端
#         self.srv_shelf_info_client = self.create_client(ShelfInfo, 'srv_shelf_info')

#         # waveshare_arm fuck you !!!
        
#         self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')
#         self.target_qr_code = '1-2-1'
#         self.sort_put_package_0()

#     def sort_put_package_0(self):
#         self.get_logger().info(f"tell the damn arm to move to shelf 2 hori !!!")
#         time.sleep(3)
#         self.sort_put_package_1()
    
#     def sort_put_package_1(self):
#         # 1. 获取货架信息
#         req = ShelfInfo.Request()
#         req.shelf_info_req = 'shelf_info'
#         while not self.srv_shelf_info_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().warn('service srv_shelf_info not available, waiting again...')
#         self.srv_shelf_info_client.call_async(req).add_done_callback(self.sort_put_package_1_cb)

#     def sort_put_package_1_cb(self, future):
#         response = future.result()
#         # self.get_logger().info(f'INFO --- shelf_info: {response.shelf_info_res}')
#         shelf_info = json.loads(response.shelf_info_res)  # {"qr_code_info": QRCode_info, "has_parcel": classify_results}
#         if len(shelf_info):
#             string_data, qr_center_x, qr_center_y, qr_rs_d = shelf_info['qr_code_info']
#             classify_result = shelf_info['has_parcel']
#             self.get_logger().info(f'INFO --- qr_code: {string_data}')
#             self.get_logger().info(f'INFO --- classify_result: {classify_result}')
#             if string_data == self.target_qr_code:  # 如果找到了目标货仓
#                 if classify_result == "has_parcel":
#                     self.get_logger().info(f'INFO --- {self.target_qr_code} has parcel, move on')
#                     self.target_qr_code = self.target_qr_code[:-1] + str(int(self.target_qr_code[-1]) + 1)
#                     self.sort_put_package_1()
#                 else:
#                     self.get_logger().info(f'INFO --- {self.target_qr_code} has no parcel, put parcel')
#                     self.sort_put_package_2()
#             else:  # 如果没有找到目标货仓,但是也发现了二维码，则根据这个二维码进行移动
#                 now_warehhouse_idx = string_data[-1]
#                 taget_warehhouse_idx = self.target_qr_code[-1]
#                 if now_warehhouse_idx < taget_warehhouse_idx:
#                     self.get_logger().info(f'INFO --- idx is small move to the right')
#                 else:
#                     self.get_logger().info(f'INFO --- idx is large move to the left')
#                 self.sort_put_package_1()
#         else:
#             self.get_logger().info(f'INFO --- no qr_code_info, move on')
#             time.sleep(1)
#             self.sort_put_package_1()

#     def sort_put_package_2(self):
#         # 1. 获取货架信息
#         req = ShelfInfo.Request()
#         req.shelf_info_req = 'shelf_info'
#         while not self.srv_shelf_info_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().warn('service srv_shelf_info not available, waiting again...')
#         self.srv_shelf_info_client.call_async(req).add_done_callback(self.sort_put_package_2_cb)
    
#     def sort_put_package_2_cb(self, future):
#         response = future.result()
#         # self.get_logger().info(f'INFO --- shelf_info: {response.shelf_info_res}')
#         shelf_info = json.loads(response.shelf_info_res)
#         if len(shelf_info):
#             string_data, qr_center_x, qr_center_y, qr_rs_d = shelf_info['qr_code_info']
#             # 对齐二维码
#             whether_ok = self.align_qr_code(qr_center_x, qr_center_y, qr_rs_d)
#             if whether_ok:
#                 self.get_logger().info(f'INFO --- align qr_code successfully')
#                 self.sort_put_package_3()
#             else:
#                 # self.get_logger().info(f'INFO --- align qr_code failed')
#                 self.sort_put_package_2()
    
#     def sort_put_package_3(self):  # 已经对准了二维码，记录当前机械臂的位置，执行固定操作
#         # req = BasicArmCmd.Request()
#         # req.basic_arm_cmd_req = json.dumps([['save_temp_pos', '', 0, 0, 0], ['arm_grab', 'xxx', -0.15, 0.0, 0.0], ['arm_place_temp_base', 'xxx', 0, 0.2, 0.15]])
#         # while not self.srv_basic_arm_cmd_client.wait_for_service(timeout_sec=1.0):
#         #     self.get_logger().warn('service srv_basic_arm_cmd not available, waiting again...')
#         # self.srv_basic_arm_cmd_client.call_async(req).add_done_callback(self.sort_put_package_3_cb)
#         self.get_logger().info(f'INFO --- tell the damn arm to grab the parcel and place it to the base')
#         time.sleep(3)
#         self.sort_put_package_3_cb(None)
    
#     def sort_put_package_3_cb(self, future):
#         self.get_logger().info(f'INFO --- parcel has been placed to the base')
#         # self.sort_put_package_4()

#     def align_qr_code(self, qr_center_x, qr_center_y, qr_rs_d):
#         # 先控制x方向
#         self.ideal_x = 0.5
#         self.ideal_x_tolerance = 0.1
#         now_x = qr_center_x/640
#         if now_x < self.ideal_x - self.ideal_x_tolerance:
#             self.get_logger().info(f'INFO --- now_x = {now_x} move to the right')
#             return False
#         elif now_x > self.ideal_x + self.ideal_x_tolerance:
#             self.get_logger().info(f'INFO --- now_x = {now_x} move to the left')
#             return False
#         else:
#             self.get_logger().info(f'INFO --- x align successfully')
#             # 再控制y方向
#             self.ideal_y = 0.5
#             self.ideal_y_tolerance = 0.1
#             now_y = qr_center_y/480
#             if now_y < self.ideal_y - self.ideal_y_tolerance:
#                 self.get_logger().info(f'INFO --- now_y = {now_y} move to the down')
#                 return False
#             elif now_y > self.ideal_y + self.ideal_y_tolerance:
#                 self.get_logger().info(f'INFO --- now_y = {now_y} move to the up')
#                 return False
#             else:
#                 self.get_logger().info(f'INFO --- y align successfully')
#                 # 最后控制d方向
#                 self.ideal_d = 0.25
#                 self.ideal_d_tolerance = 0.05
#                 now_d = qr_rs_d/1000
#                 if now_d < self.ideal_d - self.ideal_d_tolerance:
#                     self.get_logger().info(f'INFO --- now_d = {now_d} move to the front')
#                     return False
#                 elif now_d > self.ideal_d + self.ideal_d_tolerance:
#                     self.get_logger().info(f'INFO --- now_d = {now_d} move to the back')
#                     return False
#                 else:
#                     self.get_logger().info(f'INFO --- d align successfully')
#                     return True
        


# def main(args=None):
#     rclpy.init(args=args)
#     realsense_node = DebugNode('debug_node')
#     try:
#         rclpy.spin(realsense_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         realsense_node.stop()
#         realsense_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
