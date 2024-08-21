# 创建RealSenseNode类，并注册节点real_sense_node
# 1.timer每0.05秒产生最新RGBD数据(cv2格式)，imshow出'RGB'和'Depth'，暂时无需发布出去,只用一个机械臂手上的相机d435
# 2.创建msg_status订阅器，用于接收状态码，当状态码为sort_xxx时执行对应的目标检测/目标跟踪/二维码识别功能
# 3.创建srv_package_track服务，返回当下的包裹追踪信息(rs坐标系下,转换到arm的事情丢给机械臂节点去做)(json)
# 4.创建srv_shelf_info服务，返回当下的货架信息(是否有包裹，二维码信息)(json)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from caigou_interfaces.srv import PackageTrack, ParcelInfo, ShelfInfo
import json
import numpy as np

import cv2
from pyzbar.pyzbar import decode
import pyrealsense2 as rs
from caigou_pkg.yolov5_deepsort.AIDetector_pytorch import Detector
# from ultralytics import YOLO

# #DEBUG
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# from caigou_interfaces.srv import BasicArmCmd
# import cv2.aruco as aruco  # 4.5.5.64：opencv-python-4.5.5.64 opencv-contrib-python-4.5.5.64
# from scipy.spatial.transform import Rotation as R

class RealSenseNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # 定义相机参数
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fps = 60

        # 首先检查相机是不是进来了
        connect_device = []
        for d in rs.context().devices:
            self.get_logger().info(
                f'INFO --- Found device: {d.get_info(rs.camera_info.name)}, {d.get_info(rs.camera_info.serial_number)} ')
            if d.get_info(rs.camera_info.name).lower() != 'platform camera':
                connect_device.append(d.get_info(rs.camera_info.serial_number))

        if len(connect_device) == 0:
            self.get_logger().error(f'ERROR --- Registrition needs two camera connected.But got 0.')
            exit()
        else:
            self.get_logger().info(
                f'INFO --- __init__ self.camera_width = {self.camera_width}, self.camera_height = {self.camera_height}, self.camera_fps = {self.camera_fps}')

        # 初始化手臂上的realsense相机
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.camera_width, self.camera_height, rs.format.z16, self.camera_fps)
        config.enable_stream(rs.stream.color, self.camera_width, self.camera_height, rs.format.bgr8, self.camera_fps)

        # 启动pipeline
        self.profile = self.pipeline.start(config)

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # 获取两个相机的内参
        try:
            self.depth_intri = self.profile.get_stream(
                rs.stream.depth).as_video_stream_profile().get_intrinsics()  # 获取相机内参，很重要，是转到真实坐标的关键参数
            self.get_logger().info(f'INFO --- __init__ self.depth_intri: {self.depth_intri}')

            # #DEBUG
            # self.intr_matrix = np.array([  # 内参矩阵，转ndarray方便后续opencv直接使用
            # [self.depth_intri.fx, 0, self.depth_intri.ppx], [0, self.depth_intri.fy, self.depth_intri.ppy], [0, 0, 1]])
            # self.get_logger().info(f'INFO --- __init__ self.intr_matrix: {self.intr_matrix}')
            # self.intr_coeffs = np.array([self.depth_intri.coeffs])  # 畸变参数

        except Exception as e:
            self.get_logger().error(f'ERROR --- Failed to get intrinsics: {e}')

        # 创建msg_status订阅器
        self.msg_status_sub = self.create_subscription(String, 'msg_status', self.msg_status_callback, 10)

        # 创建srv_package_track服务端
        self.srv_package_track_server = self.create_service(PackageTrack, 'srv_package_track',
                                                            self.srv_package_track_callback)
        # 创建srv_parcel_info服务端
        self.srv_parcel_info_server = self.create_service(ParcelInfo, 'srv_parcel_info', self.srv_parcel_info_callback)
        # 创建srv_shelf_info服务端
        self.srv_shelf_info_server = self.create_service(ShelfInfo, 'srv_shelf_info', self.srv_shelf_info_callback)
        self.shelf_ROI_x_size = 150  # pixel, 丢给分类网络的宽度
        self.shelf_info = {}  # 保存货架信息，包括二维码信息和是否有包裹
        self.parcel_info = {}
        self.package_track_rsxyzcls_list = []
        
        # 由msg_status订阅状态码，初始为free
        self.status = "free"
        #DEBUG
        # self.status = "sort_package_tracking"
        # self.status = "sort_put_package"
        # self.srv_basic_arm_cmd_client = self.create_client(BasicArmCmd, 'srv_basic_arm_cmd')  # 机械臂基础命令服务

        # 创建RGBD timer, 0.05s调一次，会根据self.status决定加入哪些后续分析步骤
        self.rgbd_timer = self.create_timer(0.05, self.rgbd_timer_callback)
        self.latest_rgb_img = None  # cv2格式,numpy数组
        self.latest_depth_img = None
        

        # 创建目标跟踪器
        self.detector = Detector()
        # self.detector = YOLO('/home/caigou/workspace/caigou_ws/track_20240615.pt')
        # self.classifier = YOLO('/home/caigou/workspace/caigou_ws/classify.pt')
        
        # 创建RGB图像发布器
        self.bridge = CvBridge()
        self.pub_rgb = self.create_publisher(Image, 'msg_rgb', 10)


        # 初始化完成
        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')

    def msg_status_callback(self, msg):
        self.get_logger().info(f'INFO --- received msg_status = {msg.data}')  # init_start, sort_start...
        self.status = msg.data

    def srv_package_track_callback(self, request, response):
        # self.get_logger().info(f'INFO --- received srv_package_track request = {request.package_track_req}')
        if request.package_track_req == "package_track_rsxyzcls_list":
            response.package_track_res = json.dumps(self.package_track_rsxyzcls_list)
        else:
            response.package_track_res = json.dumps([])
        return response
    
    def srv_parcel_info_callback(self, request, response):
        # self.get_logger().info(f'INFO --- received srv_parcel_info request = {request.parcel_info_req}')
        if request.parcel_info_req == "parcel_info":
            response.parcel_info_res = json.dumps(self.parcel_info)
        else:
            response.parcel_info_res = json.dumps({})
        return response

    def srv_shelf_info_callback(self, request, response):
        # self.get_logger().info(f'INFO --- received srv_shelf_info request = {request.shelf_info_req}')
        if request.shelf_info_req == "shelf_info":
            response.shelf_info_res = json.dumps(self.shelf_info)
        else:
            response.shelf_info_res = json.dumps({})
        return response

    def rgbd_timer_callback(self):

        fps_start_time = time.time()

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            self.get_logger().warn('No frames available')
            return

        # 转换成numpy数组
        self.latest_depth_img = np.asanyarray(depth_frame.get_data())
        self.latest_rgb_img = np.asanyarray(color_frame.get_data())

        # 根据self.status决定对应操作
        if self.status == "sort_package_tracking"  or self.status == "sort_wait_for_new_package" or self.status == "pickup_package_tracking" or self.status == "pickup_wait_for_new_package":  # 主要任务：1.识别包裹 2.读取每个包裹的有效深度均值(类似表面中心距离) 3.保存在rs坐标系下的坐标(单位m)

            self.package_detect_x1y1x2y2cls_list = self.sort_package_detect()  # 识别包裹，返回[每个包裹在self.latest_rgb_img中的检测框（x1y1x2y2)+类别cls]

            package_track_rsxyzcls_list = []  # 目标合适抓取点的rs坐标系下的坐标

            for x1y1x2y2cls in self.package_detect_x1y1x2y2cls_list:
                # 先拿到图片中的锚选框，计算合适的抓取点
                x1, y1, x2, y2, cls = x1y1x2y2cls
                px, py, pd = self.get_idea_grab_point_xyd(x1, y1, x2, y2)  # 单位 = pixel, pixel, mm
                if px is None or py is None or pd is None:
                    continue
                # 生成合适抓取点的rs坐标系下的xyz坐标
                rs_x, rs_y, rs_z = rs.rs2_deproject_pixel_to_point(self.depth_intri, [px, py],
                                                                   pd)  # 计算合理抓取中心的rsxyd, 单位 = mm

                rs_x, rs_y, rs_z = rs_x / 1000, rs_y / 1000, rs_z / 1000  # 单位 = m

                package_track_rsxyzcls_list.append([rs_x, rs_y, rs_z, cls])

                # 在rgb图上绘制检测到的目标框和中心点，在点的下方绘制种类cls和ID，再标明中心点在rs坐标系下的坐标
                # cv2.rectangle(self.latest_rgb_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(self.latest_rgb_img, (px, py), 5, (0, 0, 255), -1)
                cv2.putText(self.latest_rgb_img, f"{cls}", (px, py + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2)
                cv2.putText(self.latest_rgb_img, f"rs_ {rs_x:.3f}, {rs_y:.3f}, {rs_z:.3f}", (px, py + 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.package_track_rsxyzcls_list = package_track_rsxyzcls_list

        elif self.status == "sort_goto_cabinets" or self.status == "pickup_goto_unloading_area":  # 主要任务：1.识别并定位包裹上的二维码
            self.parcel_info = {}
            QRCodes_info = self.detect_qr_code()  # 识别二维码，返回[[string_data, center_x, center_y, rs_d]...]
            if len(QRCodes_info):
                QRCode_info = QRCodes_info[0]
                string_data, qr_center_x, qr_center_y, qr_rs_d = QRCode_info
                # DEBUG在rgb图上绘制二维码的中心点和string_data
                cv2.circle(self.latest_rgb_img, (qr_center_x, qr_center_y), 5, (0, 0, 255), -1)
                cv2.putText(self.latest_rgb_img, f"{string_data}", (qr_center_x, qr_center_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                self.parcel_info = {"parcel_info": [string_data, qr_center_x, qr_center_y, qr_rs_d]}

        elif self.status == "sort_put_package":  # 主要任务：1.识别并定位二维码 2.判断是否为空仓
            QRCodes_info = self.detect_qr_code()  # 识别二维码，返回[[string_data, center_x, center_y, rs_d]...]
            if len(QRCodes_info):
                QRCode_info = QRCodes_info[0]  # 默认选取第一个二维码
                string_data, qr_center_x, qr_center_y, qr_rs_d = QRCode_info
                # DEBUG在rgb图上绘制二维码的中心点和string_data
                cv2.circle(self.latest_rgb_img, (qr_center_x, qr_center_y), 5, (0, 0, 255), -1)
                cv2.putText(self.latest_rgb_img, f"{string_data}", (qr_center_x, qr_center_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # # 判断是否有货物在指定区域内
                # crop_x_start = max(0, qr_center_x - self.shelf_ROI_x_size)
                # crop_x_end = min(self.latest_rgb_img.shape[1] - 1, qr_center_x + self.shelf_ROI_x_size)
                # shelf_ROI = self.latest_rgb_img[:qr_center_y, crop_x_start:crop_x_end]
                # classify_results = self.classifier(shelf_ROI, verbose=False)
                # classify_results = classify_results[0].names[classify_results[0].probs.top1]
                # DEBUG在rgb图上绘制货架区域
                # cv2.rectangle(self.latest_rgb_img, (crop_x_start, 0), (crop_x_end, qr_center_y), (0, 255, 0), 2)
                # cv2.putText(self.latest_rgb_img, f"whatsoever", (crop_x_start, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                self.shelf_info = {"qr_code_info": QRCode_info, "has_parcel": "whatsoever"}
            else:
                self.shelf_info = {}

        else:
            pass

        fps_end_time = time.time()
        fps = 1 / (fps_end_time - fps_start_time)
        # self.show_latest_rgb_img(fps)

        # 发布RGB图像
        # msg_rgb = self.bridge.cv2_to_imgmsg(self.latest_rgb_img, encoding="bgr8")
        # self.pub_rgb.publish(msg_rgb)


    def sort_package_detect(self):
        pred_boxes = self.detector.detect(self.latest_rgb_img)
        # pred_boxes.append((
        #     round(box[0] * self.scale), 
        #     round(box[1] * self.scale),
        #     round((box[0] + box[2]) * self.scale), 
        #     round((box[1] + box[3]) * self.scale),
        #     self.CLASSES[class_ids[index]]
        #             ))
        # pred_boxes = [(x1, y1, x2, y2, cls), (...)]

        return pred_boxes

    # def sort_package_tracking(self):
    #     # 已经有了self.latest_rgb_img和self.latest_depth_img
    #     # 下面是yolov5+deepsort的代码
    #     x1_y1_x2_y2_cls_id_list = self.detector.feedCap(self.latest_rgb_img)
    #     return x1_y1_x2_y2_cls_id_list
    #     # # 下面是yolov8的代码
    #     # x1_y1_x2_y2_cls_list = self.detector.feedCap(self.latest_rgb_img)
    #     # return x1_y1_x2_y2_cls_list


    #     # results = self.detector.track(self.latest_rgb_img, persist=True, verbose=False)  # 返回检测框的坐标和类别
    #     # boxes = results[0].boxes.xyxy.tolist()
    #     # classes = results[0].boxes.cls.tolist()
    #     # names = results[0].names
    #     # ids = results[0].boxes.id
    #     # if boxes is None or classes is None or ids is None:
    #     #     print("has at least one empty")
    #     #     return []
    #     # else:
    #     #     package_track_x1y1x2y2clsid_list = []
    #     #     for idx in range(len(boxes)):
    #     #         x1, y1, x2, y2 = boxes[idx]
    #     #         cls = names[classes[idx]]
    #     #         id_ = int(ids[idx].item())  # Convert the tensor to an integer
    #     #         package_track_x1y1x2y2clsid_list.append([x1, y1, x2, y2, cls, id_])
    #     #     # print(package_track_x1y1x2y2clsid_list)
    #     #     return package_track_x1y1x2y2clsid_list

    def get_idea_grab_point_xyd(self, x1, y1, x2, y2, size=10):
        px = int((x1 + x2) / 2)  #! 得到检测框中的理想抓取点xy坐标,需优化
        # 改成x为中间，y为上面1/4位置
        py = int(y1 + (y2 - y1) / 4)

        pd = self.get_pixel_valid_depth(px, py, size)  # 得到检测框中的理想抓取点深度值
        if pd is None:
            return None, None, None

        return px, py, pd

    def detect_qr_code(self):  # 检测并定位二维码，返回 [[string_data, center_x, center_y, rs_d]...]
        QRCodes = decode(self.latest_rgb_img)
        QRCodes_info = []
        for QRCode in QRCodes:  # QRCode = Decoded(data=b'1-1-1', type='QRCODE', rect=Rect(..., polygon=[Point(...
            stringData = QRCode.data.decode('utf-8')
            x, y, w, h = QRCode.rect
            center_x, center_y = int(x + w // 2), int(y + h // 2)
            try:
                rs_d = self.get_pixel_valid_depth(center_x, center_y, ROI_size=10)
                if rs_d:
                    rs_d = rs_d /1000  # convert to m
            except Exception as e:
                self.get_logger().error(f'ERROR --- get_pixel_valid_depth error: {e}')
                rs_d = None
            if rs_d is None:  #! 如果深度值无效，则根据二维码的矩形框大小判断距离
                area = w * h
                self.get_logger().info(f'INFO --- area = {area}')
                # if 10000 <= area <= 20000:
                #     rs_d = 0.2
                # elif area < 10000:
                #     rs_d = 0.1
                # else:
                #     rs_d = 0.3
                rs_d = 0.2  #! 临时赋值
            QRCode_info = [stringData, center_x, center_y, rs_d]
            QRCodes_info.append(QRCode_info)

        return QRCodes_info
        
    def get_pixel_valid_depth(self, center_x, center_y, ROI_size=10, lower_percent=10, upper_percent=90):
        center_d = self.latest_depth_img[center_y, center_x]  # 获取中心点的深度值

        crop_x_start = max(0, center_x - ROI_size // 2)
        crop_x_end = min(self.latest_depth_img.shape[1], center_x + ROI_size // 2 + 1)
        crop_y_start = max(0, center_y - ROI_size // 2)
        crop_y_end = min(self.latest_depth_img.shape[0], center_y + ROI_size // 2 + 1)
        center_ROI_depth_values = self.latest_depth_img[crop_y_start:crop_y_end, crop_x_start:crop_x_end]
        valid_center_ROI_depth_values = center_ROI_depth_values[(center_ROI_depth_values > 0) & (~np.isnan(center_ROI_depth_values))]

        if np.isnan(center_d) or center_d == 0:
            
            if valid_center_ROI_depth_values.size == 0:  # 检查数组是否为空
                return None  # 如果没有有效的深度值，则返回None

            valid_center_d = np.median(valid_center_ROI_depth_values)  # 使用中位数代替平均值以增加鲁棒性
            if np.isnan(valid_center_d):
                return None  # 如果计算后的深度仍然是NaN，则返回None
        else:
            valid_center_d = center_d

            # # 判断中心点的深度值是否在合理范围内
            if center_d < np.percentile(valid_center_ROI_depth_values, lower_percent) or center_d > np.percentile(valid_center_ROI_depth_values, upper_percent):
                valid_center_d = np.median(valid_center_ROI_depth_values)
            else:
                valid_center_d = center_d

        return int(valid_center_d)
    
    def show_latest_rgb_img(self, fps):
        cv2.putText(self.latest_rgb_img, f"fps: {fps:.2f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("latest_rgb_img", self.latest_rgb_img)
        cv2.waitKey(1)

    def show_latest_rgbd_img(self):
        # 展示最新的rgbd图像（有可能改变：被圈框，打id）
        latest_depth_img_8bit = self.latest_depth_img.astype(np.float32)  # 将16位图像转换为8位图像，有更好的观感（像素范围0～255）
        latest_depth_img_8bit -= latest_depth_img_8bit.min()
        latest_depth_img_8bit /= (latest_depth_img_8bit.max() - latest_depth_img_8bit.min())
        latest_depth_img_8bit *= 255
        latest_depth_img_8bit = 255 - latest_depth_img_8bit
        latest_depth_img_8bit = latest_depth_img_8bit.astype(np.uint8)
        cv2.imshow("latest_rgbd_img", np.hstack(
            (self.latest_rgb_img,
             cv2.applyColorMap(cv2.convertScaleAbs(latest_depth_img_8bit, alpha=1), cv2.COLORMAP_JET))))
        cv2.waitKey(1)

    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    realsense_node = RealSenseNode('realsense_node')
    try:
        rclpy.spin(realsense_node)
    except KeyboardInterrupt:
        pass
    finally:
        realsense_node.stop()
        realsense_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

