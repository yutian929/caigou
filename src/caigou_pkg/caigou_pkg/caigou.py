# import time
# import numpy as np
# import pyaudio
# import wave
#
#
# def record_audio():
#     CHUNK = 1024
#     FORMAT = pyaudio.paInt16
#     CHANNELS = 1
#     RATE = 16000
#     RECORD_SECONDS = 7 #录音时长
#     NOISE_SAMPLES = 5  #噪声样本数
#     MARGIN = 1.5  #动态阈值倍数
#     WAVE_OUTPUT_FILENAME = "output_caigou.wav"
#     p = pyaudio.PyAudio()
#     stream = p.open(format=FORMAT,
#                     channels=CHANNELS,
#                     rate=RATE,
#                     input=True,
#                     frames_per_buffer=CHUNK)
#     print("Measuring noise...")
#     noise_frames = [stream.read(CHUNK) for _ in range(NOISE_SAMPLES)]
#     noise_energy = np.mean([np.linalg.norm(np.frombuffer(frame, dtype=np.int16)) / CHUNK for frame in noise_frames])
#     THRESHOLD = noise_energy * MARGIN
#     print(f"Dynamic threshold set to: {THRESHOLD}")
#
#     print("* recording")
#     frames = []
#     silent_count = 0
#     start_recording = False
#     start_time = time.time()
#     while time.time() - start_time < RECORD_SECONDS:
#         data = stream.read(CHUNK)
#
#         energy = np.linalg.norm(np.frombuffer(data, dtype=np.int16)) / CHUNK
#
#         if energy < THRESHOLD and start_recording:
#             silent_count += 1
#             frames.append(data)
#         elif energy < THRESHOLD and not start_recording:
#             continue
#         elif energy > THRESHOLD and not start_recording:
#             start_recording = True
#             print(f"start_energy = {energy}")
#             frames.append(data)
#         elif energy > THRESHOLD and start_recording:
#             frames.append(data)
#             silent_count = max(0, silent_count-1)
#
#         if silent_count > 30:
#             break
#
#     print("* done recording")
#     stream.stop_stream()
#     stream.close()
#     p.terminate()
#     wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
#     wf.setnchannels(CHANNELS)
#     wf.setsampwidth(p.get_sample_size(FORMAT))
#     wf.setframerate(RATE)
#     wf.writeframes(b''.join(frames))
#     wf.close()
#
# record_audio()
#

# # # # # # **************python查找串口端的代码************************
# import serial.tools.list_ports
# import serial

# def find_serial_port(vendor_id, product_id):
#     ports = list(serial.tools.list_ports.comports())
#     for port in ports:
#         if port.vid == vendor_id and port.pid == product_id:
#             print(f"Found target device on port {port.device}")
#             return port.device
#     return None

# # 示例：使用 Arduino Uno 的 Vendor ID 和 Product ID
# # 注意：VID 和 PID 需要是十六进制数
# vendor_id = 0x1a86  # 十六进制的 Vendor ID
# product_id = 0x7523  # 十六进制的 Product ID

# device_port = find_serial_port(vendor_id, product_id)
# print(type(device_port))
# if device_port:
#     print(f"Device is connected at: {device_port}")
# else:
#     print("Device not found.")

# arduino_serial = serial.Serial(device_port, 9600, timeout=1)
# def read_once_pumpAB():
#     while True:
#         try:
#             line = arduino_serial.readline().decode()
#             eles = line.split()  # ['pumpA', '29', 'pumpB', '33']
#             # 找到pumpA，pumpB的值
#             if len(eles) == 4:
#                 if eles[0] == "pumpA":
#                     pumpA = int(eles[1])
#                     if eles[2] == "pumpB":
#                         pumpB = int(eles[3])
#                         return {"pumpA": pumpA, "pumpB": pumpB}

#         except Exception as e:
#             print(f"Error reading from serial port: {str(e)}")
#             # 继续读取

    





# # # # # # **************判断是否有包裹的代码************************
# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import time


# # 创建realsense管道
# pipeline = rs.pipeline()
# config = rs.config()

# # 配置彩色和深度流
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# # 开始流
# profile = pipeline.start(config)

# # 获取深度传感器的深度比例
# depth_sensor = profile.get_device().first_depth_sensor()
# depth_scale = depth_sensor.get_depth_scale()
# print(f"深度传感器的深度比例: {depth_scale}")

# # 创建对齐对象
# align_to = rs.stream.color
# align = rs.align(align_to)


# # 记录开始时间
# start_time = time.time()

# try:
#     while True:
#         frames = pipeline.wait_for_frames()
#         aligned_frames = align.process(frames)

#         # 获取对齐后的帧
#         aligned_depth_frame = aligned_frames.get_depth_frame()
#         color_frame = aligned_frames.get_color_frame()

#         if not aligned_depth_frame or not color_frame:
#             continue

#         # 转换图像到numpy数组
#         depth_image = np.asanyarray(aligned_depth_frame.get_data())
#         color_image = np.asanyarray(color_frame.get_data())

#         # 检测是否有货物
#         center_x, center_y = depth_image.shape[1] // 2, depth_image.shape[0] // 2
#         zero_ratio_threshold = 0.8
#         area_width = 150
#         area_height = 150
#         crop_y_start = max(0, center_y - area_height // 2)
#         crop_y_end = min(depth_image.shape[0] - 1, center_y + area_height // 2)
#         crop_x_start = max(0, center_x - area_width // 2)
#         crop_x_end = min(depth_image.shape[1] - 1, center_x + area_width // 2)
#         crop_depth_image = depth_image[crop_y_start:crop_y_end, crop_x_start:crop_x_end]

#         # 计算0的比例
#         zero_ratio = np.sum(crop_depth_image == 0) / crop_depth_image.size
#         has_parcel = zero_ratio > zero_ratio_threshold

#         # 在color_image上绘制矩形框
#         cv2.rectangle(color_image, (crop_x_start, crop_y_start), (crop_x_end, crop_y_end), (0, 255, 0), 2)
#         cv2.putText(color_image, f"zero_ratio: {zero_ratio:.2f}, has_parcel = {has_parcel}", (crop_x_start, crop_y_start - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


#         # 用np.stack显示rgb和depth图像
#         latest_depth_img_8bit_2 = depth_image.astype(np.float32)  # 将16位图像转换为8位图像，有更好的观感（像素范围0～255）
#         latest_depth_img_8bit_2 -= latest_depth_img_8bit_2.min()
#         latest_depth_img_8bit_2 /= (latest_depth_img_8bit_2.max() - latest_depth_img_8bit_2.min())
#         latest_depth_img_8bit_2 *= 255
#         latest_depth_img_8bit_2 = 255 - latest_depth_img_8bit_2
#         latest_depth_img_8bit_2 = latest_depth_img_8bit_2.astype(np.uint8)
#         cv2.imshow("latest_rgbd_img_2", np.hstack((color_image, cv2.applyColorMap(cv2.convertScaleAbs(latest_depth_img_8bit_2, alpha=1.0), cv2.COLORMAP_JET))))
#         cv2.waitKey(1)


# finally:
#     # 停止并释放资源
#     pipeline.stop()
#     cv2.destroyAllWindows()




#
# # # # # **************实现用zbar检测二维码的代码************************
# import cv2
# from pyzbar.pyzbar import decode
# import pyrealsense2 as rs
# import numpy as np
#
#
#
# def main():
#     # realsense摄像头读取图像
#     # 创建realsense管道
#     pipeline = rs.pipeline()
#     config = rs.config()
#
#     # 配置彩色和深度流
#     config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#
#     # 开始流
#     profile = pipeline.start(config)
#
#     # 获取深度传感器的深度比例
#     depth_sensor = profile.get_device().first_depth_sensor()
#     depth_scale = depth_sensor.get_depth_scale()
#
#     # 创建对齐对象
#     align_to = rs.stream.color
#     align = rs.align(align_to)
#     frames = pipeline.wait_for_frames()
#     aligned_frames = align.process(frames)
#
#     while True:
#         frames = pipeline.wait_for_frames()
#         aligned_frames = align.process(frames)
#         # 获取对齐后的帧
#         aligned_depth_frame = aligned_frames.get_depth_frame()
#         color_frame = aligned_frames.get_color_frame()
#
#         if not aligned_depth_frame or not color_frame:
#             continue
#
#         color_image = np.asanyarray(color_frame.get_data())
#         BarcodeReader(color_image)
#
# # Make one method to decode the barcode
# def BarcodeReader(img):
#
#     # read the image in numpy array using cv2
#     # img = cv2.imread(img)
#
#     # Decode the barcode image
#     detectedBarcodes = decode(img)
#
#     # If not detected then print the message
#     if not detectedBarcodes:
#         print("", end="")
#     else:
#
#           # Traverse through all the detected barcodes in image
#         for barcode in detectedBarcodes:
#
#             # Locate the barcode position in image
#             (x, y, w, h) = barcode.rect
#
#             # Put the rectangle in image using
#             # cv2 to highlight the barcode
#             cv2.rectangle(img, (x-10, y-10),
#                           (x + w+10, y + h+10),
#                           (255, 0, 0), 2)
#
#             if barcode.data!="":
#
#             # Print the barcode data
#                 print(barcode.data)
#                 print(barcode.type)
#
#             # Draw the barcode data and barcode type on image
#             text = "{} ({})".format(barcode.data, barcode.type)
#             cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
#
#
#     #Display the image
#     cv2.imshow("Image", img)
#     cv2.waitKey(5)
#     # cv2.destroyAllWindows()
#
# def detect_qr_code(color_image):
#     QRCodes = decode(color_image)
#     for QRCode in QRCodes:
#         print(QRCode)
#         stringData = QRCode.data.decode('utf-8')
#         print("二维码字符串是：\"" + stringData + "\"")
#         #绘制出二维码边框
#         points = np.array([QRCode.polygon], np.int32)
#         #numpy reshape: https://blog.csdn.net/DocStorm/article/details/58593682
#         points = points.reshape((-1,1,2))
#         cv2.polylines(color_image, [points], True, (0,255,0), 5)
#         rectPoints = QRCode.rect
#         cv2.putText(color_image, stringData, (rectPoints[0], rectPoints[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
#     cv2.imshow('QRCode', color_image)
#     cv2.waitKey(1)
#
#
# # Unable to find zbar shared library
# # sudo apt-get install zbar-tools
#
# if __name__ == '__main__':
#     main()











# # # # **************实现config.json手动初始化的代码************************
# import json
# def programmer_set_config(data=None):
#     if not data:
#         data = {
#             "init": False,  # 是否完成初始化
#             "sort_unloading_area_xy": (0,0),  # 快递卸货区的UWB坐标
#             "sort_unloading_area_angle": 0,  # 快递卸货区小车的角度
#             "sort_cabinets_xy": {"1":(1,1),},  # 快递分拣柜的UWB坐标
#             "sort_cabinets_angle": {"1":0,},  # 快递分拣柜小车的角度
#             "pickup_import_port_xy": (4,4),  # 用户寄件区的UWB坐标
#             "pickup_import_port_angle": 0,  # 用户寄件区小车的角度
#             "pickup_export_port_xy": {"a":(5,5), "b":(6,6), "c":(7,7)},  # 不同快递公司取件区的UWB坐标
#             "pickup_export_port_angle": {"a":0, "b":0, "c":0},  # 不同快递公司取件区小车的角度

#             "angle_tolerance": 10,  # 角度误差容忍度
#             "xy_tolerance": 0.1,  # 坐标误差容忍度
#             "others": {},
#         }

#     # 文件名
#     file_name = "config.json"

#     # 将字典写入JSON文件
#     with open(file_name, 'w') as json_file:
#         json.dump(data, json_file)

#     print("Successfully written to", file_name)

# if __name__ == "__main__":
#     programmer_set_config()



#

# # # **************实现realsense采集图片/视频的代码************************
import pyrealsense2 as rs
import numpy as np
import cv2
import time

def record_aligned_video(duration_seconds, output_file='output.avi'):
    # 创建realsense管道
    pipeline = rs.pipeline()
    config = rs.config()

    # 配置彩色和深度流
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 开始流
    profile = pipeline.start(config)

    # 获取深度传感器的深度比例
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # 创建对齐对象
    align_to = rs.stream.color
    align = rs.align(align_to)

    # 创建OpenCV视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_file, fourcc, 30.0, (640, 480))
    # AttributeError: module 'cv2' has no attribute 'VideoWriter_fourcc'

    # 记录开始时间
    start_time = time.time()

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            # 获取对齐后的帧
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                continue

            # 转换图像到numpy数组
            # depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # # 可以选择将深度图像转换为彩色图显示
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # 保存帧
            out.write(color_image)

            # 显示图像
            cv2.imshow('Aligned Frame', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # 检查录制时间
            if time.time() - start_time > duration_seconds:
                break
    finally:
        # 停止并释放资源
        pipeline.stop()
        out.release()
        cv2.destroyAllWindows()

def record_aligned_image(output_folder='/home/caigou/workspace/caigou_ws/pictures'):
    # 创建realsense管道
    pipeline = rs.pipeline()
    config = rs.config()

    # 配置彩色和深度流
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 开始流
    profile = pipeline.start(config)

    # 获取深度传感器的深度比例
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # 创建对齐对象
    align_to = rs.stream.color
    align = rs.align(align_to)

    # 记录开始时间
    start_time = time.time()

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            # 获取对齐后的帧
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                continue

            # 转换图像到numpy数组
            # depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # 检测按键s
            key = cv2.waitKey(1)
            if key & 0xFF == ord('s'):
                # 保存图像
                image_file = f'{output_folder}/image_{time.strftime("%Y%m%d_%H%M%S")}.png'
                cv2.imwrite(image_file, color_image)
                print(f"Saved image to {image_file}")

            # 显示图像
            cv2.imshow('Aligned Frame', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 停止并释放资源
        pipeline.stop()
        cv2.destroyAllWindows()

# 使用方法
# record_aligned_video(120, '/home/eai/ros2_ws/caigou_ws/videos/boxes_0608_3.avi')
record_aligned_image('/home/caigou/workspace/caigou_ws/pictures')



# # # **************实现读取两个坐标list.json文件计算旋转+平移矩阵的代码2************************
# import json
# import numpy as np
# from scipy.spatial.transform import Rotation as R

# # 读取数据点
# def read_points_from_json(file_path):
#     with open(file_path, 'r') as file:
#         points = json.load(file)
#     return np.array(points)

# # 计算变换矩阵
# def calculate_transformation_matrix(arm_points, camera_points):
#     # 去均值化
#     arm_mean = np.mean(arm_points, axis=0)
#     camera_mean = np.mean(camera_points, axis=0)
#     arm_centered = arm_points - arm_mean
#     camera_centered = camera_points - camera_mean

#     # 计算 SVD
#     H = camera_centered.T @ arm_centered
#     U, S, Vt = np.linalg.svd(H)

#     # 计算旋转矩阵
#     R = Vt.T @ U.T
#     if np.linalg.det(R) < 0:
#         Vt[-1, :] *= -1
#         R = Vt.T @ U.T

#     # 计算平移向量
#     t = arm_mean - R @ camera_mean

#     # 构造变换矩阵 T
#     T = np.eye(4)
#     T[:3, :3] = R
#     T[:3, 3] = t
#     return T

# # 主函数
# def main():
#     arm_points = read_points_from_json('arm_xyz_list.json')
#     camera_points = read_points_from_json('rs_xyz_list.json')
#     T = calculate_transformation_matrix(arm_points, camera_points)

#     # 设定一个测试点
#     test_point = np.array([-0.09553858967401795, 0.018043014266123737, 0.3352320311071586, 1])  # Realsense 相机坐标系下的点, [-0.09553858967401795, 0.018043014266123737, 0.3352320311071586]
#     mapped_point = T @ test_point  # 映射到机械臂坐标系下

#     print("变换矩阵 T:\n", T)
#     print("映射后的测试点:\n", mapped_point[:3])  # 不包含齐次坐标的部分  # [0.1517000049352646, 0.20990000665187836, 0.09730000048875809]

# if __name__ == "__main__":
#     main()




# # # **************实现读取两个坐标list.json文件计算旋转+平移矩阵的代码1************************
# import numpy as np
# import json

# def get_xyz_list_from_json(json_file):
#     with open(json_file, 'r') as file:
#         xyz_list = json.load(file)
#     return xyz_list

# def rs_xyd_2_rs_xyz(rs_x, rs_y, rs_dis):  # mm
#     # 使用勾股定理计算z，这里假设rs_dis已经代表从相机到点的直线距离
#     # 计算物体在相机坐标系中的实际z坐标
#     rs_xyl2 = rs_x ** 2 + rs_y ** 2  # 计算x和y分量的平方和
#     if rs_dis ** 2 - rs_xyl2 >= 0:
#         rs_z = np.sqrt(rs_dis ** 2 - rs_xyl2)  # 根据勾股定理计算z值,must be >0
#         return rs_x, rs_y, rs_z  # mm
#     else:
#         print("error: rs_dis**2 - rs_xyl2 < 0")
#         return 0, 0, 0  # mm

# def get_coordinate_system_convert_matrixs(RS:np.array, ARM:np.array):  # both xyz, danwei=m
#     # 1. 计算质心
#     centroid_RS = np.mean(RS, axis=0)
#     centroid_ARM = np.mean(ARM, axis=0)

#     # 2. 去中心化
#     RS_centered = RS - centroid_RS
#     ARM_centered = ARM - centroid_ARM

#     # 3. 计算旋转矩阵
#     H = RS_centered.T @ ARM_centered
#     U, _, Vt = np.linalg.svd(H)
#     R = Vt.T @ U.T

#     # 确保一个合理的旋转（避免反射）
#     if np.linalg.det(R) < 0:
#         Vt[2,:] *= -1
#         R = Vt.T @ U.T

#     # 4. 计算平移向量
#     t = centroid_ARM - R @ centroid_RS

#     return R, t

# def get_calibrate_r_t_matrixs():
#     rs_xyz_list = np.array(get_xyz_list_from_json('rs_xyz_list.json'))
#     arm_xyz_list = np.array(get_xyz_list_from_json('arm_xyz_list.json'))
#     return get_coordinate_system_convert_matrixs(rs_xyz_list, arm_xyz_list)



# if __name__ == '__main__':
#     R, t = get_calibrate_r_t_matrixs()
#     print(f"self.R = np.array({np.array(R)})")
#     print(f"self.t = np.array({np.array(t)})")
#     # 测试的两个点
#     rs_test = np.array([-0.17855298149783602, 0.0137347358446697, 0.4542347218619453])
#     arm_test = R @ rs_test + t  # [0.2084445059299469, 0.27309301495552063, -0.014186509884893894]
#     print(arm_test)

# # self.R = np.array([[ 0.04156017 -0.64583217  0.7623474 ]
# #  [-0.99881962 -0.00765544  0.0479663 ]
# #  [-0.02514207 -0.76344102 -0.64538801]])
# #
# # self.t = np.array([-0.13985799  0.07139193  0.30140226])






#
# # # **************实现两个realsense相机驱动的原始python代码************************
# import pyrealsense2 as rs
# import cv2
# import numpy as np
# import time
# import os
# import sys
#
# if __name__ == '__main__':
#
#
#     # 确定图像的输入分辨率与帧率
#     resolution_width = 640  # pixels
#     resolution_height = 480  # pixels
#     frame_rate = 15  # fps
#
#     # 注册数据流，并对其图像
#     align = rs.align(rs.stream.color)
#     rs_config = rs.config()
#     rs_config.enable_stream(rs.stream.depth, resolution_width, resolution_height, rs.format.z16, frame_rate)
#     rs_config.enable_stream(rs.stream.color, resolution_width, resolution_height, rs.format.bgr8, frame_rate)
#     #
#     rs_config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, frame_rate)
#     rs_config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, frame_rate)
#     # check相机是不是进来了
#     connect_device = []
#     for d in rs.context().devices:
#         print('Found device: ',
#               d.get_info(rs.camera_info.name), ' ',
#               d.get_info(rs.camera_info.serial_number))
#         if d.get_info(rs.camera_info.name).lower() != 'platform camera':
#             connect_device.append(d.get_info(rs.camera_info.serial_number))
#
#
#     if len(connect_device) < 2:
#         print(f'Registrition needs two camera connected.But got {len(connect_device)}.')
#         exit()
#
#     # 确认相机并获取相机的内部参数
#     pipeline1 = rs.pipeline()
#     rs_config.enable_device(connect_device[0])
#     # pipeline_profile1 = pipeline1.start(rs_config)
#     pipeline1.start(rs_config)
#
#     pipeline2 = rs.pipeline()
#     rs_config.enable_device(connect_device[1])
#     # pipeline_profile2 = pipeline2.start(rs_config)
#     pipeline2.start(rs_config)
#
#     try:
#
#         while True:
#
#             # 等待数据进来
#             frames1 = pipeline1.wait_for_frames()
#             frames2 = pipeline2.wait_for_frames()
#
#             # 将进来的RGBD数据对齐
#             aligned_frames1 = align.process(frames1)
#             aligned_frames2 = align.process(frames2)
#
#             # 将对其的RGB—D图取出来
#             color_frame1 = aligned_frames1.get_color_frame()
#             depth_frame1 = aligned_frames1.get_depth_frame()
#             color_frame2 = aligned_frames2.get_color_frame()
#             depth_frame2 = aligned_frames2.get_depth_frame()
#             # --------------------------------------
#             depth_frame1 = frames1.get_depth_frame()
#             color_frame1 = frames1.get_color_frame()
#             depth_frame2 = frames2.get_depth_frame()
#             color_frame2 = frames2.get_color_frame()
#
#             # 数组化数据便于处理
#
#             ir_frame_left1 = frames1.get_infrared_frame(1)
#             ir_frame_right1 = frames1.get_infrared_frame(2)
#             if not depth_frame1 or not color_frame1:
#                 continue
#             ir_frame_left2 = frames2.get_infrared_frame(1)
#             ir_frame_right2 = frames2.get_infrared_frame(2)
#             if not depth_frame2 or not color_frame2:
#                 continue
#
#             color_image1 = np.asanyarray(color_frame1.get_data())
#             depth_image1 = np.asanyarray(depth_frame1.get_data())
#             ir_left_image1 = np.asanyarray(ir_frame_left1.get_data())
#             ir_right_image1 = np.asanyarray(ir_frame_right1.get_data())
#
#             color_image2 = np.asanyarray(color_frame2.get_data())
#             depth_image2 = np.asanyarray(depth_frame2.get_data())
#             ir_left_image2 = np.asanyarray(ir_frame_left2.get_data())
#             ir_right_image2 = np.asanyarray(ir_frame_right2.get_data())
#
#             depth_colormap1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image1, alpha=0.03), cv2.COLORMAP_JET)
#             images1 = np.hstack((color_image1, depth_colormap1))
#
#             depth_colormap2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image2, alpha=0.03), cv2.COLORMAP_JET)
#             images2 = np.hstack((color_image2, depth_colormap2))
#             cv2.imshow('RealSense1', images1)
#             cv2.imshow('RealSense2', images2)
#
#             key = cv2.waitKey(1)
#
#             if key & 0xFF == ord('q') or key == 27:
#                 cv2.destroyAllWindows()
#                 break
#     finally:
#         pipeline1.stop()
#         pipeline2.stop()
#
#
# # # # Found device:  Intel RealSense D435I   943222073830
# # # Found device:  Intel RealSense D415   802212060288
# # # #










# # **************实现手眼标定的原始python代码************************
# import pyrealsense2 as rs
# import numpy as np
# import cv2
# # 提示没有aruco的看问题汇总
# import cv2.aruco as aruco
# # from cv2 import aruco 
# # 配置摄像头与开启pipeline
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# profile = pipeline.start(config)
# align_to = rs.stream.color
# align = rs.align(align_to)

# # 获取对齐的rgb和深度图
# def get_aligned_images():
#     frames = pipeline.wait_for_frames()
#     aligned_frames = align.process(frames)
#     aligned_depth_frame = aligned_frames.get_depth_frame()
#     color_frame = aligned_frames.get_color_frame()
#     # 获取intelrealsense参数
#     intr = color_frame.profile.as_video_stream_profile().intrinsics
#     # 内参矩阵，转ndarray方便后续opencv直接使用
#     intr_matrix = np.array([
#         [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
#     ])
#     # 深度图-16位
#     depth_image = np.asanyarray(aligned_depth_frame.get_data())
#     # 深度图-8位
#     depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)
#     pos = np.where(depth_image_8bit == 0)
#     depth_image_8bit[pos] = 255
#     # rgb图
#     color_image = np.asanyarray(color_frame.get_data())
#     # return: rgb图，深度图，相机内参，相机畸变系数(intr.coeffs)
#     return color_image, depth_image, intr_matrix, np.array(intr.coeffs)


# if __name__ == "__main__":
#     n = 0
#     while 1:
#         rgb, depth, intr_matrix, intr_coeffs = get_aligned_images()
#         # 获取dictionary, DICT_ARUCO_ORIGINAL的码，id应该是582
#         aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
#         # 创建detector parameters
#         parameters = aruco.DetectorParameters_create()
#         # 输入rgb图, aruco的dictionary, 相机内参, 相机的畸变参数
#         corners, ids, rejected_img_points = aruco.detectMarkers(rgb, aruco_dict, parameters=parameters,cameraMatrix=intr_matrix, distCoeff=intr_coeffs)
#         # 估计出aruco码的位姿，0.1对应markerLength参数，单位是meter
#         marker_size = 0.03
#         # rvec是旋转向量， tvec是平移向量
#         rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, intr_matrix, intr_coeffs)
#         try:
#             # 在图片上标出aruco码的位置
#             aruco.drawDetectedMarkers(rgb, corners)
#             # 根据aruco码的位姿标注出对应的xyz轴, 0.05对应length参数，代表xyz轴画出来的长度 
#             aruco.drawAxis(rgb, intr_matrix, intr_coeffs, rvec, tvec, 0.01)
#             cv2.imshow('RGB image', rgb)
#         except:
#             cv2.imshow('RGB image', rgb)

#         if ids is not None and len(ids) > 0:
#             # 假设我们只关心第一个检测到的标记
#             marker_position = tvec[0][0]
#             print("ArUco marker center position in camera frame:", marker_position)

#         key = cv2.waitKey(10)
#         # 按键盘q退出程序
#         if key & 0xFF == ord('q') or key == 27:
#             pipeline.stop()
#             break
#         # 按键盘s保存图片
#         elif key == ord('s'):
#             n = n + 1
#             # 保存rgb图
#             cv2.imwrite('./img/rgb' + str(n) + '.jpg', rgb)

#     cv2.destroyAllWindows()



