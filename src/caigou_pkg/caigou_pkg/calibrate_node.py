# 眼在手上的标定

import rclpy
from rclpy.node import Node
from caigou_interfaces.msg import ArmCoords
from caigou_interfaces.srv import BasicArmCmd
import json
import pyrealsense2 as rs

import transforms3d as tfs
import numpy as np
import math
import time
import cv2
import cv2.aruco as aruco  # 4.5.5.64：opencv-python==4.5.5.64 opencv-contrib-python==4.5.5.64
from scipy.spatial.transform import Rotation as R

class CalibrateNode(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        self.msg_my_coords_sub = self.create_subscription(
            ArmCoords,
            'msg_arm_coords',  # 接收机械臂末端坐标x,y,z(和姿态rx,ry,rz)的话题
            self.received_arm_coords_callback,
            10)
        self.msg_aruco_coords_pub = self.create_publisher(
            ArmCoords,
            'msg_aruco_coords',
            10)

        self.aruco_timer = self.create_timer(0.01, self.aruco_timer_callback)  # 每0.01秒检测一下aruco标志信息

        self.srv_basic_arm_cmd_client = self.create_client(BasicArmCmd, 'srv_basic_arm_cmd')  # 机械臂基础命令服务

        # 配置realsense相机
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # 配置aruco标志
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)  # 获取dictionary, DICT_ARUCO_ORIGINAL的码
        self.marker_size = 0.05  # 打印的时候就知道的标志大小,单位是m
        self.xyz_axis_length = 0.02  # 0.01对应length参数，代表xyz轴画出来的长度
        self.get_z_func = "RSDEPTH"  # "RSDEPTH" or "ARUCODEPTH"

        # 存放两个坐标系的位姿
        self.latest_arm_xyzrxyz = []
        self.latest_rs_xyzrxyz = []
        self.save_arm_coords = []
        self.save_aruco_coords = []

        

        self.get_logger().info(f'INFO --- {node_name} setup finished *^*')
        time.sleep(1)
        self.tell_to_arm([['arm_release', 'xxx', 0.3, 0.3, 0.3]])

    def get_aligned_images(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        # 获取intelrealsense参数
        intr = color_frame.profile.as_video_stream_profile().intrinsics
        # 内参矩阵，转ndarray方便后续opencv直接使用
        intr_matrix = np.array([
            [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
        ])
        # 深度图-16位
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        # 深度图-8位
        depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)
        pos = np.where(depth_image_8bit == 0)
        depth_image_8bit[pos] = 255
        # rgb图
        color_image = np.asanyarray(color_frame.get_data())
        # return: rgb图，深度图，相机内参，相机畸变系数(intr.coeffs)
        return color_image, depth_image, intr_matrix, np.array(intr.coeffs), intr

    def received_arm_coords_callback(self, msg):
        # self.get_logger().info(f'INFO --- Received my_coords: "{msg}"')
        self.latest_arm_xyzrxyz = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]

    def aruco_timer_callback(self):
        rgb, depth, intr_matrix, intr_coeffs, intr = self.get_aligned_images()

        parameters = aruco.DetectorParameters_create()  # 创建detector parameters

        # 输入rgb图, aruco的dictionary, 相机内参, 相机的畸变参数
        corners, ids, rejected_img_points = aruco.detectMarkers(rgb, self.aruco_dict, parameters=parameters,cameraMatrix=intr_matrix, distCoeff=intr_coeffs)
        
        # rvec是旋转向量， tvec是平移向量
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, self.marker_size, intr_matrix, intr_coeffs)
        try:
            # 在图片上标出aruco码的位置
            aruco.drawDetectedMarkers(rgb, corners)
            # 根据aruco码的位姿标注出对应的xyz轴, 
            aruco.drawAxis(rgb, intr_matrix, intr_coeffs, rvec, tvec, self.xyz_axis_length)
        except:
            pass

        if ids is not None and len(ids) > 0:
            # 假设我们只关心第一个检测到的标记
            R_mat = cv2.Rodrigues(rvec[0][0])[0]
            euler = R.from_matrix(R_mat).as_euler('xyz', degrees=False)
            if self.get_z_func == "ARUCODEPTH":
                self.latest_rs_xyzrxyz = [tvec[0][0][0], tvec[0][0][1], tvec[0][0][2], euler[0], euler[1], euler[2]]
            else:
                # 以下是通过深度图计算aruco码的xyz坐标
                # aruco码的中心在rgb图像中的位置
                center_x, center_y = (int((corners[0][0][0][0] + corners[0][0][2][0]) / 2), int((corners[0][0][0][1] + corners[0][0][2][1]) / 2))
                # aruco码的中心在深度图中的位置, 取5x5区间，然后去除0值和nan值，再取平均值
                crop_d = depth[center_y-2:center_y+3, center_x-2:center_x+3]
                center_d = np.mean(crop_d[np.where(crop_d != 0) and np.where(~np.isnan(crop_d))])
                
                if center_d == 0 or np.isnan(center_d):
                    self.get_logger().info(f'WARN --- center_d is 0 or nan')
                    return
                # 通过深度图和相机内参矩阵，计算aruco码的在相机坐标系下的xyz坐标
                x, y, z = rs.rs2_deproject_pixel_to_point(intr, [center_x, center_y], center_d)  # 单位都是mm
                
                z = z / 1000  # 单位转换成m
                x = x / 1000  # 单位转换成m
                y = y / 1000  # 单位转换成m
                self.latest_rs_xyzrxyz = [x, y, z, euler[0], euler[1], euler[2]]

                self.publish_aruco_coords()

        try:
            # 在图像上写两列数据，一列是机械臂末端位姿，一列是realsense看到的aruco位姿，注意是竖着写每一个数字
            for i in range(6):
                cv2.putText(rgb, f'{self.latest_arm_xyzrxyz[i]:.3f}', (10, 40+20*i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                cv2.putText(rgb, f'{self.latest_rs_xyzrxyz[i]:.3f}', (200, 40+20*i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(rgb, f'Arm coords:', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(rgb, f'RS coords:', (200, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
        except Exception as e:
            pass

        cv2.imshow('RGB image', rgb)

        key = cv2.waitKey(5)
        # 按键盘q退出程序
        if key & 0xFF == ord('q') or key == 27:
            self.pipeline.stop()
        elif key & 0xFF == ord('s'):
            self.get_logger().info(f'INFO --- Saved arm_coords: {self.latest_arm_xyzrxyz}')
            self.get_logger().info(f'INFO --- Saved rs_aruco_coords: {self.latest_rs_xyzrxyz}')

            self.save_arm_coords.append(self.latest_arm_xyzrxyz)
            self.save_aruco_coords.append(self.latest_rs_xyzrxyz)
            with open('save_arm_coords.json', 'w') as f:
                json.dump(self.save_arm_coords, f)
            with open('save_aruco_coords.json', 'w') as f:
                json.dump(self.save_aruco_coords, f)
        elif key & 0xFF == ord('c'):
            self.get_logger().info(f'INFO --- Start computing calibration matrix')
            # 读取保存的坐标
            with open('save_arm_coords.json', 'r') as f:
                save_arm_coords = json.load(f)
            with open('save_aruco_coords.json', 'r') as f:
                save_aruco_coords = json.load(f)
            # 计算变换矩阵
            Hgs,Hcs = [],[]
            for i in range(len(save_arm_coords)):
                Hg = self.get_matrix_eular_radu(*save_arm_coords[i])
                Hc = self.get_matrix_eular_radu(*save_aruco_coords[i])
                Hgs.append(Hg)
                Hcs.append(Hc)
            Hcg = self.handeyecalib(Hgs,Hcs)
            self.get_logger().info(f'INFO --- Hcg = \n{Hcg}')
            with open('save_Hcg.json', 'w') as f:
                json.dump(Hcg.tolist(), f)
        elif key & 0xFF == ord('f'):
            with open('save_Hcg.json', 'r') as f:
                Hcg = np.array(json.load(f))
            self.get_logger().info(f'INFO --- Start telling to arm with Hcg = \n{Hcg}')
            # # 转换到机械臂基座坐标系下
            cam2target_4x4 = self.get_matrix_eular_radu(self.latest_rs_xyzrxyz[0], self.latest_rs_xyzrxyz[1], self.latest_rs_xyzrxyz[2], 0, 0, 0)  # realsense看到的aruco位姿, 4x4齐次矩阵
            
            hand2cam_4x4 = Hcg
            base2hand_4x4 = self.get_matrix_eular_radu(*self.latest_arm_xyzrxyz)  # 机械臂末端位姿, 4x4齐次矩阵
            # self.get_logger().info(f'INFO --- cam2target_4x4 = \n{cam2target_4x4}')
            # self.get_logger().info(f'INFO --- hand2cam_4x4 = \n{hand2cam_4x4}')
            # self.get_logger().info(f'INFO --- base2hand_4x4 = \n{base2hand_4x4}')
            
            base2target_4x4 = np.dot(base2hand_4x4, hand2cam_4x4)
            base2target_4x4 = np.dot(base2target_4x4, cam2target_4x4)
            self.get_logger().info(f'INFO --- base2target_4x4 = \n{base2target_4x4}')
            # 发送给机械臂
            self.tell_to_arm([['debug', 'xxx', base2target_4x4[0,3], base2target_4x4[1,3], base2target_4x4[2,3]]])

    def handeyecalib(self, Hgs, Hcs):
        def skew(v):
            return np.array([[0,-v[2],v[1]],
                            [v[2],0,-v[0]],
                            [-v[1],v[0],0]])

        def rot2quat_minimal(m):
            quat =  tfs.quaternions.mat2quat(m[0:3,0:3])
            return quat[1:]

        def quatMinimal2rot(q):
            p = np.dot(q.T,q)
            w = np.sqrt(np.subtract(1,p[0][0]))
            return tfs.quaternions.quat2mat([w,q[0],q[1],q[2]])

        Hgijs,Hcijs = [], []
        A,B  = [],[]
        size = 0
        for i in range(len(Hgs)):
            for j in range(i+1,len(Hgs)):
                size += 1
                Hgij = np.dot(np.linalg.inv(Hgs[j]),Hgs[i])
                Hgijs.append(Hgij)
                Pgij = np.dot(2,rot2quat_minimal(Hgij))

                Hcij = np.dot(Hcs[j],np.linalg.inv(Hcs[i]))
                Hcijs.append(Hcij)
                Pcij = np.dot(2,rot2quat_minimal(Hcij))

                A.append(skew(np.add(Pgij,Pcij)))
                B.append(np.subtract(Pcij,Pgij))
        MA = np.asarray(A).reshape(size*3,3)
        MB = np.asarray(B).reshape(size*3,1)
        Pcg_  =  np.dot(np.linalg.pinv(MA),MB)
        pcg_norm = np.dot(np.conjugate(Pcg_).T,Pcg_)
        Pcg = np.sqrt(np.add(1,np.dot(Pcg_.T,Pcg_)))
        Pcg = np.dot(np.dot(2,Pcg_),np.linalg.inv(Pcg))
        Rcg = quatMinimal2rot(np.divide(Pcg,2)).reshape(3,3)


        A ,B = [], []
        id = 0
        for i in range(len(Hgs)):
            for j in range(i+1,len(Hgs)):
                Hgij = Hgijs[id]
                Hcij = Hcijs[id]
                A.append(np.subtract(Hgij[0:3,0:3],np.eye(3,3)))
                B.append(np.subtract(np.dot(Rcg,Hcij[0:3,3:4]),Hgij[0:3,3:4]))
                id += 1

        MA = np.asarray(A).reshape(size*3,3)
        MB = np.asarray(B).reshape(size*3,1)
        Tcg = np.dot(np.linalg.pinv(MA),MB).reshape(3,)
        return tfs.affines.compose(Tcg,np.squeeze(Rcg),[1,1,1])

    def get_matrix_eular_radu(self,x,y,z,rx,ry,rz):
        rmat = tfs.euler.euler2mat(rx,ry,rz)
        rmat = tfs.affines.compose(np.squeeze(np.asarray((x,y,z))), rmat, [1, 1, 1])
        return rmat

    def publish_aruco_coords(self):
        msg = ArmCoords()
        msg.x = self.latest_rs_xyzrxyz[0]
        msg.y = self.latest_rs_xyzrxyz[1]
        msg.z = self.latest_rs_xyzrxyz[2]
        msg.rx = self.latest_rs_xyzrxyz[3]
        msg.ry = self.latest_rs_xyzrxyz[4]
        msg.rz = self.latest_rs_xyzrxyz[5]
        self.msg_aruco_coords_pub.publish(msg)


    def tell_to_arm(self, msg_llist, callback_func = None):  # 给机械臂发送信息，可以不需要回复
        while not self.srv_basic_arm_cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('INFO --- waiting for arm service')
        request = BasicArmCmd.Request()
        request.basic_arm_cmd_req = json.dumps(msg_llist)
        self.get_logger().info(f'INFO --- tell_to_arm request.basic_arm_cmd_req = {request.basic_arm_cmd_req}')
        if callback_func is not None:
            self.srv_basic_arm_cmd_client.call_async(request).add_done_callback(callback_func)
        else:
            self.srv_basic_arm_cmd_client.call_async(request)
    
    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    calibrate_node = CalibrateNode('calibrate_node')
    try:
        rclpy.spin(calibrate_node)
    except KeyboardInterrupt:
        pass
    finally:
        calibrate_node.stop()  # Ensure pipeline is stopped on shutdown
        calibrate_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

