# 创建class AwakeNode, 并注册节点awake_node
# loop：与ASRPro串口通信，收到唤醒指令后，
# 1.调用srv_tts，回复“我在”
# 2.调用srv_stt，录音转文本
# 3.发布msg_ai_status_judge, 后续都丢给ai

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from caigou_interfaces.srv import TTS, STT
import serial
import serial.tools.list_ports

class AwakeNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # 与ASRPro板创建串口连接
        asrpro_vendor_id = 0x1a86  # 十六进制的 Vendor ID
        asrpro_product_id = 0x7522  # 十六进制的 Product ID
        asrpro_serial_port = self.find_serial_port(asrpro_vendor_id, asrpro_product_id)
        self.serial_asrpro = serial.Serial(asrpro_serial_port, 115200, timeout=1)
        self.get_logger().info(f'INFO --- Connected with ASRPro serial port: {asrpro_serial_port}')

        # 创建srv_tts客户端
        self.srv_tts_client = self.create_client(TTS, 'srv_tts')
        # 创建srv_stt客户端
        self.srv_stt_client = self.create_client(STT, 'srv_stt')
        # 创建msg_ai_status_judge发布器
        self.msg_ai_status_judge_pub = self.create_publisher(String, 'msg_ai_status_judge', 10)

        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')
        self.loop()
    
    def loop(self):
        while rclpy.ok():
            if self.serial_asrpro.in_waiting > 0:
                received_data = self.serial_asrpro.readline().decode().strip()
                # self.get_logger().info(f'INFO --- self.serial_asrpro received: {received_data}')
                if received_data == 'awake':
                    self.get_logger().info(f'INFO --- ASRPro serial port received_data = {received_data}')
                    # 1. 调用srv_tts，"我在这里"，直到语音播报结束
                    while not self.srv_tts_client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('WARN --- srv_tts not available, waiting...')

                    tts_request = TTS.Request()
                    tts_request.tts_req = '我在这里'
                    tts_future = self.srv_tts_client.call_async(tts_request)
                    rclpy.spin_until_future_complete(self, tts_future)

                    tts_response = tts_future.result()  
                    if not tts_response.tts_res:  # bool
                        self.get_logger().error(f'ERROR --- tts_response.tts_res = {tts_response.tts_res}')
                    
                    # 2.调用srv_stt，录音转文本
                    while not self.srv_stt_client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('WARN --- srv_stt not available, waiting...')

                    stt_request = STT.Request()
                    stt_request.stt_req = True
                    stt_future = self.srv_stt_client.call_async(stt_request)
                    rclpy.spin_until_future_complete(self, stt_future)

                    stt_response = stt_future.result()
                    stt_res_txt = stt_response.stt_res  # String

                    # 3.发布msg_ai_status_judge
                    msg_ai_status_judge = String()
                    msg_ai_status_judge.data = stt_res_txt
                    self.msg_ai_status_judge_pub.publish(msg_ai_status_judge)
                    self.get_logger().info(f'INFO --- stt_res_txt = {stt_res_txt}, has been published to msg_ai_status_judge')
    
    def find_serial_port(self, vendor_id, product_id):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if port.vid == vendor_id and port.pid == product_id:
                print(f"Found target device on port {port.device}")
                return port.device
        return None


def main(args=None):
    rclpy.init(args=args)
    awake_node = AwakeNode('awake_node')
    try:
        rclpy.spin(awake_node)
    except KeyboardInterrupt:
        pass
    finally:
        awake_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()