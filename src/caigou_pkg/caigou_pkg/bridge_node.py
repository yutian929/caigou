# 创建BridgeNode类，并注册节点bridge_node
# 1.创建srv_bridge服务端，用于接收其他节点要向服务器发送的json消息，然后收到服务器的响应再回复
# 2.创建tcp客户端，与树莓派(服务端)建立连接，负责消息收发，进而控制小车底盘

import rclpy
from rclpy.node import Node
from caigou_interfaces.srv import Bridge
import json
import socket
import time

class BridgeNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # self.tcp_server_ip = '192.168.32.101'
        self.tcp_server_ip = '192.168.1.108'
        self.tcp_server_port = 12345

        # 创建srv_bridge服务端
        self.srv_bridge_server = self.create_service(Bridge, 'srv_bridge', self.srv_bridge_callback)
        # we can debug this service by running the following command in the terminal:
        # ros2 service call /srv_bridge caigou_interfaces/srv/Bridge "{bridge_req: '{\"set_angle\": -90.0}'}"
        # ros2 service call /srv_bridge caigou_interfaces/srv/Bridge "{bridge_req: '{\"set_pose\": [1.0, 0.5, 90]}'}"
        # ros2 service call /srv_bridge caigou_interfaces/srv/Bridge "{bridge_req: '{\"set_pose_precise\": [0.3, 0.3, 0]}'}"
        # ros2 service call /srv_bridge caigou_interfaces/srv/Bridge "{bridge_req: '{\"set_step\": \"w\"}'}"
        # 创建tcp连接客户端
        self.tcp_client = self.setup_tcp_client()

        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')

        # message = json.dumps({"set_step": ['w', 0.2]})
        # self.tcp_client.sendall(message.encode())
        # received = self.tcp_client.recv(1024)
        # received_data = json.loads(received.decode())
        # self.get_logger().info(f"INFO --- received_data from tcp server = {received_data}")
        # time.sleep(3)
        # message = json.dumps({"set_step": [' ', 0.0]})
        # self.tcp_client.sendall(message.encode())
        # received = self.tcp_client.recv(1024)
        # received_data = json.loads(received.decode())
        # self.get_logger().info(f"INFO --- received_data from tcp server = {received_data}")
        # time.sleep(3)
        # message = json.dumps({"set_step": ['s', 0.5]})
        # self.tcp_client.sendall(message.encode())
        # received = self.tcp_client.recv(1024)
        # received_data = json.loads(received.decode())
        # self.get_logger().info(f"INFO --- received_data from tcp server = {received_data}")
        # time.sleep(3)
        # message = json.dumps({"set_step": [' ', 0.0]})
        # self.tcp_client.sendall(message.encode())
        # received = self.tcp_client.recv(1024)
        # received_data = json.loads(received.decode())
        # self.get_logger().info(f"INFO --- received_data from tcp server = {received_data}")

    def srv_bridge_callback(self, request, response):
        # self.get_logger().info(f"INFO --- srv_bridge_callback called, request.bridge_req = {request.bridge_req}")

        # # DEBUG: 用于测试srv_bridge服务端是否正常工作
        # message = request.bridge_req
        # message_dict = json.loads(message)
        # time.sleep(3)
        # response.bridge_res = json.dumps(message_dict)
        # self.get_logger().info(f"INFO --- srv_bridge_callback response.bridge_res = {response.bridge_res}")
        # return response

        if self.tcp_client:
            try:
                message = request.bridge_req  # 已经是json格式了
                self.tcp_client.sendall(message.encode())
                received = self.tcp_client.recv(1024)
                received_data = json.loads(received.decode())
                # self.get_logger().info(f"INFO --- received_data from tcp server = {received_data}")
                response.bridge_res = json.dumps(received_data)

            except Exception as e:
                self.get_logger().error(f"ERROR --- Error handling bridge request: {str(e)}")
                response.bridge_res = ""
        else:
            self.get_logger().error("ERROR --- TCP not connected")
            response.bridge_res = ""
        # self.get_logger().info(f"INFO --- srv_bridge_callback response.bridge_res = {response.bridge_res}")
        return response

    def setup_tcp_client(self):
        """设置TCP连接客户端."""
        client = None
        while not client:
            try:
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.connect((self.tcp_server_ip, self.tcp_server_port))
                self.get_logger().info(f"Connected to tcp server at {self.tcp_server_ip}:{self.tcp_server_port}")
                return client
            except Exception as e:
                self.get_logger().error(f"Failed to connect to server: {str(e)}")
                if client:
                    client.close()
                    client = None
                time.sleep(1)



def main(args=None):
    rclpy.init(args=args)
    bridge_node = BridgeNode('bridge_node')
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
