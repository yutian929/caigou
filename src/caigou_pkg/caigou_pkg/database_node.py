# 创建Database Node类，用于向数据库请求或更新数据
# 1.创建srv_database服务端，用于接收其他节点要向数据库请求或更新的json消息，然后收到数据库的响应再回复
# 2.创建tcp客户端，与数据库(服务端)建立连接，负责消息收发，进而控制数据库

import rclpy
from rclpy.node import Node
from caigou_interfaces.srv import DataBase
import json
import socket
import time

class DatabaseNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # self.tcp_server_ip = '192.168.32.84'
        # self.tcp_server_ip = '192.168.100.196'  # MIFI
        self.tcp_server_ip = '192.168.1.103'  # caigou
        self.tcp_server_port = 54321

        # 创建srv_database服务端
        self.srv_database_server = self.create_service(DataBase, 'srv_database', self.srv_database_callback)
        # we can debug this service by running the following command in the terminal:
        # ros2 service call /srv_database caigou_interfaces/srv/Database "{database_req: \'{\"ask_for_free_warehouse\": 0}\'}"
        # 如果要发一个字典 {"renew_database": ((1,1,1), 000001)}
        # ros2 service call /srv_database caigou_interfaces/srv/Database "{database_req: '{"ask_for_express_company_name": "000003"]}'}"
        
        # 创建tcp连接客户端
        self.tcp_client = self.setup_tcp_client()

        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')
    
    def srv_database_callback(self, request, response):
        # self.get_logger().info(f"INFO --- srv_database_callback called, request.database_req = {request.database_req}")
        if self.tcp_client:
            try:
                message = request.database_req  # 已经是json格式了
                # self.get_logger().info(f"INFO --- message to send to tcp server 0 = {message}")
                # 解包再打包
                message = json.loads(message)
                # self.get_logger().info(f"INFO --- message to send to tcp server 1 = {message}")
                message = json.dumps(message)
                # self.get_logger().info(f"INFO --- message to send to tcp server 2 = {message}")

                self.tcp_client.sendall(message.encode())
                # self.get_logger().info(f"INFO --- message to send to tcp server 3 = {message.encode()}")
                #
                received = self.tcp_client.recv(1024)
                received_data = json.loads(received.decode())
                # self.get_logger().info(f"INFO --- received_data from tcp server = {received_data}")
                response.database_res = json.dumps(received_data)

            except Exception as e:
                self.get_logger().error(f"ERROR --- Error handling database request: {str(e)}")
                response.database_res = ""
        else:
            self.get_logger().error("ERROR --- TCP not connected")
            response.database_res = ""
        # self.get_logger().info(f"INFO --- srv_database_callback response.database_res = {response.database_res}")
        return response

    def setup_tcp_client(self):
        try:
            tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_client.connect((self.tcp_server_ip, self.tcp_server_port))
            self.get_logger().info(f"INFO --- TCP client connected to {self.tcp_server_ip}:{self.tcp_server_port}")
            return tcp_client
        except Exception as e:
            self.get_logger().error(f"ERROR --- Error setting up TCP client: {str(e)}")
            return None
    
    def destroy_tcp_client(self):
        if self.tcp_client:
            self.tcp_client.close()
            self.get_logger().info("INFO --- TCP client closed")
        else:
            self.get_logger().error("ERROR --- TCP client not connected")
        

def main(args=None):
    rclpy.init(args=args)

    database_node = DatabaseNode("database_node")

    rclpy.spin(database_node)

    database_node.destroy_tcp_client()
    database_node.destroy_node()
    rclpy.shutdown()
    