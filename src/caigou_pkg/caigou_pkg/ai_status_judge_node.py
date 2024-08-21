# 创建class AIStatusJudgeNode, 并注册节点ai_status_judge_node
# 1.创建msg_ai_status_judge订阅器，接收来自awake_node被唤醒后用户说的消息
# 2.创建msg_status发布器，用于向各个独立功能节点发布状态码
# 3.创建srv_task_schedule服务端，接收各个独立节点的完成情况并结合msg_status做相应调度措施

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from caigou_interfaces.srv import TTS, TaskSchedule, DataBase
import time
import json

# from zhipuai import ZhipuAI
# from openai import OpenAI
api_key4 = "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"
import os

os.environ["http_proxy"] = "http://localhost:7890"
os.environ["https_proxy"] = "http://localhost:7890"


# chat4 = OpenAI(api_key=api_key4)
# kimi ai
# client = OpenAI(
#     api_key="sk-74bGYkGde3oNCesfcQZL0sIuD2iPsyJVw9qntmsA7l74yheA",
#     base_url="https://api.moonshot.cn/v1",
# )
class AIStatusJudgeNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # 创建srv_tts客户端
        self.srv_tts_client = self.create_client(TTS, 'srv_tts')

        # 创建msg_ai_status_judge订阅器
        self.msg_ai_status_judge_sub = self.create_subscription(String, 'msg_ai_status_judge',
                                                                self.msg_ai_status_judge_callback, 10)

        # 创建msg_status发布器
        self.msg_status_pub = self.create_publisher(String, 'msg_status', 10)

        # 创建任务调度服务端
        self.srv_task_schedule_server = self.create_service(TaskSchedule, 'srv_task_schedule',
                                                            self.srv_task_schedule_callback)
        self.srv_database_client = self.create_client(DataBase, 'srv_database')

        # AI端
        # self.client = ZhipuAI(api_key="98d62759d6a447a8f50954c92ee81686.eAkIWyInXj7YTzp9") # 请填写您自己的APIKey

        # 打开prompt.txt文件
        with open('judge_status_system_prompt.txt', 'r') as f:
            self.judge_status_system_prompt = f.read()
        with open('judge_status_assistant_prompt.txt', 'r') as f:
            self.judge_status_assistant_prompt = f.read()

        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')

        # debug
        # time.sleep(15)
        # self.pub_msg_status("sort_start")

    def msg_ai_status_judge_callback(self, msg):  # 目前该函数只会收到来自awake_node被唤醒后用户说的消息
        user_input = msg.data
        self.get_logger().info(f'INFO --- msg_ai_status_judge received user_input = {user_input}')

        if len(user_input) > 0:
            judge_result = self.ai_judge_status(user_input)  # AI判断用户指定的task/status：1.init 2.sort 3.pickup
            judge_result = self.reinforce_judge_status(judge_result)  # 规整
            self.get_logger().info(f'INFO --- {self.get_name()} judge_result = {judge_result}')

            self.pub_msg_status(judge_result)  # 发布状态码
            self.get_logger().info(f'INFO --- pub_msg_status(judge_result) = {judge_result}')

        else:
            self.get_logger().info(f'WARN --- user_input = {user_input}')
            self.pub_msg_status("free")

    def pub_msg_status(self, status):  # # 发布状态码“free", "sort_start", "init_start"...
        msg_status = String()
        msg_status.data = status
        self.msg_status_pub.publish(msg_status)
        self.tell_to_database({"renew_status": status}, callback_func=None)

    # def 

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

    def srv_task_schedule_callback(self, request, response):  # 接收到从其他各个独立节点的完成情况，再结合msg_status做相应调度措施
        task_schedule_req = request.task_schedule_req  # "init_finished", "sort_finished", "not_initialized"
        self.get_logger().info(f'INFO --- received task_schedule_req = {task_schedule_req}')

        if 'init' in task_schedule_req:  # 初始化相关
            if task_schedule_req == 'init_finished':  # 初始化完成了，先休息
                self.pub_msg_status("free")
            elif task_schedule_req == 'not_initialized':  # 检测到sort等节点报告未初始化，就都先休息，目前等人工干预初始化
                self.pub_msg_status("free")

        elif 'sort' in task_schedule_req:  # 分拣相关
            if task_schedule_req == 'sort_finished':  # 分拣完成了
                self.pub_msg_status("sort_wait_for_new_package")
            elif task_schedule_req == 'sort_goto_unloading_area_finished':  # 小车到卸货区完成了，开始卸货
                self.pub_msg_status("sort_package_tracking")
            elif task_schedule_req == 'sort_get_package_finished':  # 已经拿到并装好包裹了，开始前往快递柜
                self.pub_msg_status("sort_goto_cabinets")
            elif task_schedule_req == 'sort_goto_cabinets_finished':  # 到达快递柜，开始上架
                self.pub_msg_status("sort_put_package")
        elif 'pickup' in task_schedule_req:  # 寄件相关
            if task_schedule_req == "pickup_goto_loading_area_finished":  # 小车到达寄件区
                self.pub_msg_status("pickup_package_tracking")  # 开始包裹检测
            elif task_schedule_req == "pickup_get_package_finished":  # 已经抓到了寄件包裹
                self.pub_msg_status("pickup_goto_unloading_area")
            elif task_schedule_req == "pickup_goto_unloading_area_finished":  # 已经到达公司揽件区了
                self.pub_msg_status("pickup_put_package")
            elif task_schedule_req == "pickup_finished":  # 已经把包裹放好了
                self.pub_msg_status("pickup_wait_for_new_package")

        else:
            self.pub_msg_status("free")

        response.task_schedule_res = True
        self.get_logger().info(f'INFO --- return True')
        return response

    def reinforce_judge_status(self, judge_result):  # 根据ai的初始回复进行人工规整，增强鲁棒性
        final_judge_result = "sort_start"  # 默认分拣状态
        if "init" in judge_result:
            final_judge_result = "init_start"
        elif "sort" in judge_result:
            final_judge_result = "sort_start"
        elif "pickup" in judge_result:
            final_judge_result = "pickup_start"
        else:
            final_judge_result = "sort_start"
        return final_judge_result

    def ai_judge_status(self, user_input):  # 此函数用来调api
        # response = self.client.chat.completions.create(
        #     model="glm-4", # 填写需要调用的模型名称
        #     messages = [
        #         {"role": "system","content": self.judge_status_system_prompt},
        #         {"role": "assistant","content": self.judge_status_assistant_prompt},
        #         {"role": "user","content": user_input}
        #         ],
        # )
        # response = chat4.chat.completions.create(
        #     model="gpt-3.5-turbo",  # 填写需要调用的模型名称
        #     messages=[
        #             {"role": "system", "content": self.judge_status_system_prompt},
        #             {"role": "assistant", "content": self.judge_status_assistant_prompt},
        #             {"role": "user", "content": user_input}
        #     ],
        #     # stream=True,
        #     temperature = 0,
        # )
        #         response = client.chat.completions.create(
        #         model="moonshot-v1-8k",
        #             messages=[
        #                     {"role": "system", "content": self.judge_status_system_prompt},
        #                     {"role": "assistant", "content": self.judge_status_assistant_prompt},
        #                     {"role": "user", "content": user_input}
        #         ],
        #         temperature=0.3,
        # )
        # 人工判断
        if "初始" in user_input:
            res = "init"
        elif "分拣" in user_input:
            res = "sort"
        elif "寄" in user_input or "计件" in user_input or "处理" in user_input:
            res = "pickup"
        else:
            res = "sort"
        # res = response.choices[0].message.content
        self.get_logger().info(f"ai origin res = {res}")
        return res


def main(args=None):
    rclpy.init(args=args)
    ai_status_judge_node = AIStatusJudgeNode('ai_status_judge_node')
    try:
        rclpy.spin(ai_status_judge_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_status_judge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
