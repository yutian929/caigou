# 创建class TTSNode, 并注册节点tts_node
# 创建服务端，服务名：srv_tts_server
# 接收到服务请求，开始合成语音
# 语音合成完成后，播放语音
import rclpy
from rclpy.node import Node
from caigou_interfaces.srv import TTS
# from paddlespeech.cli.tts.infer import TTSExecutor
import pygame
import time

class MyTTSNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.srv_tts_server = self.create_service(TTS, 'srv_tts', self.srv_tts_callback)
        # 在终端调试这个服务可以输入：ros2 service call /srv_tts caigou_interfaces/srv/TTS "{tts_req: '我在这里'}"
        # self.tts = TTSExecutor()
        
        self.video_file_path = 'play.wav'
        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')

    def srv_tts_callback(self, request, response):  # 

        # 主要流程
        text = request.tts_req
        self.get_logger().info(f"INFO --- tts_node received request: {text}")
        # self.generate_audio(text)
        # self.tts(text=text, output=self.video_file_path )
        # self.get_logger().info(f"INFO --- tts_node generated audio {text} finished.")
        self.play_audio(text)

        response.tts_res = True
        return response

    # def generate_audio(self, TEXT):
        # self.tts(text=TEXT, output=self.video_file_path )
        # self.get_logger().info(f"INFO --- tts_node generated audio {TEXT} finished.")

        # print("result saved as :" + save_file)
    def play_audio(self, text):
        print(text)
        if text == "我在这里":
            pygame.mixer.init()
            pygame.mixer.music.load("audios/我在这里.wav")
            pygame.mixer.music.set_volume(1.0)  # 设置音量为最大
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        elif text == "开始初始化":
            pygame.mixer.init()
            pygame.mixer.music.load("audios/下面开始初始化.wav")
            pygame.mixer.music.set_volume(1.0)  # 设置音量为最大    
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        elif text == "初始化完成":
            pygame.mixer.init()
            pygame.mixer.music.load("audios/初始化完成.wav")
            pygame.mixer.music.set_volume(1.0)  # 设置音量为最大
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        elif text == "开始分拣上架":
            pygame.mixer.init()
            pygame.mixer.music.load("audios/下面开始分拣上架.wav")
            pygame.mixer.music.set_volume(1.0)  # 设置音量为最大
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        elif text == "开始前往分拣区":
            time.sleep(1)
        elif text == "开始分拣":
            time.sleep(1)
        elif text == "开始前往快递柜":
            time.sleep(0.5)
        elif text == "开始上架":
            time.sleep(0.5)
        elif text == "已分拣完全部包裹":
            pygame.mixer.init()
            pygame.mixer.music.load("audios/已分拣完全部包裹.wav")
            pygame.mixer.music.set_volume(1.0)  # 设置音量为最大
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        elif text == "开始处理寄件包裹":   
            pygame.mixer.init()
            pygame.mixer.music.load("audios/下面开始处理寄件.wav")
            pygame.mixer.music.set_volume(1.0)  # 设置音量为最大
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        elif text == "开始前往揽货区":
            time.sleep(0.5)
        elif text == "开始放置包裹":
            time.sleep(0.5)
        elif text == "开始搬运寄件包裹":
            time.sleep(1)
        elif text == "开始前往寄件区":
            time.sleep(1)
        elif text == "已处理完全部寄件包裹": 
            pygame.mixer.init()
            pygame.mixer.music.load("audios/已处理完全部寄件.wav")
            pygame.mixer.music.set_volume(1.0)  # 设置音量为最大
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        else:
            pygame.mixer.init()
            pygame.mixer.music.load("audios/不是哥们没有这个语音.wav")
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
def main(args=None):
    rclpy.init(args=args)
    tts_node = MyTTSNode('tts_node')
    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        time.sleep(0.2)
    finally:
        tts_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()