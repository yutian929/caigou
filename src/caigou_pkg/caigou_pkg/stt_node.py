# 创建class STTNode, 并注册节点stt_node
# 创建服务端，服务名：stt_service
# 接收到服务请求，开始录音
# 录音完成后，云端API语音识别
# 识别结果发布给客户端
import rclpy
from rclpy.node import Node
from caigou_interfaces.srv import STT
import pyaudio
import wave
import numpy as np
import time
import sys
import json
from funasr import AutoModel

class STTNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.srv_stt_server = self.create_service(STT, 'srv_stt', self.srv_stt_callback)
        # 在终端调试这个服务时，可以输入 ros2 service call /srv_stt caigou_interfaces/srv/STT "{stt_req: true}"

        # 需要识别的文件
        self.AUDIO_FILE = 'record.wav'
        self.model = AutoModel(model="paraformer-zh", model_revision="v2.0.4",
                  vad_model="fsmn-vad", vad_model_revision="v2.0.4",
                  punc_model="ct-punc-c", punc_model_revision="v2.0.4",
                  # spk_model="cam++", spk_model_revision="v2.0.2",
                  )
        self.get_logger().info(f'INFO --- {node_name} setup finished ^.^')
    
    def srv_stt_callback(self, request, response):
        if request.stt_req:
            self.get_logger().info("INFO --- stt_node received request")
            self.record_audio()
            self.get_logger().info(f"INFO --- self.speech_recognition() = {self.speech_recognition()}")
            try:
                response.stt_res = self.speech_recognition()
                self.get_logger().info(f"INFO --- stt_node recognized = {response.stt_res}")
            except Exception as e:
                self.get_logger().error(f"ERROR --- stt_node speech_recognition() error = {e}")
                response.stt_res = ""
        return response
    
    def record_audio(self):
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        RECORD_SECONDS = 7 #录音时长
        NOISE_SAMPLES = 5  #噪声样本数
        MARGIN = 1.5  #动态阈值倍数
        WAVE_OUTPUT_FILENAME = self.AUDIO_FILE
        p = pyaudio.PyAudio()
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)
        self.get_logger().info("Measuring noise...")
        noise_frames = [stream.read(CHUNK) for _ in range(NOISE_SAMPLES)]
        noise_energy = np.mean([np.linalg.norm(np.frombuffer(frame, dtype=np.int16)) / CHUNK for frame in noise_frames])
        THRESHOLD = noise_energy * MARGIN
        self.get_logger().info(f"Dynamic threshold set to: {THRESHOLD}")

        self.get_logger().info("* recording")
        frames = []
        silent_count = 0  
        start_recording = False
        start_time = time.time()
        while time.time() - start_time < RECORD_SECONDS:
            data = stream.read(CHUNK)

            energy = np.linalg.norm(np.frombuffer(data, dtype=np.int16)) / CHUNK

            if energy < THRESHOLD and start_recording:
                silent_count += 1
                frames.append(data)
            elif energy < THRESHOLD and not start_recording:
                continue
            elif energy > THRESHOLD and not start_recording:
                start_recording = True
                self.get_logger().info(f"start_energy = {energy}")
                frames.append(data)
            elif energy > THRESHOLD and start_recording:
                frames.append(data)
                silent_count = max(0, silent_count-1) 

            if silent_count > 30: 
                break

        self.get_logger().info("* done recording")
        stream.stop_stream()
        stream.close()
        p.terminate()
        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        time.sleep(0.05)

    def speech_recognition(self):
        self.get_logger().info("Start speech recognition...")
        try:
            res = self.model.generate(input=self.AUDIO_FILE, 
                batch_size_s=300, 
                hotword='魔搭')
            return res[0]['text'] if res else ""
        except Exception as e:
            self.get_logger().error(f"ERROR --- speech_recognition() error = {e}")
            return ""

def main(args=None):
    rclpy.init(args=args)
    stt_node = STTNode('stt_node')
    try:
        rclpy.spin(stt_node)
    except KeyboardInterrupt:
        pass
    finally:
        stt_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
