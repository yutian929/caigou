import socket
import sqlite3
import json
import random
import os
from pydub import AudioSegment
from playsound import playsound

current_status = '初始化'
code_to_tracking = {}

# 一些模拟的快递信息
recipients = [
    ('张雨田', '18012964897', '广东省深圳市'),
    ('李思雨', '18234567891', '上海市浦东新区'),
    ('王小明', '13912345678', '北京市海淀区'),
    ('赵大鹏', '13776543210', '成都市锦江区'),
    ('陈美丽', '13678901234', '广州市天河区')
]
senders = [
    ('李昌', '18123456789', '北京市朝阳区'),
    ('刘华', '18098765432', '天津市南开区'),
    ('王芳', '18687654321', '杭州市西湖区'),
    ('孙健', '13987654321', '南京市鼓楼区'),
    ('马聪', '13876543210', '武汉市武昌区')
]
import pyaudio
import wave

def play_wave_file(filename):
    # 打开 WAV 文件
    wf = wave.open(filename, 'rb')

    # 创建 PyAudio 对象
    p = pyaudio.PyAudio()

    # 打开一个流来播放音频
    stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True)

    # 读取数据块并播放
    chunk_size = 1024
    data = wf.readframes(chunk_size)
    while data:
        stream.write(data)
        data = wf.readframes(chunk_size)

    # 停止流
    stream.stop_stream()
    stream.close()

    # 关闭 PyAudio
    p.terminate()

# 查询数据库中未放置快递的取件码
def get_free_warehouse_codes():
    try:
        conn = sqlite3.connect('../express.db')
        cursor = conn.cursor()
        query = "SELECT pickup_code FROM express_info WHERE tracking_number IS NULL"
        cursor.execute(query)
        rows = cursor.fetchall()
        free_codes = [row[0] for row in rows]
        conn.close()
        return free_codes  # 返回取件码列表,eg ['1-1-2', '1-1-3', '1-1-4', '1-1-5']
    except Exception as e:
        print(f"ERROR --- Database query failed: {str(e)}")
        return []

# 生成随机的快递信息
def generate_package_info(tracking_number, pickup_code):
    recipient = random.choice(recipients)
    sender = random.choice(senders)
    return {
        'tracking_number': tracking_number,
        'pickup_code': pickup_code,
        'recipient_name': recipient[0],
        'recipient_phone': recipient[1],
        'recipient_address': recipient[2],
        'sender_name': sender[0],
        'sender_phone': sender[1],
        'sender_address': sender[2]
    }
# 更新数据库
def update_database_with_package_info(package_info):
    try:
        conn = sqlite3.connect('../express.db')
        cursor = conn.cursor()
        cursor.execute("""
            INSERT INTO express_info (tracking_number, pickup_code, recipient_name, recipient_phone, recipient_address, sender_name, sender_phone, sender_address)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(pickup_code) DO UPDATE SET
            tracking_number = excluded.tracking_number,
            recipient_name = excluded.recipient_name,
            recipient_phone = excluded.recipient_phone,
            recipient_address = excluded.recipient_address,
            sender_name = excluded.sender_name,
            sender_phone = excluded.sender_phone,
            sender_address = excluded.sender_address
        """, (package_info['tracking_number'], package_info['pickup_code'], package_info['recipient_name'], package_info['recipient_phone'],
              package_info['recipient_address'], package_info['sender_name'], package_info['sender_phone'], package_info['sender_address']))
        conn.commit()
        conn.close()
        print(f"INFO --- Updated database: {package_info['pickup_code']} -> {package_info['tracking_number']}")
    except Exception as e:
        print(f"ERROR --- Database update failed: {str(e)}")

# 查询delivery.db中的有多少个sent为1的订单
def get_sent_count():
    try:
        conn = sqlite3.connect('../delivery.db')
        cursor = conn.cursor()
        query = "SELECT COUNT(*) FROM deliveries WHERE sent = 1"
        cursor.execute(query)
        sent_count = cursor.fetchone()[0]
        conn.close()
        print(f"INFO --- Sent count: {sent_count}")
        return sent_count
    except Exception as e:
        print(f"ERROR --- Database query failed: {str(e)}")
        return 0
    
# 把其中一个sent为1的订单的sent改为0
def update_sent_status():
    try:
        conn = sqlite3.connect('../delivery.db')
        cursor = conn.cursor()
        query = "SELECT id FROM deliveries WHERE sent = 1 LIMIT 1"
        cursor.execute(query)
        row = cursor.fetchone()
        if row:
            order_id = row[0]
            cursor.execute("UPDATE deliveries SET sent = 0 WHERE id = ?", (order_id,))
            conn.commit()
            conn.close()
            print(f"INFO --- Updated sent status for order: {order_id}")
            return 1
        else:
            conn.close()
            return 0
    except Exception as e:
        print(f"ERROR --- Database query failed: {str(e)}")
        return 0
    
def play_notification_audio(pickup_code, recipient_phone):
    try:
        base_path = 'audios/'
        audio_parts = [
            "尾号.wav",
            f"{recipient_phone[-4]}.wav",
            f"{recipient_phone[-3]}.wav",
            f"{recipient_phone[-2]}.wav",
            f"{recipient_phone[-1]}.wav",
            "的用户.wav",
            "您的快递已上架.wav",
            "取件码是.wav",
            f"{pickup_code[0]}.wav",
            "杠.wav",
            f"{pickup_code[2]}.wav",
            "杠.wav",
            f"{pickup_code[4]}.wav"
        ]
        combined_audio = AudioSegment.empty()
        for part in audio_parts:
            audio_path = os.path.join(base_path, part)
            if os.path.exists(audio_path):
                audio = AudioSegment.from_file(audio_path, format='wav')
                combined_audio += audio
            else:
                print(f"ERROR --- Audio file not found: {audio_path}")
        combined_audio.export('notification.wav', format='wav')
        play_wave_file('notification.wav')
        # playsound('notification.wav')
        # os.remove('notification.wav')
        print(f"INFO --- Played notification audio for pickup code: {pickup_code}") 
    except Exception as e:
        print(f"ERROR --- Error playing notification audio: {str(e)}")
def handle_client_connection(client_socket):
    try:
        request = client_socket.recv(1024)
        request_data = json.loads(request.decode())
        print(f"INFO --- Received request: {request_data}")

        if 'ask_for_free_warehouse' in request_data:
            free_codes = get_free_warehouse_codes()
            free_code = free_codes.pop(0) if free_codes else 0  # 要么是"1-1-2"等，要么是0
            if free_code:
                nums = free_code.split('-')
                num_code = (int(nums[0]), int(nums[1]), int(nums[2]))
                code_to_tracking[free_code] = None
                response_data = {'ask_for_free_warehouse': num_code}
                print(f"INFO --- Sent free warehouse code: {num_code}")
            else:
                response_data = {'ask_for_free_warehouse': 0}

        elif 'renew_database' in request_data:
            cab, shelf, warehouse, str_tracking_number = request_data['renew_database']
            str_code = f"{cab}-{shelf}-{warehouse}"
            code_to_tracking[str_code] = str_tracking_number
            package_info = generate_package_info(str_tracking_number, str_code)
            print(f"INFO --- Generated package info: {package_info}")

            update_database_with_package_info(package_info)
            play_notification_audio(str_code, package_info['recipient_phone'])
            response_data = {'renew_database': 1}
            print(f"INFO --- Updated database: {str_code} -> {str_tracking_number}")

        elif 'renew_status' in request_data:
            # 保存状态到json文件中
            current_status = request_data['renew_status']
            if current_status == 'temp':
                count = get_sent_count()
                if count > 0:
                    # 回复客户端，寄件状态为1
                    response_data = {'renew_status': 1}
                    update_sent_status()
                else:
                    # 回复客户端，寄件状态为0
                    response_data = {'renew_status': 0}
            with open('../status.json', 'w') as f:
                json.dump({'status': current_status}, f)
            response_data = {'renew_status': 0}
            print(f"INFO --- Updated status: {current_status}")
        else:
            response_data = {}

        response_json = json.dumps(response_data)
        client_socket.sendall(response_json.encode())
    except Exception as e:
        # print(f"ERROR --- Error handling client request: {str(e)}")
        error_response = json.dumps({'error': str(e)})
        client_socket.sendall(error_response.encode())
 

def main():
    server_ip = '0.0.0.0'  # 监听所有IP地址
    server_port = 54321

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((server_ip, server_port))
    server.listen(5)
    print(f"INFO --- Server listening on {server_ip}:{server_port}")
    client_socket, addr = server.accept()
    print(f"INFO --- Accepted connection from {addr}")
    while True:
        handle_client_connection(client_socket)

if __name__ == '__main__':
    main()
