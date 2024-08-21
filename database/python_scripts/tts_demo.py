import os
import sys
import json
from urllib.request import urlopen, Request
from urllib.parse import urlencode, quote_plus
import pyaudio
import wave

class DemoError(Exception):
    pass

class TextToSpeech:
    def __init__(self):
        self.API_KEY = 'Szxla1tbL5yTUiJhdzIQj6Wg'
        self.SECRET_KEY = 'f8otDUl0LzUwgKm3qtVNLKRgb2W8Zf2T'
        
        self.PER = 4  # 发音人
        self.SPD = 5  # 语速
        self.PIT = 5  # 音调
        self.VOL = 5  # 音量
        self.AUE = 6  # 下载的文件格式
        
        FORMATS = {3: "mp3", 4: "pcm", 5: "pcm", 6: "wav"}
        self.FORMAT = FORMATS[self.AUE]

        self.CUID = "123456PYTHON"
        self.TTS_URL = 'http://tsn.baidu.com/text2audio'
        self.TOKEN_URL = 'http://aip.baidubce.com/oauth/2.0/token'
        self.SCOPE = 'audio_tts_post'

        self.token = self.fetch_token()
        self.audio_file_path = 'output.wav'

    def fetch_token(self):
        params = {'grant_type': 'client_credentials',
                  'client_id': self.API_KEY,
                  'client_secret': self.SECRET_KEY}
        post_data = urlencode(params).encode('utf-8')
        req = Request(self.TOKEN_URL, post_data)
        try:
            f = urlopen(req, timeout=5)
            result_str = f.read().decode()
        except URLError as err:
            result_str = err.read().decode()

        result = json.loads(result_str)
        if 'access_token' in result and 'scope' in result:
            if self.SCOPE not in result['scope'].split(' '):
                raise DemoError('scope is not correct')
            return result['access_token']
        else:
            raise DemoError('API_KEY or SECRET_KEY may not be correct')

    def generate_audio(self, text):
        tex = quote_plus(text)
        params = {
            'tok': self.token, 'tex': tex, 'per': self.PER, 'spd': self.SPD, 
            'pit': self.PIT, 'vol': self.VOL, 'aue': self.AUE, 'cuid': self.CUID, 
            'lan': 'zh', 'ctp': 1
        }
        data = urlencode(params).encode('utf-8')
        req = Request(self.TTS_URL, data)
        try:
            f = urlopen(req)
            result_str = f.read()

            headers = {name.lower(): value for name, value in f.headers.items()}
            if 'content-type' not in headers or headers['content-type'].find('audio/') < 0:
                raise DemoError("TTS API error")
        except URLError as err:
            result_str = err.read()
            raise DemoError(f"TTS API error: {result_str}")

        with open(self.audio_file_path, 'wb') as of:
            of.write(result_str)
        print(f"Audio saved as: {self.audio_file_path}")

    def play_audio(self):
        chunk_size = 1024
        wf = wave.open(self.audio_file_path, 'rb')
        p = pyaudio.PyAudio()
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)
        data = wf.readframes(chunk_size)
        while data:
            stream.write(data)
            data = wf.readframes(chunk_size)
        stream.stop_stream()
        stream.close()
        p.terminate()

if __name__ == '__main__':
    tts = TextToSpeech()
    text = "你好，这是一个文本转语音的示例。"
    tts.generate_audio(text)
    tts.play_audio()
