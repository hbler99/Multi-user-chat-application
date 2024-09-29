import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Int32, ByteMultiArray
import subprocess
import pyaudio, struct
# 设置音频流参数
FORMAT = pyaudio.paInt16  # 16-bit采样
CHANNELS = 1  # 单声道
RATE = 16000  # 采样率 (Hz)
CHUNK = 120  # 每次读取的帧数

# 初始化PyAudio对象
p = pyaudio.PyAudio()

# 打开音频流
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                frames_per_buffer=CHUNK,
                output=True)

audio = bytes()
send_audio = []
wav_data = bytes()
play_flag = False
message_id = 0

def save_amrnb_file(amrwb_data, output_filename):
    # AMR-WB 文件头
    amrwb_header = b"#!AMR-WB\n"
    # 将数据写入文件
    with open(output_filename, "wb") as f:
        # 写入文件头
        f.write(amrwb_header)
        # 写入 AMR-WB 音频数据
        for i in range(len(amrwb_data)):
            f.write(amrwb_data[i])

def generate_wav_header(sample_rate, bits_per_sample, num_channels, data_size):
    # RIFF header
    riff = b'RIFF'
    chunk_size = 36 + data_size
    wave = b'WAVE'

    # fmt subchunk
    fmt = b'fmt '
    subchunk1_size = 16
    audio_format = 1
    byte_rate = sample_rate * num_channels * bits_per_sample // 8
    block_align = num_channels * bits_per_sample // 8

    # data subchunk
    data = b'data'

    header = struct.pack('<4sI4s4sIHHIIHH4sI',
                         riff, chunk_size, wave, fmt, subchunk1_size,
                         audio_format, num_channels, sample_rate, byte_rate,
                         block_align, bits_per_sample, data, data_size)
    return header

def amrwb_decoder(amrwb_data):
    global message_id
    process = subprocess.Popen(
        ['ffmpeg', '-y', '-f', 'amr', '-i', 'pipe:0', '-map_metadata', '-1', '-fflags', '+bitexact', '-filter:a', 'volume=3', '-acodec', 'pcm_s16le',
         '-f', 'wav', '-ac', '1', '-ar', '16000', 'pipe:1'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=-1
    )
    amrwb_header = b'#!AMR-WB\n'
    out, err = process.communicate(input=amrwb_header + amrwb_data)
    if process.returncode != 0:
        print(f"ffmpeg error: {err.decode()}")
    return out


class AudioPlay(Node):
    def __init__(self):
        super().__init__('audio_play')
        self.qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # 创建订阅者
        self.subscription = self.create_subscription(
                                                ByteMultiArray,
                                                'Esp32_audio_data',   #根据 Esp32 设定 Topic name 做更改
                                                self.listener_callback,
                                                self.qos_profile)
        self.subscription  # 防止未被使用的变量被垃圾回收

    def listener_callback(self, msg):
        global audio, wav_data, play_flag
        for i in range(len(msg.data)):
            # audio.append(msg.data[i])
            audio += msg.data[i]
        if len(audio) >= CHUNK:
            wav_data = amrwb_decoder(audio)
            play_flag = True
            audio = bytes()
def main(args=None):
    global wav_data, play_flag, message_id
    stream.start_stream()
    rclpy.init(args=args)
    audio_data_relay = AudioPlay()
    try:
        while rclpy.ok():
            # recieve msg
            if play_flag:
                stream.write(wav_data[44:]) # 去掉帧头 44 字节
                message_id += 1
                wav_data = bytes()
                play_flag = False
            rclpy.spin_once(audio_data_relay, timeout_sec=0.002)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
