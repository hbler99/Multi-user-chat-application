import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray


class AudioSubscriber(Node):

    def __init__(self):
        super().__init__('audio_subscriber')

        # 订阅第一个话题
        self.subscription_audio = self.create_subscription(
            Int8MultiArray,
            'audio_data',
            self.audio_listener_callback,
            10)

        # 订阅第二个话题
        self.subscription_daudio = self.create_subscription(
            Int8MultiArray,
            'PC_daudio_data',
            self.daudio_listener_callback,
            10)

    def audio_listener_callback(self, msg):
        audio_data = msg.data
        self.get_logger().info('Received audio data: "%s"' % str(audio_data[:10]))  # 仅打印前10个数据样本
        # 这里可以添加进一步处理音频数据的代码，例如保存到文件或进行实时处理

    def daudio_listener_callback(self, msg):
        daudio_data = msg.data
        self.get_logger().info('Received PC_daudio_data: "%s"' % str(daudio_data[:10]))  # 仅打印前10个数据样本
        # 这里可以添加进一步处理音频数据的代码，例如保存到文件或进行实时处理


def main(args=None):
    rclpy.init(args=args)
    audio_subscriber = AudioSubscriber()
    rclpy.spin(audio_subscriber)
    audio_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
