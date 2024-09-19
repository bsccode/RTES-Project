#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TtsPublisher(Node):
    def __init__(self):
        super().__init__('tts_publisher')
        self.publisher_ = self.create_publisher(String, 'text_to_speech', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz
        self.get_logger().info('Text-to-Speech Publisher Node has been started.')

    def timer_callback(self):
        user_input = input("Enter text to speak (or 'exit' to quit): ")
        if user_input.lower() == 'exit':
            self.get_logger().info('Exiting publisher node...')
            rclpy.shutdown()
            return
        msg = String()
        msg.data = user_input
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = TtsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
