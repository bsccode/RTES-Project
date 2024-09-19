#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ResultToTtsNode(Node):
    def __init__(self):
        super().__init__('result_to_tts_node')
        # Subscriber to /result topic
        self.subscription = self.create_subscription(
            String,
            '/result',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Publisher to /text_to_speech topic
        self.publisher_ = self.create_publisher(String, '/text_to_speech', 10)

        self.get_logger().info('Result to Text-to-Speech Bridge Node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received from /result: "{msg.data}"')
        # Republish the same message to /text_to_speech
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published to /text_to_speech: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ResultToTtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
