#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import subprocess
from std_msgs.msg import String

class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__('text_to_speech_node')
        self.subscription = self.create_subscription(
            String,
            'text_to_speech',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Path to the alba.onnx model in the home directory
        self.model_path = os.path.expanduser("~/alba.onnx")
        self.output_file = 'output.wav'

    def listener_callback(self, msg):
        text = msg.data
        self.text_to_speech(text)

    def text_to_speech(self, text):
        # Run the Piper command
        command = ['piper', '--model', self.model_path, '--output_file', self.output_file]
        try:
            subprocess.run(command, input=text.encode('utf-8'), check=True)
            # Play the output file
            subprocess.run(['aplay', self.output_file], check=True)
            self.get_logger().info(f"Spoken: {text}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error in text_to_speech: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
