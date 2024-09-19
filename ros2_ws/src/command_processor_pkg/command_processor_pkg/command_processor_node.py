#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandProcessorNode(Node):
    def __init__(self):
        super().__init__('command_processor_node')
        
        # Subscriber to /result topic for transcribed text
        self.subscription = self.create_subscription(
            String,
            '/result',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Publisher to /text_to_speech for responses
        self.tts_publisher = self.create_publisher(String, '/text_to_speech', 10)

        # Publisher to /robot_commands for executing actions
        self.command_publisher = self.create_publisher(String, '/robot_commands', 10)

        # Initialize robot mode
        self.mode = 'idle'  # Default mode

        self.get_logger().info('Command Processor Node has been started.')

    def listener_callback(self, msg):
        command_text = msg.data.lower().strip()
        self.get_logger().info(f'Received command: "{command_text}"')

        if 'change mode to' in command_text:
            self.handle_mode_change(command_text)
        elif 'what is your name' in command_text:
            self.respond("I am your assistant robot.")
        elif any(cmd in command_text for cmd in ['move forward', 'move_forward',
                                                'move backward', 'move_backward',
                                                'turn left', 'turn_left',
                                                'turn right', 'turn_right',
                                                'stop']):
            self.handle_movement_command(command_text)
        else:
            self.respond("I'm sorry, I didn't understand that command.")

    def handle_mode_change(self, command):
        try:
            # Extract mode from the command
            new_mode = command.split('change mode to')[-1].strip()
            if new_mode:
                self.mode = new_mode
                response = f'Mode changed to {self.mode}.'
                self.get_logger().info(response)
                self.respond(response)
                # Publish the mode change to robot_commands
                self.command_publisher.publish(String(data=f'set_mode_{self.mode}'))
            else:
                raise ValueError("No mode specified.")
        except Exception as e:
            error_msg = "Failed to change mode. Please specify a valid mode."
            self.get_logger().error(f'{error_msg} Error: {e}')
            self.respond(error_msg)

    def handle_movement_command(self, command):
        # Forward movement commands directly
        self.command_publisher.publish(String(data=command))
        self.respond(f'Executing command: {command.replace("_", " ")}.')

    def respond(self, message):
        response_msg = String()
        response_msg.data = message
        self.tts_publisher.publish(response_msg)
        self.get_logger().info(f'Responded with: "{message}"')

def main(args=None):
    rclpy.init(args=args)
    node = CommandProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Command Processor Node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

