import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random
import time

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Publisher to /cmd_vel
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Subscriber to /robot_commands
        self.command_subscription = self.create_subscription(
            String,
            '/robot_commands',
            self.command_callback,
            qos_profile)
        self.command_subscription  # Prevent unused variable warning

        # Roaming parameters
        self.roaming_speed = 0.6  # Increased speed
        self.turn_speed = 1.0     # Increased turn speed
        self.roaming_duration = 5.0  # Seconds per movement
        self.current_movement = None
        self.movement_end_time = None

        # Timer for roaming behavior (higher polling rate)
        self.roaming_timer = self.create_timer(0.5, self.roaming_behavior)  # 0.5-second interval

        # Current mode
        self.mode = 'idle'

        self.get_logger().info('Robot Control Node has been started.')

    def command_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received robot command: "{command}"')

        if command.startswith('set_mode_'):
            mode = command.split('set_mode_')[-1]
            self.set_mode(mode)
        elif self.mode == 'roaming':
            # Ignore manual movement commands in roaming mode
            self.get_logger().warn('Currently in roaming mode. Manual movement commands are ignored.')
        else:
            self.handle_movement_command(command)

    def set_mode(self, mode):
        if mode == 'roaming':
            self.enter_roaming_mode()
        elif mode in ['idle', 'active']:
            self.exit_roaming_mode()
            self.mode = mode
            self.get_logger().info(f'Switched to mode: {self.mode}')
            # Additional behaviors for 'active' mode can be implemented here
        else:
            self.get_logger().warn(f'Unknown mode: "{mode}"')

    def enter_roaming_mode(self):
        if self.mode != 'roaming':
            self.mode = 'roaming'
            self.current_movement = None
            self.movement_end_time = None
            self.get_logger().info('Entering Roaming Mode.')

    def exit_roaming_mode(self):
        if self.mode == 'roaming':
            self.mode = 'idle'
            self.current_movement = None
            self.movement_end_time = None
            self.get_logger().info('Exiting Roaming Mode.')
            self.stop_robot()

    def handle_movement_command(self, command):
        twist = Twist()

        if command in ['move forward', 'move_forward']:
            twist.linear.x = self.roaming_speed
            twist.angular.z = 0.0
            self.get_logger().info('Executing: Move Forward')
        elif command in ['move backward', 'move_backward']:
            twist.linear.x = -self.roaming_speed
            twist.angular.z = 0.0
            self.get_logger().info('Executing: Move Backward')
        elif command in ['turn left', 'turn_left']:
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            self.get_logger().info('Executing: Turn Left')
        elif command in ['turn right', 'turn_right']:
            twist.linear.x = 0.0
            twist.angular.z = -self.turn_speed
            self.get_logger().info('Executing: Turn Right')
        elif command == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Executing: Stop')
        else:
            self.get_logger().warn(f'Unknown movement command: "{command}"')
            return  # Exit without publishing

        # Publish the Twist message to /cmd_vel
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Published Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

    def roaming_behavior(self):
        if self.mode != 'roaming':
            return  # Do nothing if not in roaming mode

        current_time = self.get_clock().now().to_msg()
        if self.movement_end_time and rclpy.time.Time.from_msg(current_time) >= self.movement_end_time:
            # Movement duration ended, choose a new movement
            self.current_movement = None

        if not self.current_movement:
            # Select a new movement direction
            self.current_movement = random.choice(['forward', 'backward', 'left', 'right'])
            self.movement_end_time = rclpy.time.Time.from_msg(current_time) + rclpy.duration.Duration(seconds=self.roaming_duration)
            self.get_logger().info(f'Roaming: Selected movement "{self.current_movement}" for {self.roaming_duration} seconds.')

        # Create Twist message based on current movement
        twist = Twist()
        if self.current_movement == 'forward':
            twist.linear.x = self.roaming_speed
            twist.angular.z = 0.0
        elif self.current_movement == 'backward':
            twist.linear.x = -self.roaming_speed
            twist.angular.z = 0.0
        elif self.current_movement == 'left':
            twist.linear.x = self.roaming_speed
            twist.angular.z = self.turn_speed
        elif self.current_movement == 'right':
            twist.linear.x = self.roaming_speed
            twist.angular.z = -self.turn_speed

        # Publish the Twist message
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Roaming Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Robot Control Node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
