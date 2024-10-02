import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Publisher to /text_to_speech
        self.tts_publisher = self.create_publisher(String, '/text_to_speech', 10)

        # Subscriber to /detected_color
        self.color_subscription = self.create_subscription(
            String,
            '/color_detected',
            self.color_detected_callback,
            10)
        self.color_subscription  # Prevent unused variable warning

        # Publisher to /cmd_vel with QoS profile
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Subscribe to front-left range sensor
        self.range_fl_subscription = self.create_subscription(
            LaserScan,
            '/range/fl',
            self.range_fl_callback,
            10)
        self.range_fl_subscription  # Prevent unused variable warning

        # Subscribe to front-right range sensor
        self.range_fr_subscription = self.create_subscription(
            LaserScan,
            '/range/fr',
            self.range_fr_callback,
            10)
        self.range_fr_subscription  # Prevent unused variable warning

        # Variables to store range data
        self.range_fl = None
        self.range_fr = None

        # Initialize obstacle_detected based on range sensors
        self.obstacle_detected = False

        # Subscriber to /robot_commands
        self.command_subscription = self.create_subscription(
            String,
            '/robot_commands',
            self.command_callback,
            qos_profile)
        self.command_subscription  # Prevent unused variable warning

        # Roaming parameters
        self.roaming_speed = 0.5       # Speed when moving forward (meters per second)
        self.turn_speed = 0.7          # Angular speed when turning (radians per second)
        self.roaming_duration = 0.5    # Duration to turn (seconds)
        self.current_movement = None
        self.movement_end_time = None

        # Timer for roaming behavior (0.1-second interval)
        self.roaming_timer = self.create_timer(0.1, self.roaming_behavior)

        # Current mode ('idol', 'roaming', or 'active')
        self.mode = 'idol'

        # Subscribe to /object_detected
        self.object_detected_subscription = self.create_subscription(
            Bool,
            '/object_detected',
            self.object_detected_callback,
            10)
        self.object_detected_subscription  # Prevent unused variable warning

        # Flag to track object detection
        self.object_detected = False

        self.get_logger().info('Robot Control Node has been started.')

    def range_fl_callback(self, msg):
        # Get the minimum range value from the array
        if msg.ranges:
            min_range = min(msg.ranges)
            self.range_fl = min_range
            self.get_logger().debug(f'Front-left min range: {self.range_fl:.2f} m')
        else:
            self.range_fl = None
            self.get_logger().debug('Front-left range data is empty.')

    def range_fr_callback(self, msg):
        # Get the minimum range value from the array
        if msg.ranges:
            min_range = min(msg.ranges)
            self.range_fr = min_range
            self.get_logger().debug(f'Front-right min range: {self.range_fr:.2f} m')
        else:
            self.range_fr = None
            self.get_logger().debug('Front-right range data is empty.')

    def check_for_obstacles(self):
        threshold_distance = 0.5  # 500 mm
        obstacle_detected = False

        # Check front-left sensor
        if self.range_fl is not None:
            if self.range_fl < threshold_distance:
                obstacle_detected = True
                self.get_logger().debug(f'Obstacle detected by front-left sensor: {self.range_fl:.2f} m')

        # Check front-right sensor
        if self.range_fr is not None:
            if self.range_fr < threshold_distance:
                obstacle_detected = True
                self.get_logger().debug(f'Obstacle detected by front-right sensor: {self.range_fr:.2f} m')

        return obstacle_detected

    def publish_text_to_speech(self, message):
        msg = String()
        msg.data = message
        self.tts_publisher.publish(msg)
        self.get_logger().info(f'Published to /text_to_speech: "{message}"')

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

    def color_detected_callback(self, msg):
        color = msg.data.lower().strip()  # Ensure consistency in color format
        if color in ['red', 'blue', 'green']:
                self.get_logger().info(f'{color.capitalize()} ball detected! Stopping the robot.')
                # Publish announcement to /text_to_speech
                announcement = f'I have found the {color} ball.'
                self.publish_text_to_speech(announcement)
        else:
            self.get_logger().warn(f'Unknown color detected: {color}')
    
    def object_detected_callback(self, msg):
        if msg.data:
            if not self.object_detected:
                self.object_detected = True
                self.get_logger().info('Object detected! Stopping the robot.')
                self.stop_robot()
                # Publish message to /text_to_speech
        else:
            self.object_detected = False


    def handle_movement_command(self, command):
        twist = Twist()

        if command in ['move forward', 'move_forward']:
            twist.linear.x = self.roaming_speed
            twist.angular.z = 0.0
            self.get_logger().info('Executing: Move Forward')
        # Uncomment the following if you wish to allow backward movement in manual mode
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
        if self.mode != 'roaming' or self.object_detected:
            return  # Do nothing if not in roaming mode or object is detected

        current_time = self.get_clock().now()

        # Check for obstacles using sensor data
        obstacle_detected = self.check_for_obstacles()

        twist = Twist()

        if obstacle_detected:
            self.get_logger().info('Obstacle detected. Stopping and deciding turning direction based on sensor readings.')
            twist.linear.x = 0.0  # Stop forward movement

            # Determine turning direction based on sensor readings
            if self.range_fl is not None and self.range_fr is not None:
                if self.range_fl > self.range_fr:
                    # More space on the left, turn left
                    self.turn_direction = self.turn_speed
                    self.get_logger().info('More space on the left. Turning left.')
                else:
                    # More space on the right, turn right
                    self.turn_direction = -self.turn_speed
                    self.get_logger().info('More space on the right. Turning right.')
            elif self.range_fl is not None:
                # Only front-left sensor available, turn left
                self.turn_direction = self.turn_speed
                self.get_logger().info('Only front-left sensor available. Turning left.')
            elif self.range_fr is not None:
                # Only front-right sensor available, turn right
                self.turn_direction = -self.turn_speed
                self.get_logger().info('Only front-right sensor available. Turning right.')
            else:
                # No sensor data available, default to turning left
                self.turn_direction = self.turn_speed
                self.get_logger().info('No sensor data available. Defaulting to turning left.')

            twist.angular.z = self.turn_direction  # Set angular velocity for turning

            # Set turning duration to ensure the robot turns sufficiently
            turn_duration = self.roaming_duration  # Duration in seconds
            self.movement_end_time = current_time + rclpy.duration.Duration(seconds=turn_duration)
            self.obstacle_detected = True  # Update obstacle detection status

        elif self.obstacle_detected and self.movement_end_time is not None and current_time < self.movement_end_time:
            # Continue turning if within the turning duration
            twist.linear.x = 0.0  # Continue stopping forward movement
            twist.angular.z = self.turn_direction  # Continue turning
            self.get_logger().info(f'Continuing to turn: angular.z={twist.angular.z:.2f}')

        else:
            # If no obstacles are detected and not currently turning
            twist.linear.x = self.roaming_speed  # Move forward
            twist.angular.z = 0.0  # No turning
            self.obstacle_detected = False  # Reset obstacle detection status
            self.movement_end_time = None  # Reset movement end time
            self.get_logger().info('No obstacles detected. Moving forward.')

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
