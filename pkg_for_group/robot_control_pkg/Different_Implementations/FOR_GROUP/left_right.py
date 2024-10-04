import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  # Import String message type for command subscription


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

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

        # Subscribe to roaming command
        self.roaming_command_subscription = self.create_subscription(
            String,
            '/roaming_command',
            self.roaming_command_callback,
            10)
        self.roaming_command_subscription  # Prevent unused variable warning

        # Variables to store range data
        self.range_fl = None
        self.range_fr = None

        # Initialize obstacle_detected based on range sensors
        self.obstacle_detected = False

        # Roaming parameters
        self.roaming_speed = 0.5       # Speed when moving forward (meters per second)
        self.turn_speed = 0.7          # Angular speed when turning (radians per second)
        self.roaming_duration = 0.5    # Duration to turn (seconds)
        self.current_movement = None
        self.movement_end_time = None

        # Timer for roaming behavior (0.1-second interval)
        self.roaming_timer = self.create_timer(0.1, self.roaming_behavior)

        # Current mode ('idle', 'roaming', or 'active')
        self.mode = 'idle'

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

    def roaming_command_callback(self, msg):
        command = msg.data.lower()
        if command == 'start':
            self.enter_roaming_mode()
        elif command == 'stop':
            self.exit_roaming_mode()
        else:
            self.get_logger().warn(f'Unknown command received: {msg.data}')

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

    def enter_roaming_mode(self):
        if self.mode != 'roaming':
            self.mode = 'roaming'
            self.current_movement = None
            self.movement_end_time = None
            self.get_logger().info('Entering Roaming Mode.')

    def exit_roaming_mode(self):
        if self.mode == 'roaming':
            self.mode = 'idle'  # Corrected from 'idol' to 'idle'
            self.current_movement = None
            self.movement_end_time = None
            self.get_logger().info('Exiting Roaming Mode.')
            self.stop_robot()

    def roaming_behavior(self):
        if self.mode != 'roaming' or self.obstacle_detected:
            # Only proceed if in roaming mode and no obstacle detected
            if self.mode != 'roaming':
                return  # Do nothing if not in roaming mode
            # If obstacle is detected, handle it below
            # Continue processing
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

