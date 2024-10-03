#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import math
from enum import Enum
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class Mode(Enum):
    IDLE = 0
    AUTOMATIC_MAPPING = 1
    SET_GOAL = 2


class State(Enum):
    NONE = 0
    MOVING_TO_GOAL = 1
    BOUNDARY_FOLLOWING = 2


class Bug2NavigatorNode(Node):
    def __init__(self):
        super().__init__('bug2_navigator_node')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/bug2_markers', 10)

        # Subscribers
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.range_fl_subscription = self.create_subscription(
            LaserScan,
            '/range/fl',
            self.range_fl_callback,
            10)
        self.range_fr_subscription = self.create_subscription(
            LaserScan,
            '/range/fr',
            self.range_fr_callback,
            10)
        self.command_subscription = self.create_subscription(
            String,
            '/roaming_command',
            self.roaming_command_callback,
            10)

        # Initialize sensor data
        self.range_fl = None
        self.range_fr = None
        self.occupancy_grid = None

        # Initialize SLAM pose tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot's goal position (can be set via parameters or commands)
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = 5.0  # Example coordinates
        self.goal_pose.pose.position.y = 5.0
        self.goal_pose.pose.orientation.w = 1.0

        # Current mode and state
        self.mode = Mode.IDLE  # Initialize to IDLE
        self.state = State.NONE  # Initialize to no specific state

        # Timer for behavior execution
        self.behavior_timer = self.create_timer(0.1, self.behavior_execution)  # 0.1-second interval

        # Parameters
        self.obstacle_threshold = 0.5  # meters
        self.m_line_threshold = 0.2    # meters to consider rejoining the M-line
        self.goal_reached_threshold = 0.3  # meters to consider goal reached

        # Bug2 Parameters
        self.boundary_following_speed = 0.2  # meters per second
        self.boundary_turn_speed = 0.3       # radians per second

        # Automatic Mapping Parameters
        self.mapping_linear_speed = 0.2      # meters per second
        self.mapping_angular_speed = 0.3     # radians per second
        self.mapping_turn_duration = 2.0     # seconds to turn during exploration
        self.last_turn_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.get_logger().info('Bug2 Navigator Node has been initialized and is in IDLE state.')

    # Callback to receive the occupancy grid map from SLAM
    def map_callback(self, msg):
        self.occupancy_grid = msg
        self.get_logger().debug('Received updated map from SLAM.')

    # Callback to receive front-left LiDAR data
    def range_fl_callback(self, msg):
        if msg.ranges:
            self.range_fl = min(msg.ranges)
            self.get_logger().debug(f'Front-left min range: {self.range_fl:.2f} m')
        else:
            self.range_fl = None
            self.get_logger().debug('Front-left range data is empty.')

    # Callback to receive front-right LiDAR data
    def range_fr_callback(self, msg):
        if msg.ranges:
            self.range_fr = min(msg.ranges)
            self.get_logger().debug(f'Front-right min range: {self.range_fr:.2f} m')
        else:
            self.range_fr = None
            self.get_logger().debug('Front-right range data is empty.')

    # Callback to receive roaming commands
    def roaming_command_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received roaming command: "{command}"')

        if command == 'start_bug2':
            self.enter_bug2_mode()
        elif command == 'start_mapping':
            self.enter_mapping_mode()
        elif command in ['stop_bug2', 'stop_mapping', 'stop']:
            self.exit_current_mode()
        elif command.startswith('set_goal'):
            # Example command format: "set_goal x y"
            try:
                parts = command.split()
                if len(parts) == 3:
                    x = float(parts[1])
                    y = float(parts[2])
                    self.set_goal(x, y)
                    self.get_logger().info(f'Goal set to ({x}, {y}).')
                else:
                    self.get_logger().warn('Invalid set_goal command format. Use: set_goal x y')
            except ValueError:
                self.get_logger().warn('Invalid coordinates for set_goal command.')
        else:
            self.get_logger().warn(f'Unknown roaming command received: "{command}"')

    # Method to set a new goal
    def set_goal(self, x, y):
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.get_logger().info(f'New goal position set to x: {x}, y: {y}.')
        if self.mode == Mode.SET_GOAL:
            # Publish a marker for visualization
            self.publish_marker(
                position=(x, y),
                marker_id=0,
                marker_type=Marker.SPHERE,
                color=(0.0, 1.0, 0.0))

    # Method to enter Bug2 (Set Goal) mode
    def enter_bug2_mode(self):
        if self.mode != Mode.SET_GOAL:
            self.mode = Mode.SET_GOAL
            self.state = State.MOVING_TO_GOAL
            self.get_logger().info('Entering Set Goal Mode: MOVING_TO_GOAL.')
            # Optionally, publish a marker for the goal
            self.publish_marker(
                position=(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y),
                marker_id=0,
                marker_type=Marker.SPHERE,
                color=(0.0, 1.0, 0.0))
        else:
            self.get_logger().info('Set Goal Mode is already active.')

    # Method to enter Automatic Mapping mode
    def enter_mapping_mode(self):
        if self.mode != Mode.AUTOMATIC_MAPPING:
            self.mode = Mode.AUTOMATIC_MAPPING
            self.state = State.NONE
            self.get_logger().info('Entering Automatic Mapping Mode.')
        else:
            self.get_logger().info('Automatic Mapping Mode is already active.')

    # Method to exit the current mode and transition to IDLE
    def exit_current_mode(self):
        if self.mode != Mode.IDLE:
            self.mode = Mode.IDLE
            self.state = State.NONE
            self.get_logger().info('Exiting current mode. Transitioning to IDLE.')
            self.stop_robot()
        else:
            self.get_logger().info('Robot is already in IDLE state.')

    # Unified behavior execution timer callback
    def behavior_execution(self):
        if self.mode == Mode.SET_GOAL:
            if self.state == State.MOVING_TO_GOAL:
                self.move_towards_goal()
            elif self.state == State.BOUNDARY_FOLLOWING:
                self.follow_boundary()
        elif self.mode == Mode.AUTOMATIC_MAPPING:
            self.automatic_mapping_behavior()
        elif self.mode == Mode.IDLE:
            # Ensure the robot remains stopped in IDLE mode
            self.stop_robot()

    # Method to move towards the goal along the M-line
    def move_towards_goal(self):
        if self.is_goal_reached():
            self.get_logger().info('Goal reached successfully.')
            self.mode = Mode.IDLE
            self.state = State.NONE
            self.stop_robot()
            return

        if self.is_obstacle_ahead():
            self.get_logger().info('Obstacle detected! Switching to Boundary Following State.')
            self.state = State.BOUNDARY_FOLLOWING
            return

        try:
            # Get robot's current pose
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            # Calculate the angle to the goal
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y
            angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)

            # Simple proportional controller for angular velocity
            current_yaw = self.get_yaw_from_transform(transform)
            angle_diff = self.normalize_angle(angle_to_goal - current_yaw)

            twist = Twist()
            twist.linear.x = 0.3  # Forward speed
            twist.angular.z = 0.5 * angle_diff
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().debug('Set Goal Mode: Moving towards goal.')
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get robot pose: {e}')
            self.stop_robot()

    # Method to follow the boundary of an obstacle
    def follow_boundary(self):
        twist = Twist()
        twist.linear.x = self.boundary_following_speed
        twist.angular.z = self.boundary_turn_speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().debug('Set Goal Mode: Following obstacle boundary.')

        # Check if the robot has rejoined the M-line
        if self.has_rejoined_m_line():
            self.get_logger().info('Rejoined M-line. Switching back to MOVING_TO_GOAL State.')
            self.state = State.MOVING_TO_GOAL
            self.stop_robot()

    # Method to perform automatic mapping behavior
    def automatic_mapping_behavior(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        time_since_last_turn = current_time - self.last_turn_time

        if self.is_obstacle_ahead():
            # If obstacle detected, perform a turn
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.mapping_angular_speed
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().debug('Automatic Mapping Mode: Obstacle detected, turning.')
            # Update the last turn time to prevent continuous turning
            self.last_turn_time = current_time
        else:
            # Continue moving forward
            twist = Twist()
            twist.linear.x = self.mapping_linear_speed
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().debug('Automatic Mapping Mode: Moving forward.')

        # Optional: Implement more sophisticated exploration strategies (e.g., random walks, frontier-based exploration)

    # Method to check if an obstacle is ahead
    def is_obstacle_ahead(self):
        obstacle = False
        if self.range_fl is not None and self.range_fl < self.obstacle_threshold:
            self.get_logger().debug('Obstacle detected on front-left.')
            obstacle = True
        if self.range_fr is not None and self.range_fr < self.obstacle_threshold:
            self.get_logger().debug('Obstacle detected on front-right.')
            obstacle = True
        return obstacle

    # Method to check if the goal is reached
    def is_goal_reached(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            distance = math.hypot(robot_x - self.goal_pose.pose.position.x,
                                  robot_y - self.goal_pose.pose.position.y)
            self.get_logger().debug(f'Distance to goal: {distance:.2f} m')
            if distance < self.goal_reached_threshold:
                return True
            return False
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('Could not get robot pose.')
            return False

    # Method to check if the robot has rejoined the M-line
    def has_rejoined_m_line(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            # Define M-line as the straight line from start (0,0) to goal
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y

            if goal_x == 0 and goal_y == 0:
                return False  # Avoid division by zero

            # Calculate distance from current position to M-line
            distance = abs(goal_y * robot_x - goal_x * robot_y) / math.hypot(goal_x, goal_y)
            self.get_logger().debug(f'Distance to M-line: {distance:.2f} m')
            if distance < self.m_line_threshold:
                return True
            return False
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('Could not get robot pose.')
            return False

    # Method to stop the robot by sending zero velocities
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Robot stopped.')

    # Utility method to normalize angles between -pi and pi
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # Utility method to extract yaw from transform
    def get_yaw_from_transform(self, transform):
        orientation = transform.transform.rotation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    # Optional: Method to publish markers for visualization
    def publish_marker(self, position, marker_id, marker_type=Marker.SPHERE, color=(1.0, 0.0, 0.0)):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'bug2_markers'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0  # Alpha
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        self.marker_publisher.publish(MarkerArray(markers=[marker]))
        self.get_logger().debug(f'Published marker {marker_id} at position {position}.')

    # Additional utility methods can be added as needed


def main(args=None):
    rclpy.init(args=args)
    node = Bug2NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Bug2 Navigator Node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
