#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
import action_msgs.msg
import rclpy.action
from collections import deque


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/frontiers_markers', 10)

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
        self.obstacle_detected = False
        self.occupancy_grid = None

        # Initialize SLAM pose tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action Client for Navigation
        self.nav_action_client = rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for NavigateToPose action server...')
        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available! Exiting.')
            rclpy.shutdown()
            return
        self.get_logger().info('NavigateToPose action server available.')

        # Modes: 'idle', 'frontier', 'left_right'
        self.mode = 'idle'

        # Timers
        self.exploration_timer = self.create_timer(0.1, self.exploration_behavior)  # 0.1-second interval

        # Frontier Tracking
        self.visited_frontiers = set()  # Set of tuples representing visited frontier clusters

        # Navigation Flags
        self.is_navigating = False

        # Frontier Clustering Parameters
        self.min_frontier_distance = 2.0  # meters
        self.cluster_eps = 0.7  # meters
        self.cluster_min_samples = 5  # Minimum number of frontiers to form a cluster

        # Left-to-Right Navigation Parameters
        self.lr_linear_speed = 0.5          # meters per second
        self.lr_angular_speed = 0.5         # radians per second
        self.lr_move_distance = 2.0         # meters to move forward before shifting
        self.lr_shift_distance = 0.5        # meters to shift left or right
        self.lr_direction = 'left'          # current shift direction
        self.lr_state = 'move_forward'      # States: 'move_forward', 'shift', 'wait_shift', 'complete_shift'

        self.get_logger().info('Robot Control Node has been started.')

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

        if command == 'start_frontier':
            self.enter_frontier_mode()
        elif command == 'stop_frontier':
            self.exit_frontier_mode()
        elif command == 'start_left_right':
            self.enter_left_right_mode()
        elif command == 'stop_left_right':
            self.exit_left_right_mode()
        elif command == 'stop':
            # Generic stop command to stop all modes
            self.exit_frontier_mode()
            self.exit_left_right_mode()
        else:
            self.get_logger().warn(f'Unknown roaming command received: "{command}"')

    # Method to enter frontier mode
    def enter_frontier_mode(self):
        if self.mode != 'frontier':
            self.mode = 'frontier'
            self.get_logger().info('Entering Frontier Mode.')
            # Stop other modes if active
            self.exit_left_right_mode()

    # Method to exit frontier mode
    def exit_frontier_mode(self):
        if self.mode == 'frontier':
            self.mode = 'idle'
            self.get_logger().info('Exiting Frontier Mode.')
            self.stop_robot()

    # Method to enter left-to-right navigation mode
    def enter_left_right_mode(self):
        if self.mode != 'left_right':
            self.mode = 'left_right'
            self.lr_state = 'move_forward'  # Initialize state
            self.get_logger().info('Entering Left-to-Right Navigation Mode.')
            # Stop other modes if active
            self.exit_frontier_mode()

    # Method to exit left-to-right navigation mode
    def exit_left_right_mode(self):
        if self.mode == 'left_right':
            self.mode = 'idle'
            self.get_logger().info('Exiting Left-to-Right Navigation Mode.')
            self.stop_robot()

    # Unified exploration behavior timer callback
    def exploration_behavior(self):
        # First, check for obstacles globally
        obstacle_detected = self.check_for_obstacles()

        if obstacle_detected:
            self.handle_obstacle()
            return  # Skip current mode's behavior and handle obstacle

        # Proceed with mode-specific behavior
        if self.mode == 'frontier':
            self.frontier_behavior()
        elif self.mode == 'left_right':
            self.left_right_behavior()
        # Add more modes here if needed

    # Frontier mode behavior
    def frontier_behavior(self):
        if self.occupancy_grid is None:
            self.get_logger().warn('No occupancy grid received yet.')
            return

        frontiers = self.find_frontiers()
        selected_frontier = self.select_frontier(frontiers)

        if selected_frontier:
            self.navigate_to_frontier(selected_frontier)
        else:
            self.get_logger().info('No frontiers found. Rotating to search for frontiers.')
            twist = Twist()
            twist.angular.z = 0.7  # Rotate in place
            self.cmd_vel_publisher.publish(twist)

    # Left-to-Right navigation behavior
    def left_right_behavior(self):
        twist = Twist()

        if self.lr_state == 'move_forward':
            twist.linear.x = self.lr_linear_speed
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Left-Right Mode: Moving Forward.')
            # Shift after moving the required distance (time-based for simplicity)
            move_duration = self.lr_move_distance / self.lr_linear_speed
            self.create_timer(move_duration, self.lr_shift_direction)
            self.lr_state = 'wait_shift'
        elif self.lr_state == 'wait_shift':
            # Waiting for shift timer to complete
            pass
        elif self.lr_state == 'shift':
            if self.lr_direction == 'left':
                twist.linear.x = 0.0
                twist.angular.z = self.lr_angular_speed
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info('Left-Right Mode: Shifting Left.')
                shift_duration = (math.pi / 2) / self.lr_angular_speed  # 90 degrees
                self.create_timer(shift_duration, self.lr_complete_shift)
                self.lr_state = 'wait_complete_shift'
            elif self.lr_direction == 'right':
                twist.linear.x = 0.0
                twist.angular.z = -self.lr_angular_speed
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info('Left-Right Mode: Shifting Right.')
                shift_duration = (math.pi / 2) / self.lr_angular_speed  # 90 degrees
                self.create_timer(shift_duration, self.lr_complete_shift)
                self.lr_state = 'wait_complete_shift'

    def lr_shift_direction(self):
        # Toggle shift direction
        self.lr_direction = 'right' if self.lr_direction == 'left' else 'left'
        self.lr_state = 'shift'

    def lr_complete_shift(self):
        # Complete the shift by turning back to original direction
        twist = Twist()
        if self.lr_direction == 'left':
            twist.angular.z = -self.lr_angular_speed
            self.get_logger().info('Left-Right Mode: Completing Left Shift.')
        else:
            twist.angular.z = self.lr_angular_speed
            self.get_logger().info('Left-Right Mode: Completing Right Shift.')
        self.cmd_vel_publisher.publish(twist)
        self.lr_state = 'move_forward'

    # Method to find frontiers in the occupancy grid with greedy clustering
    def find_frontiers(self):
        frontiers = []
        data = self.occupancy_grid.data
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        resolution = self.occupancy_grid.info.resolution
        origin = self.occupancy_grid.info.origin

        # Convert 1D data to 2D grid
        grid = np.array(data).reshape((height, width))

        # Iterate through the grid to find frontiers
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if grid[y][x] == 0:  # Free cell
                    # Check 8-connected neighbors for unknown cells (-1)
                    neighbors = grid[y - 1:y + 2, x - 1:x + 2].flatten()
                    if -1 in neighbors:
                        # Convert grid cell to world coordinates
                        world_x = origin.position.x + x * resolution
                        world_y = origin.position.y + y * resolution
                        frontiers.append([world_x, world_y])

        self.get_logger().info(f'Found {len(frontiers)} frontiers before clustering.')

        # Perform greedy clustering
        clustered_frontiers = self.cluster_frontiers(frontiers, eps=self.cluster_eps, min_samples=self.cluster_min_samples)
        self.get_logger().info(f'Found {len(clustered_frontiers)} clustered frontiers.')

        # Optional: Visualize frontiers in RViz2
        self.publish_frontier_markers(clustered_frontiers)

        return clustered_frontiers

    # Greedy clustering algorithm
    def cluster_frontiers(self, frontiers, eps=0.7, min_samples=5):
        clusters = []
        for frontier in frontiers:
            assigned = False
            for cluster in clusters:
                centroid = cluster['centroid']
                distance = math.hypot(frontier[0] - centroid[0], frontier[1] - centroid[1])
                if distance <= eps:
                    # Assign to this cluster
                    cluster['members'].append(frontier)
                    # Update centroid
                    cluster['centroid'] = tuple(np.mean(cluster['members'], axis=0))
                    assigned = True
                    break
            if not assigned:
                # Create new cluster
                clusters.append({'centroid': tuple(frontier), 'members': [frontier]})

        # Filter out clusters with insufficient members
        valid_clusters = [cluster for cluster in clusters if len(cluster['members']) >= self.cluster_min_samples]
        clustered_centroids = [cluster['centroid'] for cluster in valid_clusters]

        return clustered_centroids

    # Method to publish frontier markers for visualization in RViz2
    def publish_frontier_markers(self, frontiers):
        marker_array = MarkerArray()
        for idx, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = frontier[0]
            marker.pose.position.y = frontier[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0  # Alpha
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)
        self.get_logger().debug('Published frontier markers to RViz2.')

    # Method to select the nearest unvisited frontier
    def select_frontier(self, frontiers):
        if not frontiers:
            return None

        # Get robot's current pose
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('Could not get robot pose.')
            return None

        # Find the nearest unvisited frontier
        min_distance = float('inf')
        selected_frontier = None
        for frontier in frontiers:
            distance = math.hypot(frontier[0] - robot_x, frontier[1] - robot_y)
            if distance < self.min_frontier_distance:
                continue  # Skip frontiers too close
            if self.is_frontier_visited(frontier):
                continue  # Skip already visited frontiers
            if distance < min_distance:
                min_distance = distance
                selected_frontier = frontier

        if selected_frontier:
            self.get_logger().info(f'Selected frontier at ({selected_frontier[0]:.2f}, {selected_frontier[1]:.2f})')
        else:
            self.get_logger().info('No suitable frontier found.')

        return selected_frontier

    # Method to check if a frontier has been visited
    def is_frontier_visited(self, frontier, threshold=1.0):
        frontier_tuple = tuple(frontier)
        for visited in self.visited_frontiers:
            distance = math.hypot(frontier_tuple[0] - visited[0], frontier_tuple[1] - visited[1])
            if distance < threshold:
                return True
        return False

    # Method to navigate to the selected frontier using NavigateToPose action
    def navigate_to_frontier(self, frontier):
        if self.is_navigating:
            self.get_logger().info('Already navigating to a frontier. Skipping new goal.')
            return

        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = frontier[0]
        pose.pose.position.y = frontier[1]
        pose.pose.orientation.w = 1.0  # Facing forward

        goal_msg.pose = pose

        self.get_logger().info(f'Sending navigation goal to ({frontier[0]:.2f}, {frontier[1]:.2f})')

        self.is_navigating = True
        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

        # Mark the frontier as visited to prevent re-selection
        self.visited_frontiers.add(tuple(frontier))

    # Callback for goal response
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected.')
            self.is_navigating = False
            return

        self.get_logger().info('Navigation goal accepted.')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    # Callback to get the result of the navigation goal
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached the frontier successfully.')
        else:
            self.get_logger().info(f'Failed to reach the frontier with status: {status}')
        self.is_navigating = False

    # Callback to receive navigation feedback
    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose.position
        self.get_logger().debug(f'Navigation feedback: x={current_pose.x:.2f}, y={current_pose.y:.2f}')

    # Method to stop the robot by sending zero velocities
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Robot stopped.')

    # Method to check for obstacles using sensor data
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

    # Method to handle obstacle detection globally
    def handle_obstacle(self):
        self.get_logger().info('Obstacle detected! Executing avoidance maneuver.')
        # Stop current movement
        self.stop_robot()

        # Depending on the mode, execute appropriate avoidance strategy
        if self.mode == 'frontier':
            self.avoid_obstacle_frontier()
        elif self.mode == 'left_right':
            self.avoid_obstacle_left_right()
        else:
            self.get_logger().warn('Robot is in idle mode. No avoidance action taken.')

    # Obstacle avoidance for Frontier Mode
    def avoid_obstacle_frontier(self):
        self.get_logger().info('Frontier Mode: Avoiding obstacle by deciding turn direction.')

        # Determine turn direction based on sensor readings
        if self.range_fl is not None and self.range_fr is not None:
            if self.range_fl > self.range_fr:
                # More space on the left
                twist = Twist()
                twist.angular.z = 0.7  # Turn left
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info('Frontier Mode: More space on the left. Turning left.')
            else:
                # More space on the right
                twist = Twist()
                twist.angular.z = -0.7  # Turn right
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info('Frontier Mode: More space on the right. Turning right.')
        elif self.range_fl is not None:
            # Only front-left sensor available, turn left
            twist = Twist()
            twist.angular.z = 0.7
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Frontier Mode: Only front-left sensor available. Turning left.')
        elif self.range_fr is not None:
            # Only front-right sensor available, turn right
            twist = Twist()
            twist.angular.z = -0.7
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Frontier Mode: Only front-right sensor available. Turning right.')
        else:
            # No sensor data available, default to turning left
            twist = Twist()
            twist.angular.z = 0.7
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Frontier Mode: No sensor data available. Defaulting to turning left.')

        # Set a timer to resume normal behavior after avoidance maneuver
        avoidance_duration = 1.0  # seconds
        self.create_timer(avoidance_duration, self.resume_frontier_mode)

    def resume_frontier_mode(self):
        self.get_logger().info('Frontier Mode: Resuming normal navigation after avoidance.')
        # No specific state to reset in frontier mode
        # The timer will allow exploration_behavior to continue

    # Obstacle avoidance for Left-to-Right Mode
    def avoid_obstacle_left_right(self):
        self.get_logger().info('Left-Right Mode: Avoiding obstacle by deciding turn direction.')

        # Determine turn direction based on sensor readings
        if self.range_fl is not None and self.range_fr is not None:
            if self.range_fl > self.range_fr:
                # More space on the left
                twist = Twist()
                twist.angular.z = 0.5  # Turn left
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info('Left-Right Mode: More space on the left. Turning left.')
            else:
                # More space on the right
                twist = Twist()
                twist.angular.z = -0.5  # Turn right
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info('Left-Right Mode: More space on the right. Turning right.')
        elif self.range_fl is not None:
            # Only front-left sensor available, turn left
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Left-Right Mode: Only front-left sensor available. Turning left.')
        elif self.range_fr is not None:
            # Only front-right sensor available, turn right
            twist = Twist()
            twist.angular.z = -0.5
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Left-Right Mode: Only front-right sensor available. Turning right.')
        else:
            # No sensor data available, default to turning left
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Left-Right Mode: No sensor data available. Defaulting to turning left.')

        # Set a timer to resume normal behavior after avoidance maneuver
        avoidance_duration = 1.0  # seconds
        self.create_timer(avoidance_duration, self.resume_left_right_mode)

    def resume_left_right_mode(self):
        self.get_logger().info('Left-Right Mode: Resuming normal navigation after avoidance.')
        self.lr_state = 'move_forward'

    # Method to find frontiers in the occupancy grid with greedy clustering
    def find_frontiers(self):
        frontiers = []
        data = self.occupancy_grid.data
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        resolution = self.occupancy_grid.info.resolution
        origin = self.occupancy_grid.info.origin

        # Convert 1D data to 2D grid
        grid = np.array(data).reshape((height, width))

        # Iterate through the grid to find frontiers
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if grid[y][x] == 0:  # Free cell
                    # Check 8-connected neighbors for unknown cells (-1)
                    neighbors = grid[y - 1:y + 2, x - 1:x + 2].flatten()
                    if -1 in neighbors:
                        # Convert grid cell to world coordinates
                        world_x = origin.position.x + x * resolution
                        world_y = origin.position.y + y * resolution
                        frontiers.append([world_x, world_y])

        self.get_logger().info(f'Found {len(frontiers)} frontiers before clustering.')

        # Perform greedy clustering
        clustered_frontiers = self.cluster_frontiers(frontiers, eps=self.cluster_eps, min_samples=self.cluster_min_samples)
        self.get_logger().info(f'Found {len(clustered_frontiers)} clustered frontiers.')

        # Optional: Visualize frontiers in RViz2
        self.publish_frontier_markers(clustered_frontiers)

        return clustered_frontiers

    # Greedy clustering algorithm
    def cluster_frontiers(self, frontiers, eps=0.7, min_samples=5):
        clusters = []
        for frontier in frontiers:
            assigned = False
            for cluster in clusters:
                centroid = cluster['centroid']
                distance = math.hypot(frontier[0] - centroid[0], frontier[1] - centroid[1])
                if distance <= eps:
                    # Assign to this cluster
                    cluster['members'].append(frontier)
                    # Update centroid
                    cluster['centroid'] = tuple(np.mean(cluster['members'], axis=0))
                    assigned = True
                    break
            if not assigned:
                # Create new cluster
                clusters.append({'centroid': tuple(frontier), 'members': [frontier]})

        # Filter out clusters with insufficient members
        valid_clusters = [cluster for cluster in clusters if len(cluster['members']) >= self.cluster_min_samples]
        clustered_centroids = [cluster['centroid'] for cluster in valid_clusters]

        return clustered_centroids

    # Method to publish frontier markers for visualization in RViz2
    def publish_frontier_markers(self, frontiers):
        marker_array = MarkerArray()
        for idx, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = frontier[0]
            marker.pose.position.y = frontier[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0  # Alpha
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)
        self.get_logger().debug('Published frontier markers to RViz2.')

    # Method to select the nearest unvisited frontier
    def select_frontier(self, frontiers):
        if not frontiers:
            return None

        # Get robot's current pose
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('Could not get robot pose.')
            return None

        # Find the nearest unvisited frontier
        min_distance = float('inf')
        selected_frontier = None
        for frontier in frontiers:
            distance = math.hypot(frontier[0] - robot_x, frontier[1] - robot_y)
            if distance < self.min_frontier_distance:
                continue  # Skip frontiers too close
            if self.is_frontier_visited(frontier):
                continue  # Skip already visited frontiers
            if distance < min_distance:
                min_distance = distance
                selected_frontier = frontier

        if selected_frontier:
            self.get_logger().info(f'Selected frontier at ({selected_frontier[0]:.2f}, {selected_frontier[1]:.2f})')
        else:
            self.get_logger().info('No suitable frontier found.')

        return selected_frontier

    # Method to check if a frontier has been visited
    def is_frontier_visited(self, frontier, threshold=1.0):
        frontier_tuple = tuple(frontier)
        for visited in self.visited_frontiers:
            distance = math.hypot(frontier_tuple[0] - visited[0], frontier_tuple[1] - visited[1])
            if distance < threshold:
                return True
        return False

    # Method to navigate to the selected frontier using NavigateToPose action
    def navigate_to_frontier(self, frontier):
        if self.is_navigating:
            self.get_logger().info('Already navigating to a frontier. Skipping new goal.')
            return

        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = frontier[0]
        pose.pose.position.y = frontier[1]
        pose.pose.orientation.w = 1.0  # Facing forward

        goal_msg.pose = pose

        self.get_logger().info(f'Sending navigation goal to ({frontier[0]:.2f}, {frontier[1]:.2f})')

        self.is_navigating = True
        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

        # Mark the frontier as visited to prevent re-selection
        self.visited_frontiers.add(tuple(frontier))

    # Callback for goal response
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected.')
            self.is_navigating = False
            return

        self.get_logger().info('Navigation goal accepted.')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    # Callback to get the result of the navigation goal
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached the frontier successfully.')
        else:
            self.get_logger().info(f'Failed to reach the frontier with status: {status}')
        self.is_navigating = False

    # Callback to receive navigation feedback
    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose.position
        self.get_logger().debug(f'Navigation feedback: x={current_pose.x:.2f}, y={current_pose.y:.2f}')

    # Method to stop the robot by sending zero velocities
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Robot stopped.')

    # Method to check for obstacles using sensor data
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

    # Method to handle obstacle detection globally
    def handle_obstacle(self):
        self.get_logger().info('Obstacle detected! Executing avoidance maneuver.')
        # Stop current movement
        self.stop_robot()

        # Depending on the mode, execute appropriate avoidance strategy
        if self.mode == 'frontier':
            self.avoid_obstacle_frontier()
        elif self.mode == 'left_right':
            self.avoid_obstacle_left_right()
        else:
            self.get_logger().warn('Robot is in idle mode. No avoidance action taken.')

    # Obstacle avoidance for Frontier Mode
    def avoid_obstacle_frontier(self):
        self.get_logger().info('Frontier Mode: Avoiding obstacle by deciding turn direction.')

        # Determine turn direction based on sensor data
        if self.range_fl is not None and self.range_fr is not None:
            if self.range_fl > self.range_fr:
                # More space on the left
                twist = Twist()
                twist.angular.z = 0.7  # Turn left
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info('Frontier Mode: More space on the left. Turning left.')
            else:
                # More space on the right
                twist = Twist()
                twist.angular.z = -0.7  # Turn right
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info('Frontier Mode: More space on the right. Turning right.')
        elif self.range_fl is not None:
            # Only front-left sensor available, turn left
            twist = Twist()
            twist.angular.z = 0.7
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Frontier Mode: Only front-left sensor available. Turning left.')
        elif self.range_fr is not None:
            # Only front-right sensor available, turn right
            twist = Twist()
            twist.angular.z = -0.7
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Frontier Mode: Only front-right sensor available. Turning right.')
        else:
            # No sensor data available, default to turning left
            twist = Twist()
            twist.angular.z = 0.7
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Frontier Mode: No sensor data available. Defaulting to turning left.')

        # Set a timer to resume normal behavior after avoidance maneuver
        avoidance_duration = 1.0  # seconds
        self.create_timer(avoidance_duration, self.resume_frontier_mode)

    def resume_frontier_mode(self):
        self.get_logger().info('Frontier Mode: Resuming normal navigation after avoidance.')
        # No specific state to reset in frontier mode
        # The timer will allow exploration_behavior to continue

    # Obstacle avoidance for Left-to-Right Mode
    def avoid_obstacle_left_right(self):
        self.get_logger().info('Left-Right Mode: Avoiding obstacle by deciding turn direction.')

        # Determine turn direction based on sensor data
        if self.range_fl is not None and self.range_fr is not None:
            if self.range_fl > self.range_fr:
                # More space on the left
                twist = Twist()
                twist.angular.z = 0.5  # Turn left
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info('Left-Right Mode: More space on the left. Turning left.')
            else:
                # More space on the right
                twist = Twist()
                twist.angular.z = -0.5  # Turn right
                self.cmd_vel_publisher.publish(twist)
                self.get_logger().info('Left-Right Mode: More space on the right. Turning right.')
        elif self.range_fl is not None:
            # Only front-left sensor available, turn left
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Left-Right Mode: Only front-left sensor available. Turning left.')
        elif self.range_fr is not None:
            # Only front-right sensor available, turn right
            twist = Twist()
            twist.angular.z = -0.5
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Left-Right Mode: Only front-right sensor available. Turning right.')
        else:
            # No sensor data available, default to turning left
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info('Left-Right Mode: No sensor data available. Defaulting to turning left.')

        # Set a timer to resume normal behavior after avoidance maneuver
        avoidance_duration = 1.0  # seconds
        self.create_timer(avoidance_duration, self.resume_left_right_mode)

    def resume_left_right_mode(self):
        self.get_logger().info('Left-Right Mode: Resuming normal navigation after avoidance.')
        self.lr_state = 'move_forward'

    # Method to handle all obstacle-related maneuvers
    # (Deprecated: Since we have separate methods now)
    # def handle_obstacle_in_modes(self):
    #     ...

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
