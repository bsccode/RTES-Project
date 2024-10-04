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
from enum import Enum


class Mode(Enum):
    IDLE = 0
    FRONTIER = 1
    LEFT_RIGHT = 2
    BUG2 = 3  # Added Bug2 mode


class Bug2State(Enum):
    NONE = 0
    MOVING_TO_GOAL = 1
    BOUNDARY_FOLLOWING = 2


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.frontier_marker_publisher = self.create_publisher(MarkerArray, '/frontiers_markers', 10)
        self.bug2_marker_publisher = self.create_publisher(MarkerArray, '/bug2_markers', 10)

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
        
        # Flag to track if obstacle avoidance has been logged
        self.has_logged_obstacle = False 



        # Initialize sensor data
        self.range_fl = None
        self.range_fr = None
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

        # Modes
        self.mode = Mode.IDLE
        self.bug2_state = Bug2State.NONE

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

        # Bug2 Parameters
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = 5.0  # Default goal position
        self.goal_pose.pose.position.y = 5.0
        self.goal_pose.pose.orientation.w = 1.0
        self.obstacle_threshold = 0.5  # meters
        self.m_line_threshold = 0.2    # meters to consider rejoining the M-line
        self.goal_reached_threshold = 0.3  # meters to consider goal reached
        self.bug2_state = Bug2State.NONE
        self.boundary_following_speed = 0.2  # meters per second
        self.boundary_turn_speed = 0.3       # radians per second

        self.get_logger().info('Robot Control Node has been started with Bug2 capabilities.')

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
        elif command == 'start_bug2':
            self.enter_bug2_mode()
        elif command == 'stop_bug2':
            self.exit_bug2_mode()
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
        elif command == 'stop':
            # Generic stop command to stop all modes
            self.exit_frontier_mode()
            self.exit_left_right_mode()
            self.exit_bug2_mode()
        else:
            self.get_logger().warn(f'Unknown roaming command received: "{command}"')

    # Method to enter frontier mode
    def enter_frontier_mode(self):
        if self.mode != Mode.FRONTIER:
            self.mode = Mode.FRONTIER
            self.get_logger().info('Entering Frontier Mode.')
            # Stop other modes if active
            self.exit_left_right_mode()
            self.exit_bug2_mode()
        else:
            self.get_logger().info('Frontier Mode is already active.')

    # Method to exit frontier mode
    def exit_frontier_mode(self):
        if self.mode == Mode.FRONTIER:
            self.mode = Mode.IDLE
            self.get_logger().info('Exiting Frontier Mode.')
            self.stop_robot()

    # Method to enter left-to-right navigation mode
    def enter_left_right_mode(self):
        if self.mode != Mode.LEFT_RIGHT:
            self.mode = Mode.LEFT_RIGHT
            self.lr_state = 'move_forward'  # Initialize state
            self.get_logger().info('Entering Left-to-Right Navigation Mode.')
            # Stop other modes if active
            self.exit_frontier_mode()
            self.exit_bug2_mode()
        else:
            self.get_logger().info('Left-Right Navigation Mode is already active.')

    # Method to exit left-to-right navigation mode
    def exit_left_right_mode(self):
        if self.mode == Mode.LEFT_RIGHT:
            self.mode = Mode.IDLE
            self.get_logger().info('Exiting Left-to-Right Navigation Mode.')
            self.stop_robot()

    # Method to enter Bug2 navigation mode
    def enter_bug2_mode(self):
        if self.mode != Mode.BUG2:
            self.mode = Mode.BUG2
            self.bug2_state = Bug2State.MOVING_TO_GOAL
            self.get_logger().info('Entering Bug2 Navigation Mode.')
            # Optionally, publish a marker for the goal
            self.publish_goal_marker()
            # Stop other modes if active
            self.exit_frontier_mode()
            self.exit_left_right_mode()
        else:
            self.get_logger().info('Bug2 Navigation Mode is already active.')

    # Method to exit Bug2 navigation mode
    def exit_bug2_mode(self):
        if self.mode == Mode.BUG2:
            self.mode = Mode.IDLE
            self.bug2_state = Bug2State.NONE
            self.get_logger().info('Exiting Bug2 Navigation Mode.')
            self.stop_robot()
        else:
            self.get_logger().info('Bug2 Navigation Mode is not active.')

    # Method to set a new goal
    def set_goal(self, x, y):
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.get_logger().info(f'New goal position set to x: {x}, y: {y}.')
        if self.mode == Mode.BUG2:
            self.publish_goal_marker()

    # Method to publish goal marker for Bug2 mode
    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'bug2_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_pose.pose.position.x
        marker.pose.position.y = self.goal_pose.pose.position.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0  # Alpha
        marker.color.r = 0.0
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # Forever
        marker_array = MarkerArray(markers=[marker])
        self.bug2_marker_publisher.publish(marker_array)
        self.get_logger().debug('Published Bug2 goal marker to RViz2.')

    # Define is_obstacle_ahead method
    def is_obstacle_ahead(self):
        return self.check_for_obstacles()

    # Unified exploration behavior timer callback
    def exploration_behavior(self):
        # First, check for obstacles globally
        obstacle_detected = self.check_for_obstacles()

        if obstacle_detected:
            self.handle_obstacle()
            return  # Skip current mode's behavior and handle obstacle

        # Proceed with mode-specific behavior
        if self.mode == Mode.FRONTIER:
            self.frontier_behavior()
        elif self.mode == Mode.LEFT_RIGHT:
            self.left_right_behavior()
        elif self.mode == Mode.BUG2:
            self.bug2_behavior()
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

        # Publish frontier markers for visualization in RViz2
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
        self.frontier_marker_publisher.publish(marker_array)
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
        # result = future.result().result
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
        elif self.lr_state == 'wait_complete_shift':
            # Waiting for shift completion
            pass

    def lr_shift_direction(self):
        # Toggle shift direction
        self.lr_direction = 'right' if self.lr_direction == 'left' else 'left'
        self.lr_state = 'shift'

    def lr_complete_shift(self):
        if self.mode != Mode.LEFT_RIGHT:
            if not hasattr(self, 'logged_shift_skip') or not self.logged_shift_skip:
                self.get_logger().info('Left-Right Mode: Shift completion skipped because mode is no longer active.')
                self.logged_shift_skip = True
            return  # Exit if the mode is no longer LEFT_RIGHT

        self.logged_shift_skip = False  # Reset the flag when in the correct mode
        twist = Twist()
        if self.lr_direction == 'left':
            twist.angular.z = -self.lr_angular_speed
        else:
            twist.angular.z = self.lr_angular_speed

        self.cmd_vel_publisher.publish(twist)
        shift_duration = (math.pi / 2) / self.lr_angular_speed  # 90 degrees
        self.create_timer(shift_duration, self.lr_resume_forward)
        self.lr_state = 'wait_shift_completion'

    def lr_resume_forward(self):
        if self.mode != Mode.LEFT_RIGHT:
            if not hasattr(self, 'logged_forward_skip') or not self.logged_forward_skip:
                self.get_logger().info('Left-Right Mode: Forward movement skipped because mode is no longer active.')
                self.logged_forward_skip = True
            return  # Exit if the mode is no longer LEFT_RIGHT

        self.logged_forward_skip = False  # Reset the flag when in the correct mode
        self.lr_state = 'move_forward'

    # Obstacle avoidance for Left-to-Right Mode
    def avoid_obstacle_left_right(self):
        # Check if we are still in LEFT_RIGHT mode before proceeding
        if self.mode != Mode.LEFT_RIGHT:
            return  # Exit if the mode is no longer LEFT_RIGHT

        # Determine turn direction based on sensor readings
        twist = Twist()
        if self.range_fl is not None and self.range_fr is not None:
            if self.range_fl > self.range_fr:
                twist.angular.z = 0.5  # Turn left
            else:
                twist.angular.z = -0.5  # Turn right
        elif self.range_fl is not None:
            twist.angular.z = 0.5  # Turn left
        elif self.range_fr is not None:
            twist.angular.z = -0.5  # Turn right
        else:
            twist.angular.z = 0.5  # Default to turning left

        self.cmd_vel_publisher.publish(twist)

        # Set a timer to resume normal behavior after avoidance maneuver
        avoidance_duration = 1.0  # seconds

        def resume_left_right_mode():
            if self.mode == Mode.LEFT_RIGHT:
                self.lr_state = 'move_forward'

        # Schedule resuming normal left-right behavior after the avoidance maneuver
        self.create_timer(avoidance_duration, resume_left_right_mode)

        def resume_left_right_mode():
            # Check if we are still in LEFT_RIGHT mode before resuming
            if self.mode == Mode.LEFT_RIGHT:
                self.get_logger().info('')
                self.lr_state = 'move_forward'
            else:
                self.get_logger().info('')

            # Schedule resuming normal left-right behavior after the avoidance maneuver
            self.create_timer(avoidance_duration, resume_left_right_mode)

    # Method to check for obstacles using sensor data
    def check_for_obstacles(self):
        threshold_distance = self.obstacle_threshold  # meters
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
        # Log obstacle detected only once, then suppress until the obstacle is cleared
        if not self.has_logged_obstacle:
            self.get_logger().info('Obstacle detected! Executing avoidance maneuver.')
            self.has_logged_obstacle = True  # Set the flag to suppress further logging

        # Stop current movement
        self.stop_robot()

        # Depending on the mode, execute the appropriate avoidance strategy
        if self.mode == Mode.FRONTIER:
            self.avoid_obstacle_frontier()
        elif self.mode == Mode.LEFT_RIGHT:
            self.avoid_obstacle_left_right()
        elif self.mode == Mode.BUG2:
            self.avoid_obstacle_bug2()
        else:
            pass  # Do nothing in idle mode


    # Reset the flag once the obstacle is cleared (for example, in the exploration_behavior or any other relevant method)
    def exploration_behavior(self):
        obstacle_detected = self.check_for_obstacles()

        if obstacle_detected:
            self.handle_obstacle()
            return  # Exit exploration_behavior to focus on obstacle handling
        else:
            # Reset the flag if the obstacle is cleared
            self.has_logged_obstacle = False

        # Continue with mode-specific behavior
        if self.mode == Mode.FRONTIER:
            self.frontier_behavior()
        elif self.mode == Mode.LEFT_RIGHT:
            self.left_right_behavior()
        elif self.mode == Mode.BUG2:
            self.bug2_behavior()



    # Obstacle avoidance for Frontier Mode
    def avoid_obstacle_frontier(self):
        # Check if we are still in FRONTIER mode before proceeding
        if self.mode != Mode.FRONTIER:
            return  # Exit if the mode is no longer FRONTIER

        # Determine turn direction based on sensor readings
        twist = Twist()
        if self.range_fl is not None and self.range_fr is not None:
            if self.range_fl > self.range_fr:
                twist.angular.z = 0.7  # Turn left
            else:
                twist.angular.z = -0.7  # Turn right
        elif self.range_fl is not None:
            twist.angular.z = 0.7  # Turn left
        elif self.range_fr is not None:
            twist.angular.z = -0.7  # Turn right
        else:
            twist.angular.z = 0.7  # Default to turning left

        self.cmd_vel_publisher.publish(twist)

        # Set a timer to resume normal behavior after avoidance maneuver
        avoidance_duration = 1.0  # seconds

        def resume_frontier_mode():
            if self.mode == Mode.FRONTIER:
                pass  # Continue normal operation

        # Schedule resuming normal frontier behavior after the avoidance maneuver
        self.create_timer(avoidance_duration, resume_frontier_mode)

    # Obstacle avoidance for Bug2 Mode
    def avoid_obstacle_bug2(self):
        # Check if we are still in BUG2 mode before proceeding
        if self.mode != Mode.BUG2:
            return  # Exit if the mode is no longer BUG2

        # Determine turn direction based on sensor readings
        twist = Twist()
        if self.range_fl is not None and self.range_fr is not None:
            if self.range_fl > self.range_fr:
                twist.angular.z = 0.7  # Turn left
            else:
                twist.angular.z = -0.7  # Turn right
        elif self.range_fl is not None:
            twist.angular.z = 0.7  # Turn left
        elif self.range_fr is not None:
            twist.angular.z = -0.7  # Turn right
        else:
            twist.angular.z = 0.7  # Default to turning left

        self.cmd_vel_publisher.publish(twist)

        # Set a timer to resume normal behavior after avoidance maneuver
        avoidance_duration = 1.0  # seconds

        def resume_bug2_mode():
            if self.mode == Mode.BUG2:
                pass  # Continue normal operation

        # Schedule resuming normal Bug2 behavior after the avoidance maneuver
        self.create_timer(avoidance_duration, resume_bug2_mode)


    # Bug2 mode behavior
    def bug2_behavior(self):
        if self.bug2_state == Bug2State.MOVING_TO_GOAL:
            self.move_towards_goal()
        elif self.bug2_state == Bug2State.BOUNDARY_FOLLOWING:
            self.follow_boundary()

    def move_towards_goal(self):
        if self.is_goal_reached():
            self.get_logger().info('Goal reached successfully.')
            self.mode = Mode.IDLE
            self.bug2_state = Bug2State.NONE
            self.stop_robot()
            return

        if self.is_obstacle_ahead():
            self.get_logger().info('Obstacle detected! Switching to Boundary Following State.')
            self.bug2_state = Bug2State.BOUNDARY_FOLLOWING
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
            self.get_logger().debug('Bug2 Mode: Moving towards goal.')
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get robot pose: {e}')
            self.stop_robot()

    def follow_boundary(self):
        twist = Twist()
        twist.linear.x = self.boundary_following_speed
        twist.angular.z = self.boundary_turn_speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().debug('Bug2 Mode: Following obstacle boundary.')

        # Check if the robot has rejoined the M-line
        if self.has_rejoined_m_line():
            self.get_logger().info('Rejoined M-line. Switching back to MOVING_TO_GOAL State.')
            self.bug2_state = Bug2State.MOVING_TO_GOAL
            self.stop_robot()

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

    # Method to stop the robot by sending zero velocities
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)


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
