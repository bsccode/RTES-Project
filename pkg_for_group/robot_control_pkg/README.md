
# Robot Control System

*Date: October 4th, 2024*

This repository contains the Robot Control System, a modular framework for autonomous robotic navigation using ROS 2. The system includes multiple navigation modes, obstacle avoidance, and exploration capabilities. It leverages advanced algorithms such as **Bug2** and **Frontier-Based Exploration**, with real-time visualization and control using RViz2.

## Table of Contents

1. [Introduction](#introduction)
2. [Features](#features)
3. [Requirements](#requirements)
4. [Installation](#installation)
5. [Navigation Modes](#navigation-modes)
    - [Bug2 Mode](#bug2-mode)
    - [Left-Right Mode](#left-right-mode)
    - [Frontier Exploration Mode](#frontier-exploration-mode)
    - [Stop Command](#stop-command)
6. [Obstacle Avoidance](#obstacle-avoidance)
7. [Code Structure](#code-structure)
    - [Left-to-Right Navigation](#left-to-right-navigation)
    - [Advanced Robot Control Node](#advanced-robot-control-node)
    - [Bug2 Navigator Node](#bug2-navigator-node)
8. [Operational Flow](#operational-flow)
9. [Theoretical Concepts](#theoretical-concepts)
10. [Future Directions](#future-directions)

---

## Introduction

This project provides a comprehensive autonomous robot control system built using ROS 2. It integrates multiple navigation strategies like **Bug2**, **Left-Right**, and **Frontier-Based Exploration**, along with real-time obstacle detection and avoidance. The robot explores the environment autonomously, detects obstacles using LiDAR sensors, and adjusts its movement strategies to ensure safe navigation.

---

## Features

- **Autonomous Navigation**: Multiple navigation modes including Bug2, Left-Right, and Frontier exploration.
- **SLAM Integration**: Uses ROS 2 SLAM to map and navigate an unknown environment.
- **Obstacle Avoidance**: The robot can detect obstacles and react dynamically to avoid them.
- **Interactive Commands**: ROS 2 topics allow for real-time control and mode switching.
- **Visualization**: RViz2 markers show frontiers, goals, and robot movement in real-time.

---

## Requirements

- ROS 2 Humble
- Python 3.10 or later
- Dependencies:
  - `nav_msgs`
  - `std_msgs`
  - `geometry_msgs`
  - `sensor_msgs`
  - `nav2_msgs`
  - `tf2_ros`
  - `rclpy`
  - `numpy`
  - `visualization_msgs`

---

## Installation

1. **Install ROS 2 Humble**: [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
2. **Create a ROS 2 Workspace**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
3. **Clone this repository into your ROS 2 workspace**:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/yourusername/robot_control_pkg.git
    cd ..
    colcon build
    ```
4. **Run the node**:
    ```bash
    source install/setup.bash
    ros2 run robot_control_pkg robot_control_node
    ```

---

## Navigation Modes

### Bug2 Mode

**Theory**: The Bug2 algorithm attempts to move the robot directly toward a goal. If an obstacle is encountered, the robot follows the obstacle boundary until it can resume the direct path to the goal.

- **Command to start Bug2 Mode**:
    ```bash
    ros2 topic pub -1 /roaming_command std_msgs/String "data: 'start_bug2'"
    ```

- **Command to set a Bug2 goal**:
    ```bash
    ros2 topic pub -1 /roaming_command std_msgs/String "data: 'set_goal x y'"
    ```

- **Command to stop Bug2 Mode**:
    ```bash
    ros2 topic pub -1 /roaming_command std_msgs/String "data: 'stop_bug2'"
    ```

### Left-Right Mode

**Theory**: In Left-Right navigation, the robot alternates between forward motion and shifting left or right after a certain distance. It reacts to obstacles by turning in the direction with more space.

- **Command to start Left-Right Mode**:
    ```bash
    ros2 topic pub -1 /roaming_command std_msgs/String "data: 'start_left_right'"
    ```

- **Command to stop Left-Right Mode**:
    ```bash
    ros2 topic pub -1 /roaming_command std_msgs/String "data: 'stop_left_right'"
    ```

### Frontier Exploration Mode

**Theory**: Frontier exploration guides the robot towards the boundary between known and unknown space, which allows it to efficiently map new areas of the environment.

- **Command to start Frontier Mode**:
    ```bash
    ros2 topic pub -1 /roaming_command std_msgs/String "data: 'start_frontier'"
    ```

- **Command to stop Frontier Mode**:
    ```bash
    ros2 topic pub -1 /roaming_command std_msgs/String "data: 'stop_frontier'"
    ```

### Stop Command

**Stop all modes**:
```bash
ros2 topic pub -1 /roaming_command std_msgs/String "data: 'stop'"
```

---

## Obstacle Avoidance

The robot detects obstacles using front-left and front-right LiDAR sensors. Depending on the mode, the robot adjusts its angular velocity to avoid collisions.

- **In Bug2 Mode**: The robot switches to boundary following to avoid obstacles.
- **In Left-Right Mode**: The robot turns left or right based on which side has more space.
- **In Frontier Mode**: The robot rotates to avoid obstacles and then resumes exploration.

---

## Code Structure

### Left-to-Right Navigation

The `left_to_right.py` script is a basic ROS 2 node enabling simple navigation. It handles manual movement commands and autonomous zig-zag motion.

**Core Features**:
- Manual movement commands: `'move forward'`, `'turn left'`, `'turn right'`.
- Obstacle detection using LiDAR sensors.
- Zig-zag (Left-Right) movement based on sensor input.

### Advanced Robot Control Node

The `robot_control_node.py` script represents a more advanced node with SLAM integration, autonomous exploration, and multiple navigation modes.

**Key Concepts**:
- **SLAM**: Receives occupancy grids to map the environment.
- **Frontier Exploration**: Identifies and navigates to frontiers (unexplored regions).
- **Action Client**: Uses the `NavigateToPose` action to send navigation goals.
- **Clustering**: Groups nearby frontiers using a greedy clustering algorithm.

### Bug2 Navigator Node

The `bug2_navigator_node.py` script implements the Bug2 algorithm, allowing the robot to navigate toward a goal while avoiding obstacles by following their boundary.

**Key Concepts**:
- **Bug2 Algorithm**: A path-planning algorithm that switches between moving toward the goal and boundary following.
- **Goal Reaching**: Checks if the robot has rejoined the M-line after avoiding obstacles.
- **Boundary Following**: Circumnavigates obstacles until it can resume the path to the goal.

---

## Operational Flow

### Initialization

- The robot starts in `IDLE` mode.
- Publishers and subscribers are set up to handle incoming sensor data, occupancy grids, and movement commands.

### Frontier Exploration

1. **Frontier Detection**: Analyzes occupancy grids to detect unexplored regions.
2. **Clustering**: Groups frontiers using a greedy clustering algorithm to reduce redundancy.
3. **Navigation**: The robot navigates toward selected frontiers to explore new areas.

### Bug2 Navigation

1. **Move to Goal**: The robot moves directly toward the goal.
2. **Obstacle Detection**: If an obstacle is detected, the robot switches to boundary following.
3. **Rejoin M-line**: The robot checks if it can rejoin the straight line to the goal.

### Left-Right Navigation

- The robot moves forward and shifts left or right in a zig-zag pattern. It reacts to obstacles by adjusting the turn direction based on sensor input.

---

## Theoretical Concepts

### ROS2 Nodes

Nodes are the fundamental building blocks of ROS 2 systems. Each node performs a specific function and communicates with other nodes via topics, services, and actions.

### Publishers and Subscribers

- **Publishers** send messages on topics.
- **Subscribers** listen for messages on topics.

Example: `robot_control_node.py` publishes velocity commands on `/cmd_vel` and subscribes to sensor data on `/range/fl` and `/range/fr`.

### Action Clients

Action clients are used to send long-running tasks like navigation goals. For example, the `NavigateToPose` action sends goals to the robot for reaching a specific position.

### Frontier-Based Exploration

Frontier-based exploration uses occupancy grids to identify frontiers—boundaries between known and unknown space. The robot navigates to these frontiers to explore new areas.

### Bug2 Algorithm

The Bug2 algorithm enables the robot to navigate toward a goal while avoiding obstacles. It switches between moving toward the goal and following the boundary of obstacles when blocked.

---

Sure! Here's the expanded section on how the implementations of each algorithm work in the system:

---

## Implementation of Algorithms

### 1. Bug2 Algorithm

**Theory**: The Bug2 algorithm is a simple but effective navigation strategy that alternates between moving directly toward the goal and following the boundaries of obstacles. The robot follows a straight line from its current position to the goal, known as the **M-line**. When it encounters an obstacle, the robot switches to **boundary-following** behavior. Once it navigates around the obstacle, it checks if it can rejoin the M-line and resumes moving toward the goal.

#### Algorithm Breakdown:
1. **Move Towards Goal (M-Line Navigation)**:
   - The robot calculates the direction to the goal and moves forward.
   - This phase continues until an obstacle is detected.
   - The robot continuously checks its distance to the goal to decide if it has reached it.
  
2. **Obstacle Detection**:
   - The robot uses LiDAR sensors to check for obstacles.
   - If an obstacle is detected within a predefined threshold distance, the robot switches to boundary-following mode.

3. **Boundary Following**:
   - The robot moves along the boundary of the detected obstacle.
   - It continuously checks if it has rejoined the M-line.
   - Once it re-encounters the M-line, the robot switches back to M-line navigation and continues toward the goal.

4. **Goal Reaching**:
   - If the robot reaches the goal within a threshold distance, it stops and enters an idle state.

#### Code Implementation:
- **Main Components**:
  - `move_towards_goal()`: Calculates the direction to the goal using the current pose of the robot and sends velocity commands to move toward it.
  - `follow_boundary()`: Adjusts the robot’s speed and angular velocity to follow the contour of an obstacle.
  - `has_rejoined_m_line()`: Checks if the robot is within a predefined distance from the M-line to rejoin it after avoiding an obstacle.
  - `is_goal_reached()`: Monitors the distance to the goal and stops the robot when it is close enough.

- **ROS 2 Integration**:
  - **Subscriptions**: The robot subscribes to LiDAR topics (`/range/fl` and `/range/fr`) for obstacle detection.
  - **Transformations**: The robot tracks its position using the `tf2_ros` transformation system to compare its current location with the goal.
  - **Action Clients**: For setting goals and tracking its progress, the robot uses the `NavigateToPose` action client.

### 2. Left-Right Mode

**Theory**: Left-Right mode is a navigation pattern where the robot alternates between moving forward and shifting left or right after covering a certain distance. This method is useful for area coverage or situations where the robot needs to search in a zig-zag pattern. The mode includes built-in obstacle avoidance, where the robot decides to turn based on which side (left or right) has more space when an obstacle is detected.

#### Algorithm Breakdown:
1. **Move Forward**:
   - The robot moves forward for a predefined distance.
   - This is implemented as a time-based forward movement.

2. **Shifting**:
   - Once the robot has moved forward for a certain distance, it shifts either left or right.
   - The shift involves rotating 90 degrees and moving in the new direction.

3. **Obstacle Avoidance**:
   - During movement, the robot checks its LiDAR sensors for obstacles.
   - If it detects an obstacle, it turns in the direction with more space (either left or right) and continues shifting after avoiding the obstacle.

#### Code Implementation:
- **Main Components**:
  - `left_right_behavior()`: Controls the robot’s zig-zag movement by alternating between forward movement and lateral shifts.
  - `lr_shift_direction()`: Toggles the robot’s direction (left or right) based on its current state.
  - `avoid_obstacle_left_right()`: Handles obstacle avoidance by comparing the sensor data from the front-left and front-right LiDAR sensors, then turning toward the side with more free space.

- **ROS 2 Integration**:
  - **Subscriptions**: The robot listens to `/range/fl` and `/range/fr` topics to detect obstacles and adjust its movement accordingly.
  - **Timers**: The mode uses ROS 2 timers to handle the movement and shifts based on predefined durations, creating a periodic zig-zag pattern.
  - **Velocity Commands**: It uses the `Twist` message type to publish movement commands to the `/cmd_vel` topic, controlling the linear and angular velocities.

### 3. Frontier-Based Exploration

**Theory**: Frontier exploration is a method used for autonomous exploration in robotics. A **frontier** is defined as the boundary between known (mapped) and unknown (unmapped) regions in the environment. The robot navigates toward frontiers to maximize coverage and ensure that it explores the entire environment. Once the robot reaches a frontier, it attempts to map the new area and find new frontiers, continuing the process until the environment is fully explored.

#### Algorithm Breakdown:
1. **Occupancy Grid Analysis**:
   - The robot continuously receives occupancy grid data from the SLAM system.
   - The grid represents the environment as a 2D map where each cell is either free, occupied, or unknown.

2. **Frontier Detection**:
   - The algorithm analyzes the occupancy grid to find frontier points, which are cells that are adjacent to unknown areas.
   - A cell is considered a frontier if it is free and has at least one neighboring cell that is unknown.

3. **Frontier Clustering**:
   - The detected frontiers are grouped into clusters using a greedy clustering algorithm.
   - Clustering reduces the number of frontier points and ensures the robot selects representative targets.

4. **Navigation to Frontier**:
   - The robot selects the nearest unvisited frontier and uses the Nav2 system to navigate toward it.
   - Once it reaches a frontier, the process repeats, and the robot selects a new frontier.

#### Code Implementation:
- **Main Components**:
  - `find_frontiers()`: Analyzes the occupancy grid to detect frontiers. It iterates through each grid cell and checks for free cells adjacent to unknown cells.
  - `cluster_frontiers()`: Groups nearby frontiers into clusters based on a distance threshold (`eps`). This reduces the number of exploration targets and ensures efficient coverage.
  - `navigate_to_frontier()`: Uses the `NavigateToPose` action client to send navigation goals to the nearest frontier. The robot continues exploring the environment by moving from one frontier to the next.

- **ROS 2 Integration**:
  - **Subscriptions**: The robot subscribes to the `/map` topic, which provides the occupancy grid from the SLAM system.
  - **Occupancy Grid**: The algorithm works by analyzing the occupancy grid data to determine the frontiers.
  - **Action Clients**: The robot uses the `NavigateToPose` action to send navigation goals to selected frontiers.

### 4. Obstacle Avoidance

**Theory**: Obstacle avoidance is critical for safe autonomous navigation. The system detects obstacles using front-left and front-right LiDAR sensors. Based on sensor readings, the robot decides whether to stop or adjust its path by turning left or right. The goal is to avoid collisions while maintaining smooth movement, especially in modes like Bug2 and Left-Right navigation.

#### Algorithm Breakdown:
1. **Obstacle Detection**:
   - The robot continuously monitors LiDAR sensor data from its front-left and front-right sensors.
   - If an obstacle is detected within a certain threshold distance (e.g., 0.5 meters), the robot switches to avoidance mode.

2. **Avoidance Maneuver**:
   - The robot compares the space available on the left and right sides.
   - It turns toward the side with more free space to avoid the obstacle.

3. **Resumption of Normal Movement**:
   - Once the robot successfully avoids the obstacle, it resumes its previous navigation mode (Bug2, Left-Right, or Frontier Exploration).

#### Code Implementation:
- **Main Components**:
  - `check_for_obstacles()`: Checks the distance values from the LiDAR sensors and determines if an obstacle is within the threshold distance.
  - `handle_obstacle()`: Logs the obstacle detection, stops the robot, and initiates the appropriate obstacle avoidance behavior based on the current mode.
  - `avoid_obstacle_left_right()` and `avoid_obstacle_bug2()`: These methods handle specific avoidance maneuvers for the Left-Right and Bug2 modes, respectively.

- **ROS 2 Integration**:
  - **LiDAR Subscriptions**: The robot subscribes to the `/range/fl` and `/range/fr` topics for sensor data.
  - **Velocity Commands**: When an obstacle is detected, the robot adjusts its velocity by publishing new `Twist` messages to the `/cmd_vel` topic, controlling its angular and linear speed to avoid the obstacle.

---

## Algorithm Comparisons and Summary

- **Bug2 Algorithm**: This algorithm is best suited for goal-oriented navigation in environments with obstacles. Its main advantage is its simplicity and reliability in ensuring that the robot reaches the goal if a path exists. However, it may take longer to reach the goal due to boundary following.
  
- **Left-Right Navigation**: This pattern is ideal for area coverage or structured exploration. Its simplicity and efficiency make it useful for situations where systematic exploration is needed. The zig-zag movement allows the robot to cover large areas but may be less efficient in environments with complex obstacle configurations.

- **Frontier Exploration**: This algorithm is designed for efficient exploration of unknown environments. By focusing on frontiers, the robot maximizes its coverage of the environment. It works well in open, unknown areas but requires SLAM for mapping.

- **Obstacle Avoidance**: This is a fundamental component of any autonomous robot. The system uses real-time sensor data to avoid collisions and ensure safe navigation, integrating smoothly with the other navigation algorithms.

Each of these algorithms contributes to the robot's autonomy and flexibility in different scenarios. The system is designed to switch between these algorithms based on the robot's mode, ensuring that the robot can navigate efficiently, explore unknown areas, and avoid obstacles in a variety of environments.

---

## Future Directions

1. **Advanced Clustering Algorithms**: Implement more sophisticated clustering algorithms to improve exploration efficiency.
2. **Sensor Fusion**: Integrate additional sensors like IMUs to enhance localization and navigation accuracy.
3. **Energy Management**: Incorporate power management strategies for

 longer operation times.
4. **Machine Learning Integration**: Leverage machine learning to improve obstacle avoidance and adaptive navigation.

