```markdown
# ROS2 Humble Robot Assistant – Husarion ROSBOT Pro 2.0

## Project Overview

### **Key Functionalities:**
- **Voice Recognition:** Interprets spoken commands to control the robot.
- **Autonomous Navigation & Mapping:** Explores and maps environments, calculating optimal routes.
- **Image Recognition:** Identifies objects or subjects using advanced detection algorithms.
- **Voice Output:** Communicates responses and status updates through synthesized speech.

## Key Technologies


### [Vosk ROS](https://github.com/alphacep/vosk-api)
**Vosk ROS** handles voice recognition, converting spoken language into text that the robot can interpret and act upon, enabling natural and intuitive user interactions.

### [Piper TTS (Text-to-Speech)](https://github.com/rhasspy/piper)
**Piper TTS** synthesizes text responses into speech, allowing the robot to communicate verbally with users, providing feedback and status updates.

## Package Descriptions

### 1. `command_processor_pkg`
**Function:**  
Processes incoming text commands, interprets them, and publishes appropriate actions or mode changes to control the robot.

**Components:**
- **Node:** `command_processor_node.py`
- **Subscriptions:** `/result` (for incoming commands)
- **Publications:** `/robot_commands` (to issue movement or mode commands), `/text_to_speech` (for verbal feedback)

**How It Works:**
- **Listening for Commands:** Subscribes to the `/result` topic, which receives transcribed text from the `voskros` package.
- **Command Parsing:** Upon receiving a message, it parses the command to determine the intended action, such as changing modes or executing movement commands.
- **Publishing Actions:** Depending on the parsed command, it publishes messages to the `/robot_commands` topic, which the `robot_control_pkg` listens to for executing movements or mode changes.
- **Feedback Mechanism:** After processing a command, it publishes a response to the `/text_to_speech` topic, enabling the robot to provide audible feedback to the user.

**Key Features:**
- **Mode Management:** Handles transitions between different operational modes (e.g., `idle`, `active`, `roaming`).
- **Movement Command Handling:** Processes movement-related commands and forwards them appropriately.
- **Error Handling:** Responds with appropriate messages when encountering unknown or malformed commands.

### 2. `text_to_speech_pkg`
**Function:**  
Converts text messages into audible speech, enabling the robot to communicate responses and status updates to users.

**Components:**
- **Node:** `text_to_speech_node.py`
- **Subscriptions:** `/text_to_speech` (for messages to be spoken)

**How It Works:**
- **Listening for Speech Commands:** Subscribes to the `/text_to_speech` topic to receive text messages that need to be converted into speech.
- **Speech Synthesis:** Utilizes **Piper TTS** to convert incoming text messages into audible speech.
- **Audio Output:** Plays the synthesized speech through the robot's speaker system, providing real-time verbal feedback and responses to user commands.

**Key Features:**
- **Real-Time Feedback:** Provides immediate audible responses, enhancing user interaction.
- **Configurable Voices:** Supports multiple languages and voice configurations through Piper TTS settings.
- **Seamless Integration:** Works in tandem with the `command_processor_pkg` to deliver coherent communication flows.

### 3. `voskros`
**Function:**  
Handles voice-to-text transcription using the Vosk STT engine, enabling the robot to interpret spoken commands.

**Components:**
- **Node:** `vosk_transcription_node.py`
- **Publications:** `/result` (for transcribed text)

**How It Works:**
- **Audio Capture:** Continuously captures audio input from the robot's microphone.
- **Speech Recognition:** Processes the audio stream using **Vosk STT** to convert spoken language into text.
- **Publishing Transcriptions:** Publishes the transcribed text to the `/result` topic, which the `command_processor_pkg` listens to for further processing.

**Why Vosk ROS Over Whisper:**
- **Resource Efficiency:** Vosk is lightweight and optimized for real-time applications, making it more suitable for deployment on resource-constrained hardware like the ROSBOT Pro 2.0.
- **Low Latency:** Offers faster transcription speeds with lower latency compared to Whisper, ensuring responsive interactions.
- **Seamless ROS2 Integration:** Vosk ROS provides straightforward integration with ROS2, simplifying development and deployment within the ROS ecosystem.
- **Open-Source Flexibility:** Vosk's open-source nature allows for easier customization and adaptation to specific project needs without licensing constraints.

**Key Features:**
- **Real-Time Transcription:** Provides immediate conversion of spoken commands into text for timely robot responses.
- **Language Support:** Supports multiple languages and dialects, enhancing versatility in diverse environments.
- **Scalability:** Can handle continuous speech input, making it ideal for dynamic interaction scenarios.

### 4. `robot_control_pkg`
**Function:**  
Manages the robot's movement and navigation based on received commands, including autonomous roaming within defined areas.

**Components:**
- **Node:** `robot_control_node.py`
- **Subscriptions:** `/robot_commands` (for movement and mode instructions)
- **Publications:** `/cmd_vel` (to control robot motion)

**How It Works:**
- **Listening for Commands:** Subscribes to the `/robot_commands` topic to receive instructions for movement or mode changes.
- **Command Interpretation:** Parses incoming commands to determine actions such as moving forward, turning, or changing operational modes.
- **Velocity Control:** Publishes `Twist` messages to the `/cmd_vel` topic to control the robot's linear and angular velocities, enabling movement.
- **Autonomous Roaming:** Implements a stateful roaming behavior where the robot selects movement directions for set durations, ensuring smooth and continuous navigation.
- **Mode Management:** Handles transitions between different modes (e.g., entering/exiting roaming mode), ensuring that movements are consistent with the current operational state.

**Key Features:**
- **Smooth Movement Execution:** Implements strategies to maintain consistent movement directions, reducing jitteriness and enhancing navigation fluidity.
- **Stateful Roaming:** Maintains current movement states and durations to simulate held commands, providing stable roaming behavior.
- **Integration with NAV2 and SLAM:** Leverages ROS2's navigation stack for advanced path planning and environment mapping, enabling efficient and obstacle-aware navigation.
- **Error Handling:** Monitors and logs unknown or conflicting commands, ensuring reliable operation and easy troubleshooting.

**Recent Enhancements:**
- **Roaming Mode Implementation:** Allows the robot to autonomously roam within an area, continuously selecting and executing movement directions until instructed to stop.
- **Speed Optimization:** Increased roaming and turn speeds to enhance movement responsiveness while maintaining control and stability.
- **Polling Rate Adjustment:** Enhanced the polling rate during movement durations to simulate holding movement commands, contributing to smoother navigation.

## Commands Cheat Sheet

Interact with your robot using the following ROS2 terminal commands. These commands publish messages to specific topics, simulating voice commands or direct controls.

### **1. Change Mode Commands**

These commands change the robot's operational mode.

- **Activate Roaming Mode**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'change mode to roaming'"
  ```
  **Or Directly to Robot Control:**
  ```bash
  ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'set_mode_roaming'"
  ```

- **Activate Active Mode**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'change mode to active'"
  ```
  **Or Directly to Robot Control:**
  ```bash
  ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'set_mode_active'"
  ```

- **Set Mode to Idle**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'change mode to idle'"
  ```
  **Or Directly to Robot Control:**
  ```bash
  ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'set_mode_idle'"
  ```

### **2. Movement Commands**

These commands control the robot's movement.

- **Move Forward**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'move forward'"
  ```
  **Or**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'move_forward'"
  ```
  **Directly to Robot Control:**
  ```bash
  ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'move_forward'"
  ```

- **Move Backward**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'move backward'"
  ```
  **Or**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'move_backward'"
  ```
  **Directly to Robot Control:**
  ```bash
  ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'move_backward'"
  ```

- **Turn Left**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'turn left'"
  ```
  **Or**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'turn_left'"
  ```
  **Directly to Robot Control:**
  ```bash
  ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'turn_left'"
  ```

- **Turn Right**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'turn right'"
  ```
  **Or**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'turn_right'"
  ```
  **Directly to Robot Control:**
  ```bash
  ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'turn_right'"
  ```

- **Stop Movement**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'stop'"
  ```
  **Or Directly to Robot Control:**
  ```bash
  ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'stop'"
  ```

### **3. Inquiry Command**

These commands request information from the robot.

- **Ask Robot's Name**
  ```bash
  ros2 topic pub --once /result std_msgs/msg/String "data: 'what is your name?'"
  ```
  **Or Directly to Robot Control (if applicable):**
  ```bash
  ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'what is your name?'"
  ```

## How to Use the Commands

You can interact with your robot in two primary ways:

1. **Via `/result` Topic:**
   - Simulates voice commands processed by the `command_processor_pkg`.
   - Example:
     ```bash
     ros2 topic pub --once /result std_msgs/msg/String "data: 'move forward'"
     ```

2. **Directly via `/robot_commands` Topic:**
   - Bypasses the `command_processor_pkg` for direct control.
   - Example:
     ```bash
     ros2 topic pub --once /robot_commands std_msgs/msg/String "data: 'turn_left'"
     ```

**Note:** Both methods achieve the same end result. Use the `/result` topic for voice-command-like interactions and `/robot_commands` for programmatic or direct control.



### **7. Interact with the Robot**

Use the **Commands Cheat Sheet** to send commands via the terminal and observe the robot's responses and movements.

## Future Enhancements

- **Advanced Maze Navigation:** Improve decision-making and path-planning algorithms for more complex environments.
- **Optimized Image Recognition:** Enhance YOLO model accuracy for faster and more reliable object detection.
- **Dynamic Task Offloading:** Expand the system's ability to offload computation-heavy tasks between the robot and the base station dynamically.
- **Sensor Integration:** Incorporate additional sensors for obstacle detection and environmental awareness.
- **Natural Language Processing:** Implement more sophisticated NLP for enhanced command interpretation.
- **Energy Management:** Optimize power consumption for longer operational periods.
- **User Interface Enhancements:** Develop a more intuitive HMI for easier control and monitoring.

