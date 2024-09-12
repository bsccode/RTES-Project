# ROS2 Humble Robot Assistant – Husarion ROSBOT Pro 2.0

## Project Overview

This project aims to develop an autonomous robot assistant using the **Husarion ROSBOT Pro 2.0** as the base unit. The robot will be capable of voice recognition, image recognition, local mapping, and autonomous navigation. It will navigate through a maze-like environment, identify a subject by comparing images, and return to its starting point using the shortest path. The system will integrate several cutting-edge technologies, both onboard and through a base station, to create an interactive and intelligent robotic assistant.

The key functionalities include:
- **Voice Recognition**: The robot will listen and act on spoken commands.
- **Autonomous Navigation & Mapping**: The robot will explore and map the environment, calculating the optimal route to reach a destination.
- **Image Recognition**: Using an advanced detection library, the robot will identify objects or subjects from its environment.
- **Voice Output**: The robot will communicate back to the user with synthesized speech.

## Key Technologies

### ROS2 (Robot Operating System 2)
**ROS2 Humble** will serve as the core framework for robot control, allowing for modular design, sensor fusion, and communication between different components. ROS2 enables real-time execution and reliable system-level integration, crucial for a project that involves navigation, sensor processing, and AI.

### NAV2 and SLAM (Simultaneous Localization and Mapping)
The **NAV2** stack will handle autonomous navigation, allowing the robot to move through the environment while dynamically avoiding obstacles. **SLAM** will be used to create maps of unknown environments and simultaneously track the robot’s position within that map. This allows the robot to navigate a maze-like environment efficiently.

### Whisper STT (Speech-to-Text)
**Whisper STT** will be responsible for processing voice commands. This tool converts spoken language into text that the robot can interpret and act upon. The robot will use Whisper for command recognition, making the interaction with users more natural and intuitive.

### Piper TTS (Text-to-Speech)
**Piper TTS** is the text-to-speech engine used to provide verbal responses. The robot will use this to communicate information back to the user, such as status updates, object identification results, or navigation progress. This adds a conversational layer to the human-robot interaction.

### YOLO (You Only Look Once) – Image Recognition
**YOLO** is a real-time object detection algorithm that will enable the robot to visually recognize objects in its environment. By processing the images from the robot’s camera, YOLO will allow the robot to identify and compare objects or subjects against predefined data, enhancing the robot's decision-making process.

### Micro-ROS
**Micro-ROS** extends ROS2 to smaller, resource-constrained devices. In this project, Micro-ROS will be used for embedded system communications, particularly for interfacing with the sensors and actuators on the ROSBOT. This allows the robot to perform low-level operations efficiently while keeping in sync with the ROS2 ecosystem.

## User Interface and Control

The project includes the development of a **Human-Machine Interface (HMI)** on a base station. The HMI will provide functionalities such as:
- Manual control of the robot.
- Image verification and analysis.
- Communication with the robot (sending commands, receiving feedback).
  
In case the robot encounters computationally intensive tasks (e.g., complex image recognition), the base station PC will offload these tasks to ensure smooth and real-time operations.

## Future Enhancements

This project aims to create a foundation for autonomous robot assistants. Some possible future enhancements include:
- **Advanced Maze Navigation**: Improving the robot’s decision-making and path-planning algorithms for more complex environments.
- **Optimized Image Recognition**: Enhancing the accuracy of the YOLO model for faster and more reliable object detection.
- **Dynamic Task Offloading**: Expanding the system's ability to offload computation-heavy tasks between the robot and the base station dynamically.

