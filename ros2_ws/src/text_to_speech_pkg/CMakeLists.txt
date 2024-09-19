cmake_minimum_required(VERSION 3.5)
project(text_to_speech_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)  # Added for std_msgs

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install entry point scripts
install(
  PROGRAMS
    text_to_speech_pkg/tts_node.py
    text_to_speech_pkg/tts_publisher.py  # Added publisher script
    text_to_speech_pkg/result_to_tts.py  # Added bridge script
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
