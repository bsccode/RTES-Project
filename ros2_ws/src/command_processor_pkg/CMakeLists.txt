cmake_minimum_required(VERSION 3.5)
project(command_processor_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# If you have additional dependencies, find them here
# find_package(another_package REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install entry point scripts
install(
  PROGRAMS
    command_processor_pkg/command_processor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# Install other resources (if any)
# install(
#   DIRECTORY resource
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
