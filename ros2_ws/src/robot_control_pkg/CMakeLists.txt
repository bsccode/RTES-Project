cmake_minimum_required(VERSION 3.5)
project(robot_control_pkg)

# Default to C++14 (not strictly necessary for Python packages)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install entry point scripts
install(
  PROGRAMS
    robot_control_pkg/robot_control_node.py
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
