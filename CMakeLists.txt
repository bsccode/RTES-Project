cmake_minimum_required(VERSION 3.5)
project(tutorial_pkg)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories (optional if you have headers in an include directory)
include_directories(include)

# Add executables and link the required libraries
add_executable(rotate_robot_publisher src/rotate_robot_publisher.cpp)
ament_target_dependencies(rotate_robot_publisher rclcpp geometry_msgs sensor_msgs cv_bridge OpenCV std_msgs)

add_executable(velocity_subscriber src/velocity_subscriber.cpp)
ament_target_dependencies(velocity_subscriber rclcpp geometry_msgs std_msgs)

# Link OpenCV libraries after the executable is defined
target_link_libraries(rotate_robot_publisher ${OpenCV_LIBRARIES})

# Install executables
install(TARGETS
  rotate_robot_publisher
  velocity_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

