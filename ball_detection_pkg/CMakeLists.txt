cmake_minimum_required(VERSION 3.8)
project(ball_detection_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories (optional)
include_directories(include)

# Add executables and link required libraries
add_executable(ball_color_detector src/ball_color_detector.cpp)
ament_target_dependencies(ball_color_detector rclcpp sensor_msgs cv_bridge OpenCV std_msgs)

add_executable(ball_info_subscriber src/ball_info_subscriber.cpp)
ament_target_dependencies(ball_info_subscriber rclcpp std_msgs)

# Link OpenCV libraries
target_link_libraries(ball_color_detector ${OpenCV_LIBRARIES})

# Install executables
install(TARGETS
  ball_color_detector
  ball_info_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()
