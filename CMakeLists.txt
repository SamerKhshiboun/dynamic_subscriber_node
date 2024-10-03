cmake_minimum_required(VERSION 3.8)
project(dynamic_subscriber_node)

# Ensure C++14 or later is enabled
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add the executable
add_executable(dynamic_subscriber_node src/dynamic_subscriber_node.cpp)

# Link the necessary libraries
ament_target_dependencies(dynamic_subscriber_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install the executable
install(TARGETS dynamic_subscriber_node
  DESTINATION lib/${PROJECT_NAME})

# Package ament configuration
ament_package()
