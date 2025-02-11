cmake_minimum_required(VERSION 3.5)
project(multi_subscriber)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ros2_api_msgs REQUIRED)   
find_package(geometry_msgs REQUIRED)

# Build the subscriber executable.
add_executable(multi_subscriber_node src/multi_subscriber.cpp)
ament_target_dependencies(multi_subscriber_node rclcpp std_msgs trajectory_msgs)

# Build the publisher executable.
add_executable(multi_publisher_node src/multi_publisher.cpp)
ament_target_dependencies(multi_publisher_node rclcpp ros2_api_msgs geometry_msgs)

# Install executables.
install(TARGETS
  multi_subscriber_node
  multi_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
