cmake_minimum_required(VERSION 3.8)
project(core VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED) 
find_package(pluginlib REQUIRED)
find_package(protocol_base REQUIRED)
find_package(ros2_api_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(converter REQUIRED)
find_package(types_api REQUIRED)
find_package(config_parser REQUIRED)
find_package(publisher REQUIRED)
find_package(sensor_msgs REQUIRED)

# Specify include directories
include_directories(
  include
)

# Gather source and header files
file(GLOB SERVER_SOURCES src/*.cpp)
file(GLOB SERVER_HEADERS include/core/*.hpp)

# Add library (this includes config_manager.cpp)
add_library(core SHARED ${SERVER_SOURCES})

# Set header file directory
target_include_directories(core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to the library (core)
target_link_libraries(core yaml-cpp)  # Add yaml-cpp linking explicitly

ament_target_dependencies(core 
  rclcpp 
  yaml-cpp
  pluginlib
  protocol_base
  ros2_api_msgs
  std_msgs
  trajectory_msgs
  converter
  types_api
  config_parser
  publisher
  sensor_msgs
)

# Add executable for the server node
add_executable(ros2_api_node src/ros2_api_main.cpp)

target_link_libraries(ros2_api_node core)

# Install library
install(TARGETS core
  EXPORT coreConfig
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable
install(TARGETS ros2_api_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(
DIRECTORY include/ros2_api/core 
DESTINATION include/ros2_api)

# Install launch and config files
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

# Export package information
ament_export_include_directories(include)
ament_export_libraries(core)
ament_export_dependencies(rclcpp yaml-cpp pluginlib) 

ament_package()
