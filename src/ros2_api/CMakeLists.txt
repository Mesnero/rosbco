cmake_minimum_required(VERSION 3.8)
project(ros2_api VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)  

# Specify include directories
include_directories(
  include
)

# Gather source and header files
file(GLOB SERVER_SOURCES src/*.cpp)
file(GLOB SERVER_HEADERS include/ros2_api/*.hpp)

# Add library (this includes config_manager.cpp)
add_library(ros2_api SHARED ${SERVER_SOURCES})

# Set header file directory
target_include_directories(ros2_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to the library (ros2_api)
target_link_libraries(ros2_api yaml-cpp)  # Add yaml-cpp linking explicitly

ament_target_dependencies(ros2_api rclcpp std_msgs yaml-cpp)

# Add executable for the server node
add_executable(ros2_api_node src/ros2_api_node.cpp)

# Link the `ros2_api` library and `yaml-cpp` to the executable
target_link_libraries(ros2_api_node ros2_api yaml-cpp)

# Install library
install(TARGETS ros2_api
  EXPORT ros2_apiConfig
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable
install(TARGETS ros2_api_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/ DESTINATION include)

# Install launch and config files
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

# Export package information
ament_export_include_directories(include)
ament_export_libraries(ros2_api)
ament_export_dependencies(rclcpp std_msgs yaml-cpp)  # Ensure yaml-cpp is exported

if(BUILD_TESTING)
  # Find required test packages
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(config_manager_test
    test/test_config_manager.cpp
  )

  target_include_directories(config_manager_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(config_manager_test ros2_api)

endif()

ament_package()
