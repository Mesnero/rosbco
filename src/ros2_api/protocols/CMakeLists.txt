cmake_minimum_required(VERSION 3.8)
project(protocols VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(protocol_base REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)

add_library(protocols src/unix_domain_socket.cpp)

target_include_directories(protocols PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  protocols
  pluginlib
  protocol_base
  yaml-cpp
  rclcpp
)

pluginlib_export_plugin_description_file(protocol_base plugins.xml)

# install library headers
install(
  DIRECTORY include/ros2_api/protocols
  DESTINATION include/ros2_api)

# install library binaries
install(
  TARGETS protocols
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#! export headers and binaries for ament
ament_export_include_directories(
  include)
ament_export_libraries(
  protocols
)
ament_export_targets(
  export_${PROJECT_NAME})

ament_package()