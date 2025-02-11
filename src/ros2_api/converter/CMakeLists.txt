cmake_minimum_required(VERSION 3.8)

project(converter VERSION 0.1.0 LANGUAGES CXX)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ros2_api_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(types_api REQUIRED)
find_package(config_parser REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(sensor_msgs REQUIRED)

# Specify include directories
include_directories(
  include
)

# Gather source and header files
file(GLOB SERVER_SOURCES src/*.cpp)
file(GLOB SERVER_HEADERS include/converter/*.hpp)

add_library(converter SHARED ${SERVER_SOURCES})

target_link_libraries(converter nlohmann_json::nlohmann_json)

target_include_directories(converter PUBLIC
  ${nlohmann_json_SOURCE_DIR}/single_include
)


target_include_directories(converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(converter 
  ros2_api_msgs
  std_msgs
  trajectory_msgs
  geometry_msgs
  builtin_interfaces
  rclcpp
  types_api
  config_parser
  sensor_msgs
)

# install library headers
install(
  DIRECTORY include/ros2_api/converter
  DESTINATION include/ros2_api
)

# install library binaries
install(
  TARGETS converter
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
  converter
)


ament_package()