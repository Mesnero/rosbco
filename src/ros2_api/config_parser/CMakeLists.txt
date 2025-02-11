cmake_minimum_required(VERSION 3.8)
project(config_parser VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED) 
find_package(types_api REQUIRED)

# Specify include directories
include_directories(
  include
)

# Gather source and header files
file(GLOB SERVER_SOURCES src/*.cpp)
file(GLOB SERVER_HEADERS include/config_parser/*.hpp)

add_library(config_parser SHARED ${SERVER_SOURCES})

# Set header file directory
target_include_directories(config_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(config_parser yaml-cpp)  # Add yaml-cpp linking explicitly

ament_target_dependencies(config_parser 
  rclcpp 
  yaml-cpp
  types_api
)

# Install library
install(TARGETS config_parser
  EXPORT config_parserConfig
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(
  DIRECTORY include/ros2_api/config_parser 
  DESTINATION include/ros2_api
)


# Export package information
ament_export_include_directories(include)
ament_export_libraries(config_parser)
ament_export_dependencies(rclcpp yaml-cpp) 

ament_package()
