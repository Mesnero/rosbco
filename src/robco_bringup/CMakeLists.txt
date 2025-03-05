cmake_minimum_required(VERSION 3.8)
project(robco_bringup)

# Since we have no actual C++ executables, we only need to find
# the packages that we rely on for launching, etc.
find_package(ament_cmake REQUIRED)

# Install the launch and config directories if they exist
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
