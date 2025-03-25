# Rosbco
## Overview
This project is a ROS2 package for the RobCo industrial robot including: A custom API, a Validator, a URDF based description and ROS2 Control.
## Packages
1. [robco_bringup](https://github.com/Mesnero/rosbco/tree/main/src/robco_bringup): 4 launch files with configuration to launch the project
2. [robco_description](https://github.com/Mesnero/rosbco/tree/main/src/robco_description): URDF based description. Also includes configuration for gazebo and ros2 control
3. [robco_hw](https://github.com/Mesnero/robco_hw/tree/30fc28cb0a967cc33d9e9ec78d64a999a6730ad2): Git submodule linking to the Hardware Interface repo. Implementation of a ros2_control hardware interface, for communication with the robot. (More details in repo)
4. [robco_validator](https://github.com/Mesnero/rosbco/tree/main/src/robco_validator): A validator that checks incomming commands against limits defined in yaml and collision objects, also defined in yaml
5. [robcomm](https://github.com/Mesnero/rosbco/tree/main/src/robcomm): Driver provided by RobCo to communicate with the real hardware. Used by the hardware interface.
6. [ros2_api](https://github.com/Mesnero/ros2-api/tree/b5fa434124187328734ab6808b1bd62fecc78e46): Git submodule to the ros2_api. In depth documentation can be found in the repo.

## Building the project
To build the project, follow these steps:
1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) (Has to be run on Ubuntu 22.04.)
2. Install the [MoveIt binary](https://moveit.ai/install-moveit2/binary/)
3. Source the ROS2 installation path
4. Clone the repository.
5. Navigate to the root directory of the project.
6. Run the following commands:

`rosdep install --from-paths src --ignore-src -r -y`

`colcon build`

7. Source the built version with `source install/setup.bash`
## Running the project
To run the project, you can use one of four lanch files:

`ros2  launch  robco_bringup  robco_velocity_sim.launch.py`: Starts the API with the config in the /config path of robco_bringup. The validator expects velocity commands. Gazebo is started for simulation.

`ros2  launch  robco_bringup  robco_velocity_real.launch.py`: Starts the API with the config in the /config path of robco_bringup. The validator expects velocity commands. robco_hw is used for communication.

`ros2  launch  robco_bringup  robco_gamepad_sim.launch.py`: EXPERIMENTAL!!! The try to add gamepad support for easy testing. Still buggy. Launches gazebo.

`ros2  launch  robco_bringup  robco_gamepad_real.launch.py`: EXPERIMENTAL!!! The try to add gamepad support for easy testing. Still buggy. Not recommended on real robot!!!

Each command can have the argument `start_rvit:=false` set to not start RViz. 




## Configuration
In robco_bringup 3 yaml files can be adjusted:
- `api_config.yaml`: Configuration for the API. The structure can be found in the [ROS2 Api README](https://github.com/Mesnero/ros2-api/tree/b5fa434124187328734ab6808b1bd62fecc78e46)
- `collision_objects.yaml`: The collision objects to be avoided by the validator. The workspace creates six walls around the base of the robot. Each wall has the distance set in the yaml file in meters. Additional cuboidal collision objects can be created, by setting the width, depth and height in meters and the center x,y,z relative to the base in meters.
- `joint_limits.yaml`: The joint limits each drive has. Important: Gazebo still enforces the limits set in the URDF. If values are bigger than the ones set in the URDF, the "old" limits will be enforced by gazebo.

