import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder, load_yaml, load_xacro
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.substitutions import Xacro
from pathlib import Path


def launch_setup(context, params):
    # Resolve the servo_config_file_path from the launch context.
    servo_file = LaunchConfiguration('servo_config_file_path').perform(context)
    # Get the absolute path.
    absolute_servo_file = os.path.abspath(servo_file)

    # Build servo_params using the resolved absolute file path.
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml(absolute_servo_file)
        .to_dict()
    }
    
    params.append({"use_joy_republisher": LaunchConfiguration("use_joy_republisher").perform(context).lower() == "true"})
    params.append(servo_params)

    # Create the node with the parameters.
    servo_node = launch_ros.actions.Node(
        package="robco_validator",
        executable="validator",
        parameters=params,
        output="screen",
    )
    return [servo_node]

def generate_launch_description():
    # Get package directories.
    robco_description_share = get_package_share_directory("robco_description")
    robco_validator_share = get_package_share_directory("robco_validator")

    # Declare launch arguments.
    use_joy_republisher_arg = DeclareLaunchArgument(
        'use_joy_republisher',
        default_value='false',
        description='Set to "true" to use JoyRepublisher; otherwise JointJogRepublisher will be used.'
    )

    servo_config_file_path_arg = DeclareLaunchArgument(
        'servo_config_file_path',
        default_value=os.path.join(robco_validator_share, 'config', 'servo_config_cartesian_sim.yaml'),
        description='Filename for the servo config file'
    )

    # Define file paths for static files.
    urdf_path = os.path.join(robco_description_share, "urdf", "robco.xacro")
    joint_limits_path = os.path.join(robco_validator_share, "config", "joint_limits.yaml")
    srdf_path = os.path.join(robco_validator_share, "config", "robco.srdf")
    kinematics_path = os.path.join(robco_validator_share, "config", "kinematics.yaml")
    collision_object_path = os.path.join(robco_validator_share, "config", "collision_objects.yaml")

    params = []
    description = {"robot_description": ParameterValue(Xacro(str(urdf_path)),value_type=str)}
    params.append(description)
    
    semantic = {"robot_description_semantic": ParameterValue(Xacro(str(srdf_path)),value_type=str)}
    params.append(semantic)
    
    kinematics = {"robot_description_kinematics": load_yaml(Path(kinematics_path))}
    params.append(kinematics)
    
    joint_limits = {"robot_description_planning": load_yaml(Path(joint_limits_path))}
    params.append(joint_limits)

    acceleration_filter_update_period = {"update_period": 0.01}
    params.append(acceleration_filter_update_period)
    
    params.append(collision_object_path)

    # Use OpaqueFunction to resolve the launch configuration and build the node.
    opaque_function = OpaqueFunction(
        function=lambda context: launch_setup(
            context,
            params
        )
    )

    return launch.LaunchDescription([
        use_joy_republisher_arg,
        servo_config_file_path_arg,
        opaque_function
    ])
