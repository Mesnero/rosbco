import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder, load_yaml
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.substitutions import Xacro
from pathlib import Path


def launch_setup(context, params_servo):
    
    params_validator = []
    
    # Resolve the servo_config_file_path from the launch context.
    servo_file = LaunchConfiguration('servo_config_file_path').perform(context)
    # Get the absolute path.
    absolute_servo_file = os.path.abspath(servo_file)
    
    joint_limits_file = LaunchConfiguration('joint_limits_file_path').perform(context)
    absolute_joint_limits_file = os.path.abspath(joint_limits_file)
    
    collision_object_path = LaunchConfiguration('collision_objects_file_path').perform(context)
    params_validator.append(collision_object_path)

    # Build servo_params using the resolved absolute file path.
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml(absolute_servo_file)
        .to_dict()
    }
    params_servo.append(servo_params)
    params_validator.append(servo_params)
    
    joint_limits = {"robot_description_planning": load_yaml(Path(absolute_joint_limits_file))}
    params_servo.append(joint_limits)
    

    # Create the node with the parameters.
    validator_node = launch_ros.actions.Node(
        package="robco_validator",
        executable="validator",
        parameters=params_validator,
        output="screen",
    )
    
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=params_servo,
        output="screen",
    )
    
    return [servo_node, validator_node]

def generate_launch_description():
    # Get package directories.
    robco_description_share = get_package_share_directory("robco_description")
    robco_validator_share = get_package_share_directory("robco_validator")

    servo_config_file_path_arg = DeclareLaunchArgument(
        'servo_config_file_path',
        default_value=os.path.join(robco_validator_share, 'config', 'servo_config_cartesian_sim.yaml'),
        description='Filename for the servo config file'
    )
    
    collision_objects_file_path_arg = DeclareLaunchArgument(
        'collision_objects_file_path',
        default_value=os.path.join(robco_validator_share, 'config', 'collision_objects.yaml'),
        description='File path for collision objects (will NOT be loaded, just passed)'
    )

    joint_limits_file_path_arg = DeclareLaunchArgument(
        'joint_limits_file_path',
        default_value=os.path.join(robco_validator_share, 'config', 'joint_limits.yaml'),
        description='YAML file containing joint limits'
    )
        
    # Define file paths for static files.
    urdf_path = os.path.join(robco_description_share, "urdf", "robco.xacro")
    srdf_path = os.path.join(robco_validator_share, "config", "robco.srdf")
    kinematics_path = os.path.join(robco_validator_share, "config", "kinematics.yaml")

    params_servo = []
    description = {"robot_description": ParameterValue(Xacro(str(urdf_path)),value_type=str)}
    params_servo.append(description)
    
    semantic = {"robot_description_semantic": ParameterValue(Xacro(str(srdf_path)),value_type=str)}
    params_servo.append(semantic)
    
    kinematics = {"robot_description_kinematics": load_yaml(Path(kinematics_path))}
    params_servo.append(kinematics)
    
    # Use OpaqueFunction to resolve the launch configuration and build the node.
    opaque_function = OpaqueFunction(
        function=lambda context: launch_setup(
            context,
            params_servo
        )
    )

    return launch.LaunchDescription([
        servo_config_file_path_arg,
        collision_objects_file_path_arg,
        joint_limits_file_path_arg,
        opaque_function
    ])
