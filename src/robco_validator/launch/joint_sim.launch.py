import os
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the share directory where your servo_launch.py is located.
    package_share = get_package_share_directory('robco_validator')
    servo_launch_path = os.path.join(package_share, 'launch', 'servo_launch_params.py')
    servo_config_file_path = os.path.join(package_share, 'config', 'servo_config_joing_sim.yaml')
    
    # Declare launch arguments
    joint_limits_param = DeclareLaunchArgument(
        'joint_limits_param',
        default_value=os.path.join(package_share, 'config', 'joint_limits.yaml'),
        description='Path to the joint limits configuration file'
    )

    collision_objects_param = DeclareLaunchArgument(
        'collision_objects_param',
        default_value=os.path.join(package_share, 'config', 'collision_objects.yaml'),
        description='Path to the collision objects configuration file'
    )

    # Get the launch configuration values
    joint_limits = LaunchConfiguration('joint_limits_param')
    collision_objects = LaunchConfiguration('collision_objects_param')

    
    servo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(servo_launch_path),
            # Override the launch arguments here.
            launch_arguments={
                'servo_config_file_path': servo_config_file_path,
                'collision_objects_file_path': collision_objects,
                'joint_limits_file_path': joint_limits
            }.items()
        )

    return launch.LaunchDescription([
        servo_launch,
        joint_limits_param,
        collision_objects_param
    ])
