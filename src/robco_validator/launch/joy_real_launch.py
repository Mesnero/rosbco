import os
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory where your servo_launch.py is located.
    package_share = get_package_share_directory('robco_validator')
    servo_launch_path = os.path.join(package_share, 'launch', 'servo_launch_params.py')
    servo_config_file_path = os.path.join(package_share, 'config', 'servo_config_cartesian_real.yaml')

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(servo_launch_path),
            # Override the launch arguments here.
            launch_arguments={
                'use_joy_republisher': 'true',
                'servo_config_file_path': servo_config_file_path
            }.items()
        )
    ])
