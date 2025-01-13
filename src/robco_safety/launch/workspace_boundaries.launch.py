from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = get_package_share_directory('robco_safety')
    workspace_params = os.path.join(config_path, 'config', 'workspace_bounds.yaml')

    return LaunchDescription([
        Node(
            package='robco_safety',
            executable='workspace_boundaries',
            name='workspace_boundaries',
            output='screen',
            parameters=[workspace_params]
        )
    ])
