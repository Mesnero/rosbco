from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    
    config_file = PathJoinSubstitution([FindPackageShare('ros2_api') , 'config', 'api_config.yaml'])
    
    node_server = Node(
        package='ros2_api',
        executable='ros2_api_node',
        output='screen',
        parameters=[{
            'path_to_yaml': config_file
        }]
    )

    return LaunchDescription([node_server])