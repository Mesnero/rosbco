from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    
    config_file = PathJoinSubstitution([FindPackageShare('core') , 'config', 'api_config.yaml'])
    
    node_server = Node(
        package='core',
        executable='ros2_api_node',
        output='screen',
        arguments=['--config_file', config_file]
    )

    return LaunchDescription([node_server])