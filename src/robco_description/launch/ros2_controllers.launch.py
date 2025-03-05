from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Whether to run in simulation (Gazebo, sim time, etc.).'
    )
    use_sim = LaunchConfiguration('use_sim')
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_velocity_controller"],
        parameters=[{"use_sim_time": use_sim}]
    ) 
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": use_sim}]
    )
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robco_description"),
            "ros2_control",
            "ros2_controllers.yaml",
        ]
    )    
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        condition=UnlessCondition(use_sim)
    )
    
    return LaunchDescription([use_sim_arg, joint_state_broadcaster_spawner, robot_controller_spawner, control_node])