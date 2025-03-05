from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    # Resolve the simulation flag at runtime.
    simulation_value = LaunchConfiguration('simulation').perform(context)
    
    # Build the robot description command with the resolved simulation value.
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([FindPackageShare('robco_description'), 'urdf', 'robco.xacro']),
            ' use_gazebo:=', TextSubstitution(text=simulation_value)
        ]),
        value_type=str
    )
    
    arguments = {
        'robot_description': robot_description_content,
        'use_sim_time': simulation_value == 'true'
    }
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[arguments],
    )
    
    return [node_robot_state_publisher]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation',
            default_value='false',
            description='Simulation mode'
        ),
        OpaqueFunction(function=launch_setup)
    ])
