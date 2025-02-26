from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    
    simulation = LaunchConfiguration('simulation')
    # Get URDF via xacro
    robot_description_content = ParameterValue( 
        Command(
            ['xacro ',             
                 PathJoinSubstitution([FindPackageShare('robco_description'), 'urdf', 'robco.xacro']),
                ' use_gazebo:=true'
            ])
        ,value_type=str)
    

    arguments = {'robot_description': robot_description_content, 'use_sim_time': simulation}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[arguments],
    )    


    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation', 
            default_value='false', 
            description='Simulation mode'),
        node_robot_state_publisher])