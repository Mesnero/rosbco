from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = ParameterValue( 
        Command(
            ['xacro ',             
                 PathJoinSubstitution([FindPackageShare('robco_description'), 'urdf', 'robco.xacro']),
                ' use_gazebo:=false'
            ])
        ,value_type=str)

    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    


    return LaunchDescription([node_robot_state_publisher])