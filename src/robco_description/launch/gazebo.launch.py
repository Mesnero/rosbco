from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Path to the world file
    world_file = PathJoinSubstitution(
        [FindPackageShare("robco_description"), "worlds", "my_world.sdf"]
    )
    
    params = PathJoinSubstitution(
        [FindPackageShare("robco_description"), "gazebo", "gazebo_params.yaml"]
    )

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"]
        ),
        launch_arguments={'world': world_file, 'params_file': params}.items()
    )

    gz_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "robco"
        ]
    )

    return LaunchDescription([gazebo, gz_spawn_entity])