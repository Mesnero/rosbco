from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    
    package_name = "robco_description"
    
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare(package_name), "/launch/rsp.launch.py"]
        ), launch_arguments={"simulation": "true"}.items()
    )

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"]
        ),
        launch_arguments={"verbose": "true"}.items()
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

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_velocity_controller"],
        parameters=[{"use_sim_time": True}]
    ) 
    
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True}]
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}]
    )
    
    delay_controller_spawning = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                TimerAction(
                    period=3.0, 
                    actions=[
                        joint_state_broadcaster_spawner,
                        robot_controller_spawner
                    ],
                )
            ]
        )   
    )

    nodes = [
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        joint_state_publisher,
        delay_controller_spawning
    ]

    return LaunchDescription(nodes)