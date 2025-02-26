from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    
    robot_description_content = ParameterValue( 
        Command(
            ['xacro ',             
                 PathJoinSubstitution([FindPackageShare('robot_description_test'), 'urdf', 'panda.xacro']),
            ])
        ,value_type=str)
    

    arguments = {'robot_description': robot_description_content, 'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[arguments],
    ) 

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
            "-entity", "panda"
        ]
    )
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        parameters=[{"use_sim_time": True}]
    ) 
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}]
    )
    
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True}]
    )
    
    delay_controller_spawning = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=gz_spawn_entity,
        on_exit=[
            TimerAction(
                period=3.0,  # ✅ Wait 3 seconds to ensure Gazebo loads
                actions=[
                    joint_state_broadcaster_spawner,
                    robot_controller_spawner
                ],
            )
        ],
    )
)
    nodes = [
        node_robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        joint_state_publisher,
        delay_controller_spawning
    ]

    return LaunchDescription(nodes)    