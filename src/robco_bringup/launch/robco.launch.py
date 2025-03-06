#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Whether to run in simulation (Gazebo, sim time, etc.).'
    )
    use_sim = LaunchConfiguration('use_sim')

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Whether to start RViz.'
    )
    start_rviz = LaunchConfiguration('start_rviz')

    control_method_arg = DeclareLaunchArgument(
        'control_method',
        default_value='velocity',
        description='Control input type (e.g., velocity, gamepad, etc.).'
    )
    control_method = LaunchConfiguration('control_method')

    collision_object_path_arg = DeclareLaunchArgument(
        'collision_object_path',
        default_value='',
        description='Path to the collision objects configuration file.'
    )
    collision_object_path = LaunchConfiguration('collision_object_path')

    joint_limits_path_arg = DeclareLaunchArgument(
        'joint_limits_path',
        default_value='',
        description='Path to the joint limits configuration file.'
    )
    joint_limits_path = LaunchConfiguration('joint_limits_path')

    api_config_path_arg = DeclareLaunchArgument(
        'api_config_path',
        default_value='',
        description='Path to the API config file (UDS/TCP).'
    )
    api_config_path = LaunchConfiguration('api_config_path')
    
    # Launch rviz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('robco_bringup'), 'config', 'rviz_config.rviz'])],
        condition=IfCondition(start_rviz)
    )
    
    ros2_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robco_description"), "/launch/ros2_controllers.launch.py"]
        ), launch_arguments={"simulation": use_sim}.items()
    )
    
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robco_description"), "/launch/rsp.launch.py"]
        ), launch_arguments={"simulation": use_sim}.items()
    )
    
    ros2_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("core"), "/launch/server_launch.py"]
        ),
        launch_arguments={"config_file": api_config_path}.items()
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robco_description"), "/launch/gazebo.launch.py"]
        ),
        condition=IfCondition(use_sim)
    )
    
    joint_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robco_validator"), "/launch/joint_real.launch.py"]
        ),
        launch_arguments={"joint_limits_param": joint_limits_path, "collision_objects_param": collision_object_path}.items(),
        condition=IfCondition(PythonExpression(["'" , use_sim , "' != 'true' and '" , control_method , "' == 'velocity'"]))
    )
    
    joint_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robco_validator"), "/launch/joint_sim.launch.py"]
        ),
        launch_arguments={"joint_limits_param": joint_limits_path, "collision_objects_param": collision_object_path}.items(),
        condition=IfCondition(PythonExpression(["'" , use_sim , "' == 'true' and '" , control_method , "' == 'velocity'"]))
    )
    
    joy_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robco_validator"), "/launch/joy_real.launch.py"]
        ),
        launch_arguments={"joint_limits_param": joint_limits_path, "collision_objects_param": collision_object_path}.items(),
        condition=IfCondition(PythonExpression(["'" , use_sim , "' != 'true' and '" , control_method , "' == 'gamepad'"]))
    )    
    
    joy_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robco_validator"), "/launch/joy_sim.launch.py"]
        ),
        launch_arguments={"joint_limits_param": joint_limits_path, "collision_objects_param": collision_object_path}.items(),
        condition=IfCondition(PythonExpression(["'" , use_sim , "' == 'true' and '" , control_method , "' == 'gamepad'"]))
    )
    
    # Return a LaunchDescription containing only argument declarations for now
    return LaunchDescription([
        use_sim_arg,
        start_rviz_arg,
        control_method_arg,
        collision_object_path_arg,
        joint_limits_path_arg,
        api_config_path_arg,
        TimerAction(
            period=5.0,
            actions=[                
                joint_real,
                joint_sim,
                joy_real,
                joy_sim,
                node_rviz
            ]
        ),
        ros2_controllers,
        robot_state_publisher,
        gazebo,
        ros2_api
    ])
