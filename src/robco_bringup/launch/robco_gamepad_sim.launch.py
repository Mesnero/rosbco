#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    This launch file calls robco.launch.py with specific arguments for a velocity-control simulation.
    """

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    bringup_share_dir = get_package_share_directory('robco_bringup')
    collision_object_path = os.path.join(bringup_share_dir, 'config', 'collision_objects.yaml')
    joint_limits_path = os.path.join(bringup_share_dir, 'config', 'joint_limits.yaml')
    api_config_path = os.path.join(bringup_share_dir, 'config', 'api_config.yaml')

    launch_arguments = {
        'use_sim': 'true',                       
        'start_rviz': LaunchConfiguration('start_rviz'),
        'control_method': 'gamepad',           
        'collision_object_path': collision_object_path,
        'joint_limits_path': joint_limits_path,
        'api_config_path': api_config_path, 
    }

    include_robco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share_dir, 'launch', 'robco.launch.py')
        ),
        launch_arguments=launch_arguments.items()
    )

    ld = LaunchDescription([
        start_rviz_arg,
        include_robco_launch
    ])

    return ld
