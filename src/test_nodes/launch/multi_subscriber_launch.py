#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node1 = Node(
            package='multi_subscriber',
            executable='multi_subscriber_node',
            name='multi_subscriber_node',
            output='screen'
        )
    
    node2 = Node(
            package='multi_subscriber',
            executable='multi_publisher_node',
            name='multi_publisher_node',
            output='screen'
        )
    return LaunchDescription([
       node1, 
       node2 
    ])
