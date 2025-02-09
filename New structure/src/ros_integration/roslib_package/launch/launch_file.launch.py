#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slm_ros',
            executable='slm_node',
            name='slm_node',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
