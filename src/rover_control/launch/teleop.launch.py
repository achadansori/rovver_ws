#!/usr/bin/env python3
# Copyright 2026 User
# SPDX-License-Identifier: MIT

"""
Launch file for Rover Teleop Control
Launches joy_node and joystick_teleop together
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('rover_control')
    
    # Declare launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'joystick_params.yaml'),
        description='Path to joystick parameters file'
    )
    
    # Joy node - reads joystick and publishes to /joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        remappings=[
            ('joy', 'joy'),
        ],
    )
    
    # Joystick teleop node - converts joy to cmd_vel
    teleop_node = Node(
        package='rover_control',
        executable='joystick_teleop',
        name='joystick_teleop',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
    )
    
    return LaunchDescription([
        joy_dev_arg,
        config_file_arg,
        joy_node,
        teleop_node,
    ])
