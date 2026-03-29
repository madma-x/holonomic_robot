#!/usr/bin/env python3
"""Launch file for pick-place mission."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_application'),
            'config',
            'mission_controller.yaml'
        ]),
        description='Path to mission controller config file'
    )
    
    tasks_file_arg = DeclareLaunchArgument(
        'tasks_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_application'),
            'config',
            'pick_place_tasks.yaml'
        ]),
        description='Path to pick-place tasks config file'
    )
    
    # Pick-place mission node
    pick_place_node = Node(
        package='robot_application',
        executable='pick_place_mission.py',
        name='pick_place_mission',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            LaunchConfiguration('tasks_file')
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        tasks_file_arg,
        pick_place_node
    ])
