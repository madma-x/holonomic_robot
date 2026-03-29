#!/usr/bin/env python3
"""Launch file for patrol mission."""

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
    
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_application'),
            'config',
            'patrol_waypoints.yaml'
        ]),
        description='Path to patrol waypoints config file'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Automatically start mission on launch'
    )
    
    # Patrol mission node
    patrol_node = Node(
        package='robot_application',
        executable='patrol_mission.py',
        name='patrol_mission',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            LaunchConfiguration('waypoints_file')
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        waypoints_file_arg,
        autostart_arg,
        patrol_node
    ])
