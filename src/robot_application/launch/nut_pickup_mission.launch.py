#!/usr/bin/env python3
"""Launch file for the nut pickup mission."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_side_arg = DeclareLaunchArgument(
        'robot_side',
        default_value='yellow',
        description='Robot competition side: "yellow" or "blue"'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_application'),
            'config',
            'nut_pickup_config.yaml'
        ]),
        description='Path to nut pickup mission config file'
    )

    nut_pickup_node = Node(
        package='robot_application',
        executable='nut_pickup_mission',
        name='nut_pickup_mission',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'robot_side': LaunchConfiguration('robot_side')},
        ]
    )

    return LaunchDescription([
        robot_side_arg,
        config_file_arg,
        nut_pickup_node,
    ])
