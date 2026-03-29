#!/usr/bin/env python3
"""
Main launch file for holonomic robot RVIZ2 simulation with keyboard teleop.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    description_pkg_dir = get_package_share_directory('holonomic_robot_description')
    
    # Path to URDF file
    urdf_file = os.path.join(description_pkg_dir, 'urdf', 'holonomic_robot.urdf')
    
    # Path to RVIZ config
    rviz_config_file = os.path.join(description_pkg_dir, 'rviz', 'robot_view.rviz')
    
    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Robot State Publisher - publishes TF for robot links
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Static Joint Publisher - publishes joint states for wheels
    static_joint_publisher_node = Node(
        package='holonomic_robot_bringup',
        executable='static_joint_publisher',
        name='static_joint_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Fake Optical Flow Odometry Node
    fake_optical_flow_node = Node(
        package='holonomic_robot_bringup',
        executable='fake_optical_flow',
        name='fake_optical_flow',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Fake Lidar Publisher Node
    fake_lidar_node = Node(
        package='holonomic_robot_bringup',
        executable='fake_lidar_publisher',
        name='fake_lidar_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    # Environment Markers Node
    environment_markers_node = Node(
        package='holonomic_robot_bringup',
        executable='environment_markers',
        name='environment_markers',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

        # Floor image publisher (shows your floor markings)
    floor_image_publisher_node = Node(
        package='holonomic_robot_bringup',
        executable='floor_image_publisher',
        name='floor_image_publisher',
        output='screen'
    )
    
    # RVIZ2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )
    
    # Teleop Twist Keyboard
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal window
        parameters=[{'use_sim_time': False}]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        static_joint_publisher_node,
        fake_optical_flow_node,
        fake_lidar_node,
        environment_markers_node,
        floor_image_publisher_node,
        rviz_node,
        teleop_node,
    ])
