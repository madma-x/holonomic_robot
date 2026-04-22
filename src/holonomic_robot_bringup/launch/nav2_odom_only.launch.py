#!/usr/bin/env python3
"""
Odom-only Nav2 launch file.
No lidar, no AMCL — localization is a static map→odom TF (robot starts at origin).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('holonomic_robot_bringup')
    description_dir = get_package_share_directory('holonomic_robot_description')

    urdf_file  = os.path.join(description_dir, 'urdf', 'holonomic_robot.urdf')
    rviz_config = os.path.join(description_dir, 'rviz', 'robot_view.rviz')
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params_odom_only.yaml')
    map_file    = os.path.join(bringup_dir, 'maps', 'rectangle_2x4_map.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ── Robot model ──────────────────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='holonomic_robot_bringup',
            executable='static_joint_publisher',
            name='static_joint_publisher',
            output='screen'
        ),

        # ── Localization: static map→odom (robot starts at map origin) ───────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # ── Map server + lifecycle manager ───────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file
            }]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        # ── Nav2 navigation nodes (no collision_monitor) ──────────────────────
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params]
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
            ]
        ),

        # ── Collision monitor ─────────────────────────────────────────────────
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[nav2_params]
        ),

        # Lifecycle manager for navigation
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    #'collision_monitor',
                ]
            }]
        ),

        # ── SL Lidar C1 ──────────────────────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 460800,
                'frame_id': 'base_scan',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen'
        ),

        # ── CAN bridge (real robot) ───────────────────────────────────────────
        Node(
            package='can_interface',
            executable='can_node',
            name='can_node',
            output='screen'
        ),

        # ── RViz ─────────────────────────────────────────────────────────────
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config],
        #     output='screen'
        # ),
    ])
