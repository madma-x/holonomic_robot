#!/usr/bin/env python3
"""
Full system launch: Nav2 odom-only base + robot application nodes.
Includes robot model, localization, Nav2 stack, sensors (lidar, CAN), and application logic.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('holonomic_robot_bringup')
    description_dir = get_package_share_directory('holonomic_robot_description')
    app_dir = get_package_share_directory('robot_application')

    # File paths
    urdf_file = os.path.join(description_dir, 'urdf', 'holonomic_robot.urdf')
    rviz_config = os.path.join(app_dir, 'config', 'robot.rviz')
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params_odom_only.yaml')
    map_file = os.path.join(bringup_dir, 'maps', 'rectangle_2x3_map.yaml')

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

        # ── Nav2 navigation nodes ────────────────────────────────────────────
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
                    'collision_monitor',
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

        # ── Robot Application Nodes ──────────────────────────────────────────

        # Game state manager node
        Node(
            package='robot_application',
            executable='game_state_manager.py',
            name='game_state_manager',
            output='screen',
            parameters=[
                os.path.join(app_dir, 'config', 'game_state.yaml')
            ]
        ),

        # Task planner node
        Node(
            package='robot_application',
            executable='task_planner.py',
            name='task_planner',
            output='screen',
            parameters=[
                os.path.join(app_dir, 'config', 'task_planner.yaml')
            ]
        ),

        # Mission executor node
        Node(
            package='robot_application',
            executable='mission_executor.py',
            name='mission_executor',
            output='screen',
            parameters=[
                os.path.join(app_dir, 'config', 'mission_controller.yaml')
            ]
        ),

        # ── RViz ─────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
