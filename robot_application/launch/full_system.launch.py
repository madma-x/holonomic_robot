#!/usr/bin/env python3
"""Launch all application nodes for full autonomous operation."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # argument for nav2 parameter file – defaults to robot_application's copy
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_application'),
            'config',
            'nav2_params_holonomic.yaml'
        ]),
        description='Full path to a YAML file with Nav2 parameters'
    )

    # map file argument (forwarded to map_server_launch)
    map_file = LaunchConfiguration('map')
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('holonomic_robot_bringup'),
            'maps',
            'polygon_map.yaml'
        ]),
        description='Full path to a YAML file for the map_server'
    )

    # Setup CAN interface with 1Mbps bitrate
    setup_can = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '1000000'],
        output='screen',
        shell=False
    )

    # Include CAN bridge launch
    can_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('can_interface'),
                'launch',
                'can_bridge.launch.py'
            ])
        ])
    )

    # Include map server + lifecycle manager (from this package)
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_application'),
                'launch',
                'map_server_launch.py'
            ])
        ]),
        launch_arguments={'map': map_file, 'use_sim_time': 'false'}.items()
    )

    # Include Nav2 bringup, pass the parameter file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('holonomic_robot_bringup'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={'params_file': nav2_params_file}.items()
    )

    # Include RPLIDAR sllidar_ros2 launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py'
            ])
        ])
    )

    # Lightweight lidar filter node (C++), runs before Nav2 so /scan_filtered exists
    lidar_filter_node = Node(
        package='lidar_filter',
        executable='scan_filter',
        name='lidar_scan_filter',
        output='screen',
        parameters=[{
            'scan_in_topic': '/scan',
            'scan_out_topic': '/scan_filtered',
            'pose_topic': '/odom',
            'map_topic': '/map'
        }]
    )

    # Game state manager node
    game_state_manager_node = Node(
        package='robot_application',
        executable='game_state_manager',
        name='game_state_manager',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_application'),
                'config',
                'game_state.yaml'
            ])
        ]
    )

    # Task planner node
    task_planner_node = Node(
        package='robot_application',
        executable='task_planner',
        name='task_planner',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_application'),
                'config',
                'task_planner.yaml'
            ])
        ]
    )

    # RViz2 for real-time visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('robot_application'),
            'config',
            'robot.rviz'
        ])],
        output='screen'
    )

    return LaunchDescription([
        declare_nav2_params,
        declare_map,
        setup_can,
        can_bridge_launch,
        rplidar_launch,
        lidar_filter_node,
        map_server_launch,
        nav2_launch,
        game_state_manager_node,
        task_planner_node,
        rviz_node
    ])
