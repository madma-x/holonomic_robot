#!/usr/bin/env python3
"""Launch strategic planner system."""

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    match_duration_arg = DeclareLaunchArgument(
        'match_duration_sec',
        default_value='180.0',
        description='Match duration in seconds'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start match on launch'
    )
    
    replan_interval_arg = DeclareLaunchArgument(
        'replan_interval_sec',
        default_value='5.0',
        description='Replanning interval in seconds'
    )

    launch_alignment_arg = DeclareLaunchArgument(
        'launch_alignment',
        default_value='true',
        description='Launch the AlignToCluster service node'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic for alignment node'
    )

    mock_navigation_arg = DeclareLaunchArgument(
        'mock_navigation',
        default_value='false',
        description='Allow mission executor to simulate navigation success'
    )

    mock_actuators_arg = DeclareLaunchArgument(
        'mock_actuators',
        default_value='true',
        description='Allow mission executor to simulate actuator success'
    )
    
    # Game state manager node
    game_state_node = Node(
        package='robot_application',
        executable='game_state_manager.py',
        name='game_state_manager',
        output='screen',
        parameters=[{
            'match_duration_sec': LaunchConfiguration('match_duration_sec'),
            'auto_start': LaunchConfiguration('auto_start'),
            'early_phase_threshold': 60.0,
            'mid_phase_threshold': 30.0,
            'late_phase_threshold': 10.0
        }]
    )
    
    # Task planner node
    task_planner_node = Node(
        package='robot_application',
        executable='task_planner.py',
        name='task_planner',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_application'),
                'config',
                'task_planner.yaml'
            ]),
            {
                'replan_interval_sec': LaunchConfiguration('replan_interval_sec')
            }
        ]
    )

    mission_executor_node = Node(
        package='robot_application',
        executable='mission_executor.py',
        name='mission_executor',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_application'),
                'config',
                'mission_controller.yaml'
            ]),
            {
                'mock_navigation': LaunchConfiguration('mock_navigation'),
                'mock_actuators': LaunchConfiguration('mock_actuators')
            }
        ]
    )

    alignment_node = Node(
        condition=IfCondition(LaunchConfiguration('launch_alignment')),
        package='aruco_alignment',
        executable='alignment_node',
        name='alignment_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('aruco_alignment'),
                'config',
                'alignment_config.yaml'
            ]),
            {
                'odometry_topic': LaunchConfiguration('odom_topic')
            }
        ]
    )
    
    return LaunchDescription([
        match_duration_arg,
        auto_start_arg,
        replan_interval_arg,
        launch_alignment_arg,
        odom_topic_arg,
        mock_navigation_arg,
        mock_actuators_arg,
        game_state_node,
        task_planner_node,
        mission_executor_node,
        alignment_node,
    ])
