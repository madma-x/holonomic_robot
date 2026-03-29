#!/usr/bin/env python3
"""Launch strategic planner system."""

from launch import LaunchDescription
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
        parameters=[{
            'replan_interval_sec': LaunchConfiguration('replan_interval_sec'),
            'enable_task_interruption': True,
            'min_utility_threshold': 0.5,
            'base_location_x': 0.0,
            'base_location_y': 0.0
        }]
    )
    
    return LaunchDescription([
        match_duration_arg,
        auto_start_arg,
        replan_interval_arg,
        game_state_node,
        task_planner_node
    ])
