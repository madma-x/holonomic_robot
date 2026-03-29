#!/usr/bin/env python3
"""
Unified launch file for holonomic robot with optional AMCL localization and Nav2 navigation.
Includes: robot model, sensors (lidar, optical flow), Nav2, RVIZ2, teleop
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('holonomic_robot_bringup')
    description_dir = get_package_share_directory('holonomic_robot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths
    urdf_file = os.path.join(description_dir, 'urdf', 'holonomic_robot.urdf')
    rviz_config = os.path.join(description_dir, 'rviz', 'robot_view.rviz')
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params_holonomic.yaml')
    amcl_params = os.path.join(bringup_dir, 'config', 'amcl_config.yaml')
    map_file = os.path.join(bringup_dir, 'maps', 'polygon_map.yaml')
    
    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_amcl = LaunchConfiguration('use_amcl')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'use_amcl',
            default_value='true',
            description='Enable AMCL localization'
        ),
        
        # Robot state publisher
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
        
        # Static joint publisher (for wheels)
        Node(
            package='holonomic_robot_bringup',
            executable='static_joint_publisher',
            name='static_joint_publisher',
            output='screen'
        ),
        
        # Fake optical flow (with drift)
        Node(
            package='holonomic_robot_bringup',
            executable='fake_optical_flow',
            name='fake_optical_flow',
            output='screen'
        ),
        
        # Fake lidar
        Node(
            package='holonomic_robot_bringup',
            executable='fake_lidar_publisher',
            name='fake_lidar_publisher',
            output='screen'
        ),
        
        # Environment markers
        Node(
            package='holonomic_robot_bringup',
            executable='environment_markers',
            name='environment_markers',
            output='screen'
        ),
        
        # Map server (always needed for Nav2)
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
        
        # AMCL (optional - only if use_amcl:=true)
        Node(
            condition=IfCondition(use_amcl),
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params]
        ),
        
        # ParticleCloud to PoseArray converter for RViz
        Node(
            condition=IfCondition(use_amcl),
            package='holonomic_robot_bringup',
            executable='particle_cloud_converter',
            name='particle_cloud_converter',
            output='screen'
        ),
        
        # Lifecycle manager for localization (map_server + optionally AMCL)
        Node(
            condition=IfCondition(use_amcl),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl']  # Include AMCL when enabled
            }]
        ),
        
        # Lifecycle manager for localization (map_server only, no AMCL)
        Node(
            condition=UnlessCondition(use_amcl),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server']  # Only map_server when AMCL disabled
            }]
        ),
        
        # Static transform map->odom (only when AMCL is disabled)
        Node(
            condition=UnlessCondition(use_amcl),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        # Nav2 navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
                'autostart': 'true'
            }.items()
        ),
        
        # Velocity Smoother (prevents stuttering!)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel')
            ]
        ),
        
        # Keyboard teleop
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e'
        ),
        
        # RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
