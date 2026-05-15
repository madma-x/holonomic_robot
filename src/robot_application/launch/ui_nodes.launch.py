#!/usr/bin/env python3
"""Launch only UI nodes: robot_gui + ArUco pose debug viewer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gui_dir = get_package_share_directory('robot_gui')
    gui_config = os.path.join(gui_dir, 'config', 'gui_config.yaml')
    rviz_config = os.path.join(
        get_package_share_directory('holonomic_robot_description'),
        'rviz',
        'robot_view.rviz',
    )
    camera_topic = LaunchConfiguration('camera_topic')
    tags_topic = LaunchConfiguration('tags_topic')
    pickability_topic = LaunchConfiguration('pickability_topic')

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('tags_topic', default_value='/findeeznuts/detected_tags'),
        DeclareLaunchArgument('pickability_topic', default_value='/cluster_pickability'),

        Node(
            package='robot_gui',
            executable='robot_gui',
            name='robot_gui',
            parameters=[gui_config],
            output='screen',
            emulate_tty=True,
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config],
        #     output='screen',
        # ),
        # Node(
        #     package='aruco_manager',
        #     executable='aruco_pose_debug_node.py',
        #     name='aruco_pose_debug_ui',
        #     output='screen',
        #     parameters=[{
        #         'camera_topic': camera_topic,
        #         'tags_topic': tags_topic,
        #         'pickability_topic': pickability_topic,
        #         'show_window': True,
        #     }],
    #    )

    ])
