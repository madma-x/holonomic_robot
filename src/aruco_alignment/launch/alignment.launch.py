from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('aruco_alignment'),
        'config'
    )
    config_file = os.path.join(config_dir, 'alignment_config.yaml')

    alignment_node = Node(
        package='aruco_alignment',
        executable='alignment_node',
        name='alignment_node',
        parameters=[
            config_file,
            {
                'verbose_logging': LaunchConfiguration('verbose_logging'),
            },
        ],
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level',
            ['alignment_node:=', LaunchConfiguration('log_level')],
        ],
        remappings=[
            ('/odom', LaunchConfiguration('odom_topic')),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry topic'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level for alignment_node (debug, info, warn, error)'
        ),
        DeclareLaunchArgument(
            'verbose_logging',
            default_value='false',
            description='Enable detailed alignment runtime logs'
        ),
        alignment_node
    ])
