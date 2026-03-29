from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('aruco_localization'),
        'config', 'localization_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='aruco_localization',
            executable='localization_node',
            name='aruco_localization_node',
            output='screen',
            parameters=[config],
        )
    ])
