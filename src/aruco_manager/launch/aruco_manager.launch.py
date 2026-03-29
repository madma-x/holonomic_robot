from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('aruco_manager'),
        'config', 'tag_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='aruco_manager',
            executable='tag_manager_node',
            name='tag_manager_node',
            output='screen',
            parameters=[config],
        )
    ])
