from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('safety_node'),
        'config',
        'safety_config.yaml',
    )

    return LaunchDescription([
        Node(
            package='safety_node',
            executable='safety_node',
            name='safety_node',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        )
    ])
