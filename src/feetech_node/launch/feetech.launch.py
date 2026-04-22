from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('feetech_node'),
        'config',
        'feetech_config.yaml',
    )

    return LaunchDescription([
        Node(
            package='feetech_node',
            executable='feetech_node',
            name='feetech_node',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        )
    ])
