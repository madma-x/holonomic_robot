from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('i2c_node'),
        'config',
        'i2c_config.yaml',
    )

    return LaunchDescription([
        Node(
            package='i2c_node',
            executable='i2c_node',
            name='i2c_node',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        )
    ])
