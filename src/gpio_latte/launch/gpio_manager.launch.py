from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gpio_latte'),
        'config',
        'gpio_config.yaml',
    )

    return LaunchDescription([
        Node(
            package='gpio_latte',
            executable='gpio_manager',
            name='gpio_manager',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        )
    ])
