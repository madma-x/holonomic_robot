from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_gui'),
        'config',
        'gui_config.yaml',
    )

    return LaunchDescription([
        Node(
            package='robot_gui',
            executable='robot_gui',
            name='robot_gui',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        )
    ])
