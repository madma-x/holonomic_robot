from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_actuators'),
        'config',
        'sequencer_config.yaml',
    )

    return LaunchDescription([
        Node(
            package='robot_actuators',
            executable='actuator_sequencer.py',
            name='actuator_sequencer',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        )
    ])