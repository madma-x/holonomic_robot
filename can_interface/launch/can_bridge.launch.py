import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Setup CAN interface with 1Mbps bitrate
    setup_can = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '1000000'],
        output='screen',
        shell=False
    )

    # Launch the CAN bridge node
    can_bridge_node = Node(
        package='can_interface',
        executable='can_node',
        name='can_bridge',
        output='screen'
    )

    return LaunchDescription([
        setup_can,
        can_bridge_node
    ])
