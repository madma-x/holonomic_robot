import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # Launch the CAN bridge node
    can_bridge_node = Node(
        package='can_interface',
        executable='can_test_node',
        name='can_bridge',
        output='screen'
    )

    return LaunchDescription([
        can_bridge_node
    ])
