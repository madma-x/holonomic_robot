"""Eurobot 2027 table simulation + RViz.

  ros2 launch table_sim sim.launch.py                      # you drive (click 2D Goal Pose)
  ros2 launch table_sim sim.launch.py our_robot:=scripted:mirror opponent:=raider
  ros2 launch table_sim sim.launch.py team:=blue opponent:=teleop

Arguments
  team         yellow | blue                        (our colour)
  opponent     mirror | greedy | raider | center_first | random | blocker | idle | teleop
  our_robot    cmd_vel (driven by /cmd_vel) | scripted:<behaviour> (to watch a full match)
  goal_driver  true: click "2D Goal Pose" in RViz to move our robot (cmd_vel mode only)
  rviz         true | false
  rtf          real-time factor (2.0 = twice as fast)
  auto_start   true: the 85 s match starts 3 s after launch, else call /sim/start
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    share = get_package_share_directory('table_sim')
    urdf = os.path.join(get_package_share_directory('holonomic_robot_description'),
                        'urdf', 'holonomic_robot.urdf')
    with open(urdf) as f:
        robot_description = f.read()
    our_robot = lc('our_robot')
    nodes = [
        Node(package='table_sim', executable='sim_node', name='table_sim', output='screen',
             parameters=[{
                 'team': lc('team'),
                 'opponent': lc('opponent'),
                 'our_robot': our_robot,
                 'real_time_factor': float(lc('rtf')),
                 'auto_start': lc('auto_start').lower() == 'true',
                 'field_file': lc('field_file'),
                 'sim_file': lc('sim_file'),
             }]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
    ]
    if our_robot == 'cmd_vel' and lc('goal_driver').lower() == 'true':
        nodes.append(Node(package='table_sim', executable='goal_driver', name='goal_driver',
                          output='screen', parameters=[{'use_sim_time': True}]))
    if lc('rviz').lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                          arguments=['-d', os.path.join(share, 'rviz', 'table_sim.rviz')],
                          parameters=[{'use_sim_time': True}]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('table_sim')
    return LaunchDescription([
        DeclareLaunchArgument('team', default_value='yellow'),
        DeclareLaunchArgument('opponent', default_value='mirror'),
        DeclareLaunchArgument('our_robot', default_value='cmd_vel'),
        DeclareLaunchArgument('goal_driver', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rtf', default_value='1.0'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('field_file', default_value=os.path.join(share, 'config', 'field_2027.yaml')),
        DeclareLaunchArgument('sim_file', default_value=os.path.join(share, 'config', 'sim.yaml')),
        OpaqueFunction(function=_setup),
    ])
