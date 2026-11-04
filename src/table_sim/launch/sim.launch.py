"""Eurobot 2027 table simulation + RViz.

  ros2 launch table_sim sim.launch.py                      # you drive (click 2D Goal Pose)
  ros2 launch table_sim sim.launch.py our_robot:=scripted:mirror opponent:=raider
  ros2 launch table_sim sim.launch.py team:=blue opponent:=teleop

Arguments
  team         yellow | blue                        (our colour)
  opponent     mirror | greedy | raider | center_first | random | blocker | idle | teleop
  our_robot    cmd_vel (driven by /cmd_vel) | scripted:<behaviour> (to watch a full match)
  goal_driver  true: click "2D Goal Pose" in RViz to move our robot (cmd_vel mode only)
  opponent_teleop  true: spawn an xterm (VNC desktop) running teleop_twist_keyboard
                   remapped to /opponent/cmd_vel - only does anything if opponent:=teleop
  rviz         true | false
  panel        true: separate match panel window (score, stacks, zones, pick/drop buttons)
  rtf          real-time factor (2.0 = twice as fast)
  auto_start   true: the 85 s match starts 3 s after launch, else call /sim/start
  nav2         true: Nav2 (planner/controller/bt_navigator) + the dynamic box
               costmap, see nav2_sim.launch.py. odom_sim (always on) supplies
               /odom, odom->base_footprint and map->odom (perfect, no noise).
  record       true: record the match to a rosbag (see bag_dir), for replaying
               step by step afterwards with `ros2 run table_sim bag2frames`
               + `replay_match` - the only way to check the heading asked for
               at every pick/drop was the intended one and was actually hit.
  bag_dir      where to write it (default /tmp/table_sim_bags/<timestamp>)
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# What a match replay needs. /sim/true_odom rather than /tf for the pose:
# odom_sim future-dates its transforms by transform_tolerance_sec (0.2s),
# which is awkward to replay faithfully. /strategy/{status,goal} carry the
# requested heading at each pick/drop - the whole point of recording.
RECORD_TOPICS = [
    '/sim/score', '/sim/true_odom', '/opponent/odom', '/sim/reset_event',
    '/strategy/status', '/strategy/goal', '/cmd_vel',
    '/scan', '/camera/detections',
    '/game/match_active', '/game/time_remaining',
    '/clock', '/tf', '/tf_static',
]


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
                 # ROS-side tick/publish rate, decoupled from physics.dt (still
                 # 0.01s internally via World.step's own substepping). Lower
                 # than the physics step because rclpy's executor overhead
                 # scales with (entity count x wakeup rate), and table_sim has
                 # a lot of entities (many pubs/subs, 3 services, 1 action
                 # server) - 50Hz here was pegging a CPU core under any
                 # steady /cmd_vel stream.
                 'rate': 10.0,
             }]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        # Realistic sensing pipeline for "us" (always on): odom_sim republishes
        # sim_node's ground truth as /odom + odom->base_footprint TF, and
        # also broadcasts map->odom (identity, since /odom is a perfect copy
        # of ground truth - map and odom always coincide). Event-driven off
        # /sim/true_odom, so it runs at sim_node's own 'rate' param above.
        # sim_node itself no longer publishes /odom or either TF directly.
        Node(package='table_sim', executable='odom_sim', name='odom_sim', output='screen',
             parameters=[{'sim_file': lc('sim_file'), 'transform_tolerance_sec': 0.2}]),
    ]
    if our_robot == 'cmd_vel' and lc('goal_driver').lower() == 'true':
        nodes.append(Node(package='table_sim', executable='goal_driver', name='goal_driver',
                          output='screen', parameters=[{'use_sim_time': True}]))
    if lc('opponent_teleop').lower() == 'true':
        # teleop_twist_keyboard needs a real terminal for raw keypresses -
        # an xterm on the VNC desktop (DISPLAY, already set by whoever
        # launched this) gives it one without a second docker exec -it.
        nodes.append(ExecuteProcess(
            cmd=['xterm', '-T', 'opponent teleop (/opponent/cmd_vel)', '-e', 'bash', '-lc',
                'source /opt/ros/jazzy/setup.bash && ros2 run teleop_twist_keyboard '
                'teleop_twist_keyboard --ros-args -r /cmd_vel:=/opponent/cmd_vel'],
            output='screen'))
    if lc('panel').lower() == 'true':
        nodes.append(Node(package='table_sim', executable='sim_panel', name='sim_panel', output='screen'))
    if lc('rviz').lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                          arguments=['-d', os.path.join(share, 'rviz', 'table_sim.rviz')],
                          parameters=[{'use_sim_time': True}]))
    if lc('nav2').lower() == 'true':
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'nav2_sim.launch.py'))))
    if lc('record').lower() == 'true':
        bag = lc('bag_dir') or os.path.join('/tmp', 'table_sim_bags',
                                            time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(os.path.dirname(bag), exist_ok=True)
        nodes.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag, *RECORD_TOPICS],
            output='screen'))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('table_sim')
    return LaunchDescription([
        DeclareLaunchArgument('team', default_value='yellow'),
        DeclareLaunchArgument('opponent', default_value='mirror'),
        DeclareLaunchArgument('our_robot', default_value='cmd_vel'),
        DeclareLaunchArgument('goal_driver', default_value='true'),
        DeclareLaunchArgument('opponent_teleop', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('panel', default_value='true'),
        DeclareLaunchArgument('rtf', default_value='1.0'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('nav2', default_value='false'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_dir', default_value=''),
        DeclareLaunchArgument('field_file', default_value=os.path.join(share, 'config', 'field_2027.yaml')),
        DeclareLaunchArgument('sim_file', default_value=os.path.join(share, 'config', 'sim.yaml')),
        OpaqueFunction(function=_setup),
    ])
