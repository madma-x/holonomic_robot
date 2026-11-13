"""Adaptive match strategy - brings up Nav2 (belief-fed costmap) + strategy_node.

  ros2 launch table_sim sim.launch.py goal_driver:=false nav2:=false
  ros2 launch adaptive_strategy strategy.launch.py

Launches table_sim's own nav2_sim.launch.py (the real robot's
PurePursuitController + custom_objects_layer costmap plugin) with its
custom_objects_topic pointed at /strategy/belief_objects instead of
sim_node's ground-truth /custom_objects, so the costmap reflects only what
this strategy actually believes, not sim-only omniscience - see
strategy_node.py's docstring. Don't also pass nav2:=true to sim.launch.py,
or Nav2 ends up launched twice (duplicate node names).

Arguments
  team          yellow | blue (must match sim.launch.py's team)
  control_rate  Hz, how often the brain's step() runs
  field_file    defaults to table_sim's own field_2027.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    table_sim_share = get_package_share_directory('table_sim')
    strategy_share = get_package_share_directory('adaptive_strategy')
    field_default = os.path.join(table_sim_share, 'config', 'field_2027.yaml')
    strategy_params_default = os.path.join(strategy_share, 'config', 'strategy.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('team', default_value='yellow'),
        DeclareLaunchArgument('control_rate', default_value='20.0'),
        DeclareLaunchArgument('field_file', default_value=field_default),
        DeclareLaunchArgument('strategy_params_file', default_value=strategy_params_default),
        DeclareLaunchArgument('brain', default_value='planner'),   # planner | rules
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(table_sim_share, 'launch', 'nav2_sim.launch.py')),
            launch_arguments={'custom_objects_topic': '/strategy/belief_objects'}.items()),
        Node(package='adaptive_strategy', executable='strategy_node', name='strategy_node',
             output='screen', parameters=[{
                 'team': LaunchConfiguration('team'),
                 'control_rate': LaunchConfiguration('control_rate'),
                 'field_file': LaunchConfiguration('field_file'),
                 'strategy_params_file': LaunchConfiguration('strategy_params_file'),
                 'brain': LaunchConfiguration('brain'),
                 'use_sim_time': True,
             }]),
    ])
