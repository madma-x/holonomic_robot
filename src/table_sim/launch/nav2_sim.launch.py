"""Nav2 (no AMCL) for table_sim: map_server + controller/planner/behavior/
bt_navigator/waypoint_follower, using the real robot's own pure-pursuit
controller plugin (motion_controller::PurePursuitController) and dynamic
obstacle costmap layer (custom_objects_layer), fed by sim_node's
/custom_objects by default. odom_sim (started by sim.launch.py) supplies
map -> odom, so there's no static map_odom_tf node here.

  ros2 launch table_sim sim.launch.py nav2:=true
  ros2 launch table_sim nav2_sim.launch.py custom_objects_topic:=/strategy/belief_objects

`custom_objects_topic` lets a belief-based obstacle source (adaptive_strategy's
strategy_node, see its launch/strategy.launch.py) feed the costmap instead of
sim_node's ground truth - custom_objects_layer's subscribed topic is just a
plugin parameter, so no code changes there, only which yaml value is loaded.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    table_sim_share = get_package_share_directory('table_sim')
    bt_nav_share = get_package_share_directory('nav2_bt_navigator')

    params_path = os.path.join(table_sim_share, 'config', 'nav2_params_sim.yaml')
    with open(params_path) as f:
        params = yaml.safe_load(f)
    params['global_costmap']['global_costmap']['ros__parameters'] \
        ['custom_objects_layer']['topic'] = lc('custom_objects_topic')

    # The real robot's copied "no_fallback" BT XMLs instant-aborted every
    # goal here with no planner/controller activity logged in between -
    # likely a BTCPP_format/node-registration mismatch against whatever
    # Nav2 version they were authored against vs. this freshly apt-installed
    # one. Using Nav2's own bundled default BTs instead - guaranteed to
    # match the installed nav2_bt_navigator exactly, and they add replanning
    # + recovery (spin/backup/wait, already configured in behavior_server)
    # which is strictly more capable than "no_fallback" anyway.
    bt = params['bt_navigator']['ros__parameters']
    bt['default_nav_to_pose_bt_xml'] = os.path.join(
        bt_nav_share, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')
    bt['default_nav_through_poses_bt_xml'] = os.path.join(
        bt_nav_share, 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery.xml')
    params['map_server']['ros__parameters']['yaml_filename'] = os.path.join(
        table_sim_share, 'maps', 'table_2027.yaml')

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='_nav2_params_sim.yaml', delete=False)
    yaml.safe_dump(params, tmp)
    tmp.close()
    nav2_params = tmp.name

    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen', parameters=[nav2_params]),

        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[nav2_params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=[nav2_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', parameters=[nav2_params]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('custom_objects_topic', default_value='/custom_objects'),
        OpaqueFunction(function=_setup),
    ])
