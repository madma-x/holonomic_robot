from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map',
        default=PathJoinSubstitution([
            FindPackageShare('holonomic_robot_bringup'),
            'maps',
            'polygon_map.yaml'
        ])
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file to load into map_server'
    )
    declare_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Map server node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file
        }]
    )

    # Lifecycle manager to bring map_server to active
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    return LaunchDescription([
        declare_map,
        declare_sim,
        map_server,
        lifecycle_mgr
    ])
