from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('can_interface'),
                'launch',
                'can_bridge.launch.py',
            ])
        )
    )

    aruco_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('aruco_manager'),
                'launch',
                'aruco_manager.launch.py',
            ])
        )
    )

    motion_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('motion_controller'),
                'launch',
                'motion_controller.launch.py',
            ])
        ),
        launch_arguments={
            'verbose_logging': LaunchConfiguration('mc_verbose_logging'),
            'log_level': LaunchConfiguration('mc_log_level'),
        }.items(),
    )

    aruco_pose_debug_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('aruco_manager'),
                'launch',
                'aruco_pose_debug.launch.py',
            ])
        ),
        launch_arguments={
            'show_window': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mc_verbose_logging',
            default_value='false',
            description='Enable detailed runtime logs for motion_controller_node'
        ),
        DeclareLaunchArgument(
            'mc_log_level',
            default_value='info',
            description='ROS log level for motion_controller_node (debug, info, warn, error)'
        ),
        can_interface_launch,
        aruco_manager_launch,
        motion_controller_launch,
        aruco_pose_debug_launch,
    ])
