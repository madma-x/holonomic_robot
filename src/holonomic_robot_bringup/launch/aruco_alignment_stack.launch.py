from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    aruco_alignment_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('aruco_alignment'),
                'launch',
                'alignment.launch.py',
            ])
        )
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
            'show_window': 'true',
        }.items(),
    )

    return LaunchDescription([
        can_interface_launch,
        aruco_manager_launch,
        aruco_alignment_launch,
        aruco_pose_debug_launch,
    ])
