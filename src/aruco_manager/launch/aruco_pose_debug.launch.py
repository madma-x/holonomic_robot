from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic = LaunchConfiguration('camera_topic')
    tags_topic = LaunchConfiguration('tags_topic')
    pickability_topic = LaunchConfiguration('pickability_topic')
    show_window = LaunchConfiguration('show_window')

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('tags_topic', default_value='/findeeznuts/detected_tags'),
        DeclareLaunchArgument('pickability_topic', default_value='/cluster_pickability'),
        DeclareLaunchArgument('show_window', default_value='true'),

        Node(
            package='aruco_manager',
            executable='aruco_pose_debug_node.py',
            name='aruco_pose_debug',
            output='screen',
            parameters=[{
                'camera_topic': camera_topic,
                'tags_topic': tags_topic,
                'pickability_topic': pickability_topic,
                'show_window': show_window
            }],
        )
    ])
