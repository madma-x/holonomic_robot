from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_application'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py') + glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'images'), glob('images/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='user@example.com',
    description='High-level mission control and behavior coordination',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_state_manager = robot_application.game_state_manager:main',
            'task_planner = robot_application.task_planner:main',
            'mission_executor = robot_application.mission_executor:main',
            'odom_theta_monitor = robot_application.odom_theta_monitor:main',
            'custom_object_delete_test = robot_application.custom_object_delete_test:main',
            'environment_markers = robot_application.environment_markers:main',
            'custom_objects_initializer = robot_application.custom_objects_initializer:main',
            'static_joint_publisher = robot_application.static_joint_publisher:main',
            'odom_to_base_tf_broadcaster = robot_application.odom_to_base_tf_broadcaster:main',
            'map_generator = robot_application.map_generator:main',
            'floor_image_publisher = robot_application.floor_image_publisher:main',
            'particle_cloud_converter = robot_application.particle_cloud_converter:main',
        ],
    },
)
