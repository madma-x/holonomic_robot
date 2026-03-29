from setuptools import setup
import os
from glob import glob

package_name = 'holonomic_robot_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'images'), glob('images/*')),  # Add images

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='user@example.com',
    description='Launch files and scripts for holonomic robot simulation in RVIZ2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_lidar_publisher = holonomic_robot_bringup.fake_lidar_publisher:main',
            'environment_markers = holonomic_robot_bringup.environment_markers:main',
            'static_joint_publisher = holonomic_robot_bringup.static_joint_publisher:main',
            'fake_optical_flow = holonomic_robot_bringup.fake_optical_flow:main',
            'map_generator = holonomic_robot_bringup.map_generator:main',
            'floor_image_publisher = holonomic_robot_bringup.floor_image_publisher:main',
            'particle_cloud_converter = holonomic_robot_bringup.particle_cloud_converter:main',
        ],
    },
)
