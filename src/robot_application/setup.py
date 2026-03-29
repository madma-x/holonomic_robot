from setuptools import setup
import os
from glob import glob

package_name = 'robot_application'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
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
            'patrol_mission = robot_application.patrol_mission:main',
            'pick_place_mission = robot_application.pick_place_mission:main',
            'teleop_mission = robot_application.teleop_mission:main',
            'game_state_manager = robot_application.game_state_manager:main',
            'task_planner = robot_application.task_planner:main',
        ],
    },
)
