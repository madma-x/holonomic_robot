import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'table_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='maxmorlion2024@gmail.com',
    description='Eurobot 2027 table simulator with RViz display',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_node = table_sim.sim_node:main',
            'goal_driver = table_sim.goal_driver:main',
            'sim_panel = table_sim.sim_panel:main',
            'headless_match = table_sim.headless:main',
        ],
    },
)
