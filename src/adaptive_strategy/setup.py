import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'adaptive_strategy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='maxmorlion2024@gmail.com',
    description='Sensor-realistic adaptive match strategy for the Eurobot 2027 robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'strategy_node = adaptive_strategy.strategy_node:main',
        ],
    },
)
