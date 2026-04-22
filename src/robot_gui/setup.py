from setuptools import setup
import os
from glob import glob

package_name = 'robot_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'PySide6'],
    zip_safe=True,
    maintainer='ovoleur',
    maintainer_email='maxmorlion2024@gmail.com',
    description='Vertical touchscreen GUI for match control and diagnostics.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_gui = robot_gui.gui_node:main',
        ],
    },
)
