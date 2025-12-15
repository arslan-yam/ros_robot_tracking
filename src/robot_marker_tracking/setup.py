from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_marker_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/robot_marker_tracking']),
    ('share/robot_marker_tracking', ['package.xml']),
    (os.path.join('share', 'robot_marker_tracking', 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', 'robot_marker_tracking', 'rviz'), glob('rviz/*.rviz')),
    (os.path.join('share', 'robot_marker_tracking', 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='armaiamaletdinov',
    maintainer_email='armaiamaletdinov@edu.hse.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'marker_tracker_node = robot_marker_tracking.marker_tracker_node:main',
        ],
    },
)