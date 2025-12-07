from setuptools import find_packages, setup

package_name = 'robot_marker_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
