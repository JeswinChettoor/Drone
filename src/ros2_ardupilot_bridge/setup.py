import os
from glob import glob
from setuptools import setup

package_name = 'ros2_ardupilot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='ROS 2 odometry -> ArduPilot MAVLink ODOMETRY bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_to_mavlink_node = ros2_ardupilot_bridge.odom_to_mavlink_node:main',
        ],
    },
)
