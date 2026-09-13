import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lidar_processing'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pymavlink', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='2D LiDAR tilt stabilization using ArduPilot attitude',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_tilt_stabilizer_node = lidar_processing.lidar_tilt_stabilizer_node:main',
        ],
    },
)
