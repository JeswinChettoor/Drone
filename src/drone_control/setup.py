import os
from glob import glob
from setuptools import setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Installs every *.launch.py under launch/ so future launch
        # files are picked up automatically without editing this file.
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Installs every *.yaml under config/ the same way.
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Launch files and configs for drone control/navigation nodes',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
