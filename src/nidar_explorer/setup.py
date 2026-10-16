import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'nidar_explorer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NIDAR team',
    maintainer_email='team@example.com',
    description='DFS frontier explorer + Nav2 config for indoor maze exploration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dfs_explorer = nidar_explorer.dfs_explorer:main',
        ],
    },
)
