import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'pymavlink', 'tf_transformations'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@todo.todo',
    description='MAVLink TF bridges, scan levelling and SLAM bringup for NIDAR',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf3 = mapping.ekf3tobase:main',
            'stabilizer = mapping.scanstabilizer:main',
            'posxy = mapping.position_send:main',
            'velocity = mapping.vel2ap:main',
        ],
    },
)