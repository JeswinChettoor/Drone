from setuptools import find_packages
from setuptools import setup

setup(
    name='nidar_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('nidar_msgs', 'nidar_msgs.*')),
)
