from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # REMOVED: config and launch entries
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='jeswin.j.chettoor@gmail.com',
    description='NIDAR Person Detection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo = perception.camera_yolo_node:main',
            'localizer = perception.survivor_localizer_node:main',
            'confirmer = perception.survivor_confirmation_node:main'
        ],
    },
)