import os
from setuptools import find_packages, setup

package_name = 'maze_explorer'

def package_files(directories):
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    for directory in directories:
        if os.path.exists(directory):
            for (path, _, filenames) in os.walk(directory):
                if filenames:
                    target_path = os.path.join('share', package_name, path)
                    file_paths = [os.path.join(path, f) for f in filenames]
                    data_files.append((target_path, file_paths))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # Added 'config' to ensure your nav2_params.yaml and slam params are copied!
    data_files=package_files(['launch', 'urdf', 'worlds', 'meshes', 'config']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash',
    maintainer_email='yash@todo.todo',
    description='Autonomous maze explorer for search and rescue',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer_node = maze_explorer.explorer_node:main'
        ],
    },
)
