import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('lidar_processing'), 'config')
    params_file = os.path.join(config_dir, 'lidar_tilt_stabilizer.yaml')

    return LaunchDescription([
        Node(
            package='lidar_processing',
            executable='lidar_tilt_stabilizer_node',
            name='lidar_tilt_stabilizer',
            parameters=[params_file],
            output='screen',
        ),
    ])
