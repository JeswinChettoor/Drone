import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('drone_control'), 'config')
    slam_config = os.path.join(config_dir, 'slam_toolbox.yaml')

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config]
        )
    ])
