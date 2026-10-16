import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    maze_explorer_pkg = get_package_share_directory('maze_explorer')
    explorer_params = os.path.join(maze_explorer_pkg, 'config', 'explorer_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        SetParameter(name='use_sim_time', value=use_sim_time),
        Node(
            package='maze_explorer',
            executable='explorer_node',
            name='explorer_node',
            output='screen',
            parameters=[explorer_params, {'use_sim_time': use_sim_time}]
        ),
    ])
