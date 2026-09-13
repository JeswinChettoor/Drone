from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    bridge_config = LaunchConfiguration('bridge_config')

    default_bridge_config_path = PathJoinSubstitution([
        FindPackageShare('gz_ros_bridge'),
        'config',
        'bridge.yaml'
    ])

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Sync bridge to simulation clock (/clock)'
    )

    declare_bridge_config = DeclareLaunchArgument(
        'bridge_config',
        default_value=default_bridge_config_path,
        description='Path to ros_gz_bridge parameter YAML config'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_bridge_config,
        bridge_node,
    ])
