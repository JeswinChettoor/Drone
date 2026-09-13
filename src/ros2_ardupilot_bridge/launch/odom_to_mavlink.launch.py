from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom_rf2o',
        description='Odometry topic to bridge into ArduPilot'
    )
    connection_arg = DeclareLaunchArgument(
        'connection_string', default_value='udpin:127.0.0.1:14551',
        description='pymavlink connection string to ArduPilot (SITL companion-computer link)'
    )

    return LaunchDescription([
        odom_topic_arg,
        connection_arg,

        # rf2o laser odometry (adjust to match your existing launch file)
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan_stabilized',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0,
            }],
        ),

        # ROS 2 -> ArduPilot bridge
        Node(
            package='ros2_ardupilot_bridge',
            executable='odom_to_mavlink_node',
            name='odom_to_mavlink',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'connection_string': LaunchConfiguration('connection_string'),
                'target_system': 1,
                'target_component': 1,
                'wait_for_heartbeat': True,
            }],
        ),
    ])
