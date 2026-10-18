from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    def make_static_tf(name, child_frame, x, y, z, roll, pitch, yaw):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            arguments=[
                '--x', str(x),
                '--y', str(y),
                '--z', str(z),
                '--roll', str(roll),
                '--pitch', str(pitch),
                '--yaw', str(yaw),
                '--frame-id', 'base_link',
                '--child-frame-id', child_frame,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        make_static_tf(
            name='base_to_camera_tf',
            child_frame='iris/front_camera_link/front_down_camera',
            x=0.12, y=0.0, z=-0.05,
            roll=0.0, pitch=0.7854, yaw=0.0,
        ),
        make_static_tf(
            name='base_to_lidar_tf',
            child_frame='iris/lidar_link/lidar_2d',
            x=0.0, y=0.0, z=0.0,
            roll=0.0, pitch=0.0, yaw=0.0,
        ),
    ])