import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('perception')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 1) YOLO: /camera/image_raw -> /person_detections
        #    cwd=<share>/models so the node's relative YOLO("yolo11m.pt") resolves
        Node(
            package='perception', executable='yolo', name='camera_yolo_node',
            output='screen',
            cwd=os.path.join(share, 'models'),
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 2) Localizer: /person_detections + /camera/camera_info + TF -> /localized_person
        Node(
            package='perception', executable='localizer', name='survivor_localizer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 3) Confirmer: /localized_person -> /confirmed_survivors
        Node(
            package='perception', executable='confirmer',
            name='survivor_confirmation_node', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])