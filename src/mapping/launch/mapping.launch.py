import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_stabilizer = LaunchConfiguration('use_stabilizer')
    use_ekf3 = LaunchConfiguration('use_ekf3')
    use_rf2o = LaunchConfiguration('use_rf2o')
    use_slam = LaunchConfiguration('use_slam')
    use_posxy = LaunchConfiguration('use_posxy')
    use_velocity = LaunchConfiguration('use_velocity')

    mapping_share = get_package_share_directory('mapping')
    rf2o_share = get_package_share_directory('rf2o_laser_odometry')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'use_stabilizer',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'use_ekf3',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'use_rf2o',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'use_posxy',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'use_velocity',
            default_value='true',
        ),
    ]

    # 1. Static TFs: base_link -> camera / LiDAR
    # Static transforms do not require continuously advancing sim time.
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mapping_share,
                'launch',
                'static_tf.launch.py',
            )
        ),
    )

    # 2. Scan stabilizer: /scan -> /scan_levelled
    stabilizer = Node(
        package='mapping',
        executable='stabilizer',
        name='scan_leveler',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_stabilizer),
    )

    # 3. ArduPilot EKF3 bridge:
    #    base_footprint -> base_link
    ekf3 = Node(
        package='mapping',
        executable='ekf3',
        name='footprint2link',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_ekf3),
    )

    # 4. RF2O laser odometry:
    #    /scan_levelled -> odometry and odom -> base_footprint
    rf2o = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rf2o_share,
                'launch',
                'rf2o_laser_odometry.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_rf2o),
    )

    # 5. SLAM Toolbox:
    #    Publishes map -> odom
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(
                mapping_share,
                'config',
                'slam_toolbox.yaml',
            ),
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_slam),
    )

    # 6. ArduPilot external position bridge
    posxy = Node(
        package='mapping',
        executable='posxy',
        name='ap_slam_bridge',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'connection_url': 'udp:127.0.0.1:15001',
            },
        ],
        condition=IfCondition(use_posxy),
    )

    # 7. ArduPilot velocity bridge
    velocity = Node(
        package='mapping',
        executable='velocity',
        name='vel2ap',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_velocity),
    )

    return LaunchDescription(
        declare_args + [
            static_tf,
            stabilizer,
            ekf3,
            rf2o,
            slam,
            posxy,
            velocity,
        ]
    )

