import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    # everything custom (world, urdf, configs, the explorer node) lives in maze_explorer
    pkg_maze_explorer = get_package_share_directory('maze_explorer')
   # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

   # world_file = os.path.join(pkg_maze_explorer, 'worlds', 'maze.world')
   # urdf_file = os.path.join(pkg_maze_explorer, 'urdf', 'robot.urdf')

    nav2_params_file = os.path.join(pkg_maze_explorer, 'config', 'nav2_params.yaml')
    #slam_params_file = os.path.join(pkg_maze_explorer, 'config', 'slam_params.yaml')
    explorer_params_file = os.path.join(pkg_maze_explorer, 'config', 'explorer_params.yaml')

    #models_dir = os.path.expanduser('~/.gazebo/models')

    # robot_state_publisher wants the urdf as a plain string
  #  with open(urdf_file, 'r') as urdf:
   #     robot_description = urdf.read()

    # ------------------------------------------------------------------
    # 3. Simulation (starts right away)
    # ------------------------------------------------------------------
 #   gazebo = IncludeLaunchDescription(
 #       PythonLaunchDescriptionSource(
 #           os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
 #       ),
 #       launch_arguments={'world': world_file}.items()
 #   )

#    robot_state_publisher = Node(
 #       package='robot_state_publisher',
  #      executable='robot_state_publisher',
   #     output='screen',
    #    parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    #)

    # ------------------------------------------------------------------
    # 4. Put the robot in the world (after 5 s so gazebo is ready)
    # ------------------------------------------------------------------
#    spawn_robot = Node(
#        package='gazebo_ros',
#        executable='spawn_entity.py',
#        arguments=['-topic', 'robot_description', '-entity', 'holonomic_bot',
#                   '-x', '-6.1', '-y', '6.5', '-z', '0.2'],
 #       output='screen'
  #  )
   # delayed_robot_spawn = TimerAction(period=5.0, actions=[spawn_robot])

    # ------------------------------------------------------------------
    # 5. SLAM + Nav2 (after 5 s)
    # ------------------------------------------------------------------
 #   slam_toolbox = Node(
  #      package='slam_toolbox',
   #     executable='async_slam_toolbox_node',
  #      name='slam_toolbox',
  #      output='screen',
  #      parameters=[slam_params_file, {'use_sim_time': True}]
   # )

    # navigation_launch.py (not bringup_launch.py) because slam_toolbox
    # already gives us the map and the map->odom transform
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': nav2_params_file
        }.items()
    )
    delayed_navigation = TimerAction(period=5.0, actions=[nav2_bringup])

    # ------------------------------------------------------------------
    # 6. RViz (after 7 s)
    # ------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    delayed_rviz = TimerAction(period=7.0, actions=[rviz_node])

    # ------------------------------------------------------------------
    # 7. Our explorer (after 10 s)
    # ------------------------------------------------------------------
    # name must match the top-level key in explorer_params.yaml,
    # otherwise the parameters get silently ignored.
    # (the node also waits on its own for /map, tf and the nav2 action servers)
    explorer_node = Node(
        package='maze_explorer',
        executable='explorer_node',
        name='explorer_node',
        output='screen',
        parameters=[explorer_params_file, {'use_sim_time': True}]
    )
    delayed_explorer = TimerAction(period=10.0, actions=[explorer_node])

    # ------------------------------------------------------------------
    # 8. Put it all together
    # ------------------------------------------------------------------
    return LaunchDescription([
        #AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=models_dir),

       # gazebo,
      # robot_state_publisher,
       # delayed_robot_spawn,
        delayed_navigation,
        delayed_rviz,
        delayed_explorer,
    ])
