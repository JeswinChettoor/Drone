#!/usr/bin/env python3
"""
Nav2 (controller, planner, behaviors, bt_navigator) + DFS explorer.

Assumes SLAM / the simulator already provide:
  /map, TF map->odom->base_link, /scan, /odom, and a robot listening on cmd_vel.

Arguments:
  use_sim_time:=true|false   (default true)
  explore:=true|false        false = Nav2 only (send goals from RViz)
  cmd_vel_topic:=cmd_vel     topic your robot listens on
  params_file:=...  bt_xml:=...
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("nidar_explorer")
    params = LaunchConfiguration("params_file")
    bt_xml = LaunchConfiguration("bt_xml")
    cmd_vel = LaunchConfiguration("cmd_vel_topic")
    sim = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}

    nav_nodes = [
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             output="screen", parameters=[params, sim], remappings=[("cmd_vel", cmd_vel)]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[params, sim]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             output="screen", parameters=[params, sim], remappings=[("cmd_vel", cmd_vel)]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             output="screen",
             parameters=[params, sim, {"default_nav_to_pose_bt_xml": bt_xml}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[sim, {"autostart": True,
                               "node_names": ["controller_server", "planner_server",
                                              "behavior_server", "bt_navigator"]}]),
    ]

    explorer = Node(package="nidar_explorer", executable="dfs_explorer", name="dfs_explorer",
                    output="screen", parameters=[sim],
                    condition=IfCondition(LaunchConfiguration("explore")))

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("explore", default_value="true"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="cmd_vel"),
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("bt_xml",
                              default_value=os.path.join(share, "behavior_trees", "nav_bt.xml")),
        *nav_nodes,
        explorer,
    ])
