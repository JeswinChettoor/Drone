# Maze Explorer (`maze_explorer`)

A ROS 2 simulation package for autonomous maze exploration using frontier-based exploration and Finite State Machine (FSM) control logic. 

> **Note:** Designed as a proof-of-concept for drone exploration logic, simplified using a differential drive robot in Gazebo simulation.

---

## Features
* **Frontier-Based Exploration:** Automatically detects unknown boundaries in occupancy grid maps to choose navigation targets.
* **FSM Architecture:** Managed via a Finite State Machine for state transitions (Exploring, Turning/Re-orienting, Goal Reached, Recovery).
* **Gazebo Simulation:** Custom world environment (`worlds/`), robot models (`urdf/`, `sdf/`, `meshes/`), and launch configurations.

---

## Prerequisites & Dependencies
* ROS 2 (Humble / Jazzy)
* Gazebo / Ign Gazebo
* Navigation2 (`nav2_bringup`, `nav2_msgs`)
* `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `rclpy`

Install dependencies automatically:
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
