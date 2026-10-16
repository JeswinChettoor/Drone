```
```bash
sudo apt update
sudo apt install ros-humble-tf2-ros ros-humble-tf2-tools
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-robot-state-publisher
```
```
```
```bash
ros2 pkg create --build-type ament_cmake lidar_preprocessing --dependencies rclcpp sensor_msgs geometry_msgs tf2 tf2_geometry_msgs nav_msgs
```
```

install mavros-extras
