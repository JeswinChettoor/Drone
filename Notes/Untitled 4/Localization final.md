```
ros2 run tf2_ros static_transform_publisher \
  0.20 0.0 0.10   0 0 0 \
  base_link laser_link
# args: x y z  roll pitch yaw   parent child
```
```
This is done by the fcu_tf_bridge

cd ~/workspace/src
ros2 pkg create fcu_tf_bridge --build-type ament_python --dependencies rclpy sensor_msgs tf2_ros geometry_msgs
```
ros2 pkg create lidar_tilt_compensator --build-type ament_python --dependencies rclpy sensor_msgs laser_geometry
```

```