



cd /workspace/SIM/Worlds && gz sim -v4 -r world_dummies.sdf
sim_vehicle.py -v ArduCopter -f gazebo-iris   --model JSON   -w   --add-param-file=/workspace/test.parm   --out=udpout:127.0.0.1:15000   --out=udpout:127.0.0.1:15001   --console --out=udpout:127.0.0.1:15003 --out=udpout:127.0.0.1:14551


ros2 launch gz_ros_bridge bridge.launch.py

 ros2 launch lidar_processing lidar_stabilization_launch.py

ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py

ros2 launch drone_control slam.launch.py



ros2 run tf2_ros static_transform_publisher \
  --x 0.12 --y 0 --z -0.00 \
  --roll 0 --pitch 0.7854 --yaw 0 \
  --frame-id base_link \
  --child-frame-id iris/front_camera_link/front_down_camera \
  --ros-args -p use_sim_time:=true

### To give position commands
```
module load setpos
setpos 5 0 0 0
```


## TO RUN IRL
docker exec -it nidar-dev bash
source install/setup.bash
source /opt/ros/humble/setup.bash

```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/workspace/src/mapping/config/slam_toolbox.yaml
```

ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id base_footpring --child-frame-id laser


ros2 launch sllidar_ros2 view_sllidar_c1_launch.py

