ros2 launch drone_control slam.launch.py
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
 ros2 launch lidar_processing lidar_stabilization_launch.py
ros2 launch gz_ros_bridge bridge.launch.py
cd /workspace/SIM/Worlds && gz sim -v4 -r combined_arena.sdf
sim_vehicle.py -v ArduCopter -f gazebo-iris   --model JSON   -w   --add-param-file=/workspace/no_gps.parm   --out=udpout:127.0.0.1:15000   --out=udpout:127.0.0.1:15001   --console

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link iris/lidar_link/lidar_2d