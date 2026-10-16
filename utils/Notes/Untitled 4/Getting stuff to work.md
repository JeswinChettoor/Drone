```
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH}"
```
```# Set environment variables system-wide for non-interactive and interactive shells
export AMENT_PREFIX_PATH="/opt/ros/humble"
export PATH="/opt/ros/humble/bin:$PATH"
export LD_LIBRARY_PATH="/opt/ros/humble/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH"
# Also append sourcing to default bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
```
```
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://127.0.0.1:14550@127.0.0.1:14557 \
  -p tgt_system:=1 \
  -p tgt_component:=1 \
  -p use_sim_time:=true


ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@127.0.0.1:14557 use_sim_time:=true


cd /workspace
ros2 launch drone_bringup.launch.py


```

```
export PATH="/opt/ros/humble/bin:$PATH" && \
export AMENT_PREFIX_PATH="/opt/ros/humble" && \
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH}" && \
source /opt/ros/humble/setup.bash && \
if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && \
ros2 launch drone_bringup.launch.py
```

```
export LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH}" && \
export PATH="/opt/ros/humble/bin:${PATH}" && \
export AMENT_PREFIX_PATH="/opt/ros/humble" && \
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH}" && \
source /opt/ros/humble/setup.bash && \
if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && \
ros2 launch drone_bringup.launch.py
```

## Latest  
```
ros2 launch drone_bringup drone_bringup.launch.py
```
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
also change base to base_footprint

 ros2 launch lidar_tilt_compensator lidar_stabilization.launch.py

```bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```


Setting 