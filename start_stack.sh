#!/usr/bin/env bash
set -e

PIDS=()

cleanup() {
  echo -e "\n[INFO] Shutting down simulation and ROS 2 nodes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  killall -9 gz ruby 2>/dev/null || true
  exit 0
}

# Trap SIGINT, SIGTERM, and EXIT to tear down all processes cleanly
trap cleanup SIGINT SIGTERM EXIT

# 1. Source workspace environment
if [ -f "/workspace/install/setup.bash" ]; then
  source /workspace/install/setup.bash
fi

# 2. Gazebo Harmonic (world_dummies.sdf)
echo "[1/5] Starting Gazebo Harmonic..."
(cd /workspace/SIM/Worlds && gz sim -v4 -r world_dummies.sdf) &
PIDS+=($!)
sleep 3

# 3. Static Transform Publisher
echo "[2/5] Publishing static transform (base_link -> iris/lidar_link/lidar_2d)..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id iris/lidar_link/lidar_2d &
PIDS+=($!)
sleep 1
echo "[2/5] Publishing static transform (base_link -> base_footprint)..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id base_footprint &
PIDS+=($!)
sleep 1

# 4. GZ ROS Bridge
echo "[3/5] Launching gz_ros_bridge..."
ros2 launch gz_ros_bridge bridge.launch.py &
PIDS+=($!)
sleep 2

# 5. LiDAR Stabilization
echo "[4/5] Launching LiDAR stabilization..."
ros2 launch lidar_processing lidar_stabilization_launch.py &
PIDS+=($!)
sleep 2

# 6. Laser Odometry (RF2O)
echo "[5/5] Launching RF2O laser odometry..."
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &
PIDS+=($!)
sleep 2

# 7. SLAM
echo "[INFO] Launching SLAM..."
ros2 launch drone_control slam.launch.py &
PIDS+=($!)

echo "[INFO] Stack is running. Press Ctrl+C to terminate all processes."
wait
