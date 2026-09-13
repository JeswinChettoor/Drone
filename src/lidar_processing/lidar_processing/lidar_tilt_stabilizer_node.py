#!/usr/bin/env python3
"""
lidar_tilt_stabilizer_node.py

Replaces the previous 4-node/TF pipeline:
    static_transform_publisher + fcu_tf_bridge + scan_to_cloud + pointcloud_to_laserscan
with a single node.

What it does:
  1. Reads roll/pitch straight from ArduPilot SITL over MAVLink (pymavlink).
  2. Converts each /scan point (range, angle) to XY in the sensor frame.
  3. Rotates those points by the current roll/pitch to "level" them out.
  4. Height-filters the rotated points (same idea as min_height/max_height
     in the old pointcloud_to_laserscan config).
  5. Re-bins the surviving points back onto the original angle grid and
     publishes a flat LaserScan on /scan_stabilized.

No TF tree, no PointCloud2 intermediate, no extra processes.
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from pymavlink import mavutil


class LidarTiltStabilizer(Node):
    def __init__(self):
        super().__init__('lidar_tilt_stabilizer')

        # ---- Parameters ----------------------------------------------
        self.declare_parameter('mavlink_connection', 'udp:127.0.0.1:14550')
        self.declare_parameter('scan_in_topic', '/scan')
        self.declare_parameter('scan_out_topic', '/scan_stabilized')
        self.declare_parameter('min_height', -0.20)
        self.declare_parameter('max_height', 0.50)
        self.declare_parameter('use_yaw', False)  # yaw doesn't affect leveling

        self.mav_conn_str = self.get_parameter('mavlink_connection').value
        scan_in = self.get_parameter('scan_in_topic').value
        scan_out = self.get_parameter('scan_out_topic').value
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.use_yaw = bool(self.get_parameter('use_yaw').value)

        # ---- Shared attitude state -------------------------------------
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._lock = threading.Lock()
        self._running = True

        # ---- MAVLink connection (background thread) --------------------
        self.get_logger().info(f'Connecting to MAVLink at {self.mav_conn_str} ...')
        self.mav = mavutil.mavlink_connection(self.mav_conn_str)
        self.mav.wait_heartbeat()
        self.get_logger().info('MAVLink heartbeat received, streaming ATTITUDE.')
        self._mav_thread = threading.Thread(target=self._mavlink_loop, daemon=True)
        self._mav_thread.start()

        # ---- ROS I/O -----------------------------------------------------
        self.pub = self.create_publisher(LaserScan, scan_out, 10)
        self.sub = self.create_subscription(
            LaserScan, scan_in, self.scan_cb, qos_profile_sensor_data)

    def destroy_node(self):
        self._running = False
        super().destroy_node()

    def _mavlink_loop(self):
        """Continuously read ATTITUDE messages from ArduPilot SITL."""
        while self._running:
            msg = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
            if msg is None:
                continue
            with self._lock:
                self.roll = msg.roll
                self.pitch = msg.pitch
                self.yaw = msg.yaw

    def _rotation_matrix(self, roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        Rx = np.array([[1, 0, 0],
                        [0, cr, -sr],
                        [0, sr, cr]])
        Ry = np.array([[cp, 0, sp],
                        [0, 1, 0],
                        [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0],
                        [sy, cy, 0],
                        [0, 0, 1]])

        if self.use_yaw:
            return Rz @ Ry @ Rx
        return Ry @ Rx

    def scan_cb(self, scan: LaserScan):
        with self._lock:
            roll, pitch, yaw = self.roll, self.pitch, self.yaw

        R = self._rotation_matrix(roll, pitch, yaw)

        n = len(scan.ranges)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        ranges = np.array(scan.ranges, dtype=float)

        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)

        pts = np.vstack((xs, ys, zs))   # 3 x n, sensor frame
        pts_level = R @ pts             # rotated into gravity-aligned frame

        x_l, y_l, z_l = pts_level[0], pts_level[1], pts_level[2]

        height_ok = (z_l >= self.min_height) & (z_l <= self.max_height)
        keep = valid & height_ok

        new_ranges = np.hypot(x_l, y_l)
        new_angles = np.arctan2(y_l, x_l)

        out_ranges = np.full(n, math.inf)

        # Re-bin surviving points onto the ORIGINAL angle grid, keeping the
        # closest return per bin (same behavior as a real 2D lidar / as
        # pointcloud_to_laserscan did).
        bin_idx = np.round((new_angles - scan.angle_min) / scan.angle_increment).astype(int)
        keep_idx = np.where(keep)[0]
        for i in keep_idx:
            b = bin_idx[i]
            if 0 <= b < n and new_ranges[i] < out_ranges[b]:
                out_ranges[b] = new_ranges[i]

        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = out_ranges.tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTiltStabilizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
