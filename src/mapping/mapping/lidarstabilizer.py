#!/usr/bin/env python3
"""Projects a tilted 2D LiDAR scan onto the horizontal plane of base_footprint.

Intended for rf2o / slam_toolbox. Roll and pitch must be present in the
base_footprint -> base_link transform.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class ScanLeveler(Node):
    def __init__(self):
        super().__init__('scan_leveler')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_levelled')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('z_min', 0.05)
        self.declare_parameter('z_max', 1.0)
        self.declare_parameter('tf_timeout', 0.05)
        self.declare_parameter('fill_single_gaps', True)

        self.frame = self.get_parameter('target_frame').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.timeout = Duration(seconds=self.get_parameter('tf_timeout').value)
        self.fill_gaps = self.get_parameter('fill_single_gaps').value

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self, spin_thread=True)

        # Reliable output so both reliable and best-effort subscribers connect
        self.pub = self.create_publisher(
            LaserScan, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('input_topic').value,
            self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan: LaserScan):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame, scan.header.frame_id,
                Time.from_msg(scan.header.stamp), timeout=self.timeout)
        except TransformException as exc:
            self.get_logger().warning(f'TF lookup failed: {exc}', throttle_duration_sec=2.0)
            return

        q, t = tf.transform.rotation, tf.transform.translation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        shift = [t.x, t.y, t.z]

        n = len(scan.ranges)
        inc = scan.angle_increment
        full_circle = abs((scan.angle_max - scan.angle_min) - 2 * math.pi) < 2 * abs(inc)

        r = np.asarray(scan.ranges, dtype=np.float64)
        a = scan.angle_min + np.arange(n) * inc
        ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        r, a = r[ok], a[ok]

        pts = rot.apply(np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros_like(r)))) + shift
        pts = pts[(pts[:, 2] > self.z_min) & (pts[:, 2] < self.z_max)]

        new_r = np.hypot(pts[:, 0], pts[:, 1])
        rel = (np.arctan2(pts[:, 1], pts[:, 0]) - scan.angle_min) % (2 * math.pi)
        idx = np.round(rel / inc).astype(int)
        if full_circle:
            idx %= n   # a point at the very end of the sweep wraps to bin 0

        ok = (idx < n) & (new_r >= scan.range_min) & (new_r <= scan.range_max)
        out = np.full(n, np.inf)
        np.minimum.at(out, idx[ok], new_r[ok])   # nearest point per bin

        if self.fill_gaps:
            left, right = np.roll(out, 1), np.roll(out, -1)
            hole = ~np.isfinite(out) & np.isfinite(left) & np.isfinite(right)
            if not full_circle:
                hole[0] = hole[-1] = False   # don't wrap across the FOV edge
            out[hole] = np.minimum(left[hole], right[hole])

        msg = LaserScan()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = self.frame
        msg.angle_min, msg.angle_max = scan.angle_min, scan.angle_max
        msg.angle_increment = inc
        msg.time_increment, msg.scan_time = scan.time_increment, scan.scan_time
        msg.range_min, msg.range_max = scan.range_min, scan.range_max
        msg.ranges = out.tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ScanLeveler())


if __name__ == '__main__':
    main()