
#!/usr/bin/env python3
"""Projects a tilted 360-degree 2D LiDAR scan onto the horizontal plane of base_footprint.

Intended for rf2o / slam_toolbox. Roll and pitch must be present in the
base_footprint -> base_link transform. Requires only rclpy and NumPy.
"""

import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

INPUT_TOPIC = '/scan'
OUTPUT_TOPIC = '/scan_levelled'
FRAME = 'base_footprint'
TF_TIMEOUT = Duration(seconds=1.0)
FILL_SINGLE_GAPS = True


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class ScanLeveler(Node):
    def __init__(self):
        super().__init__('scan_leveler')

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self, spin_thread=True)

        # Reliable output so both reliable and best-effort subscribers connect
        self.pub = self.create_publisher(LaserScan, OUTPUT_TOPIC, 10)
        self.create_subscription(
            LaserScan, INPUT_TOPIC, self.on_scan, qos_profile_sensor_data
        )

    def on_scan(self, scan: LaserScan):
        try:
            # Use the latest available transform
            tf = self.tf_buffer.lookup_transform(
                FRAME,
                scan.header.frame_id,
                Time(),
                timeout=TF_TIMEOUT
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'TF lookup failed: {exc}',
                throttle_duration_sec=2.0
            )
            return

        t = tf.transform.translation
        R = quat_to_rot(tf.transform.rotation)
        shift = np.array([t.x, t.y, t.z])

        n = len(scan.ranges)
        inc = scan.angle_increment

        r = np.asarray(scan.ranges, dtype=np.float64)
        a = scan.angle_min + np.arange(n) * inc

        # Keep all valid input ranges
        valid = (
            np.isfinite(r)
            & (r >= scan.range_min)
            & (r <= scan.range_max)
        )
        r, a = r[valid], a[valid]

        # LiDAR frame -> base_footprint
        # Points are stored as rows, hence R.T
        pts = (
            np.column_stack((
                r * np.cos(a),
                r * np.sin(a),
                np.zeros_like(r)
            )) @ R.T
        ) + shift

        # No height-based filtering: retain all transformed points

        # Project onto the horizontal plane and rebin
        new_r = np.hypot(pts[:, 0], pts[:, 1])
        rel = (
            np.arctan2(pts[:, 1], pts[:, 0]) - scan.angle_min
        ) % (2 * math.pi)

        idx = np.round(rel / inc).astype(int) % n

        valid = (
            (new_r >= scan.range_min)
            & (new_r <= scan.range_max)
        )

        out = np.full(n, np.inf)
        np.minimum.at(out, idx[valid], new_r[valid])

        if FILL_SINGLE_GAPS:
            # Fill isolated empty bins that have a valid neighbour on both sides
            left, right = np.roll(out, 1), np.roll(out, -1)
            hole = (
                ~np.isfinite(out)
                & np.isfinite(left)
                & np.isfinite(right)
            )
            out[hole] = np.minimum(left[hole], right[hole])

        msg = LaserScan()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = FRAME
        msg.angle_min = scan.angle_min
        msg.angle_max = scan.angle_max
        msg.angle_increment = inc
        msg.time_increment = scan.time_increment
        msg.scan_time = scan.scan_time
        msg.range_min = scan.range_min
        msg.range_max = scan.range_max
        msg.ranges = out.tolist()

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanLeveler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
