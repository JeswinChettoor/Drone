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
TF_TIMEOUT = Duration(seconds=0.1)


def quat_to_rot(q):
    """Full 3x3 rotation matrix from quaternion [x, y, z, w]."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w)],
        [    2 * (x * y + z * w), 1 - 2 * (x * x + z * z),     2 * (y * z - x * w)],
        [    2 * (x * z - y * w),     2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ScanLeveler(Node):
    def __init__(self):
        super().__init__('scan_leveler')
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pub = self.create_publisher(LaserScan, OUTPUT_TOPIC, 10)
        self.create_subscription(
            LaserScan, INPUT_TOPIC, self.on_scan, qos_profile_sensor_data
        )

    def on_scan(self, scan: LaserScan):
        try:
            tf = self.tf_buffer.lookup_transform(
                FRAME, scan.header.frame_id, Time(), timeout=TF_TIMEOUT
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'TF lookup failed: {exc}', throttle_duration_sec=2.0
            )
            return

        t = tf.transform.translation
        R = quat_to_rot(tf.transform.rotation)

        n, inc = len(scan.ranges), scan.angle_increment
        r = np.asarray(scan.ranges, dtype=np.float64)
        a = scan.angle_min + np.arange(n) * inc

        # Keep valid ranges only
        valid = (r >= scan.range_min) & (r <= scan.range_max)
        r, a = r[valid], a[valid]
        if len(r) == 0:
            return

        # 3D points in sensor frame (z=0 for planar laser) -> footprint frame (x/y)
        pts_sensor = np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros_like(r)))
        xy = pts_sensor @ R[:2, :].T + (t.x, t.y)

        # Re-project to 2D polar bins
        new_r = np.hypot(xy[:, 0], xy[:, 1])
        rel = (np.arctan2(xy[:, 1], xy[:, 0]) - scan.angle_min) % (2 * np.pi)
        idx = np.round(rel / inc).astype(int) % n

        out = np.full(n, np.inf)
        np.minimum.at(out, idx, new_r)

        scan.header.frame_id = FRAME
        scan.ranges = out.tolist()
        scan.intensities = []
        self.pub.publish(scan)


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