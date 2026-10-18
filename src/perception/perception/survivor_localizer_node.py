import cv2
import numpy as np
import rclpy

from geometry_msgs.msg import PoseStamped
from nidar_msgs.msg import PersonDetectionArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener


def rotate_vector(q, v):
    """Rotate 3D vector v by quaternion q using Rodrigues formula."""
    x, y, z = v
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # t = 2 * (q_vec x v)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)

    # v' = v + qw * t + (q_vec x t)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


class SurvivorLocalizer(Node):
    def __init__(self):
        super().__init__("survivor_localizer")

        self.K = None
        self.D = None
        self.ground_z = 0.0

        # Change to match your exact TF frame if different
        self.camera_frame = "iris/front_camera_link/front_down_camera"
        self.target_frame = "map"

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.pub = self.create_publisher(PoseStamped, "/localized_person", 10)

        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.on_info, qos_profile_sensor_data
        )
        self.create_subscription(
            PersonDetectionArray, "/person_detections", self.on_detection, 10
        )

    def on_info(self, msg: CameraInfo):
        if self.K is not None:
            return

        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if K[0, 0] <= 0.0:
            return

        self.K = K
        self.D = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros(5)
        self.get_logger().info("Camera intrinsics initialized.")

    def on_detection(self, msg: PersonDetectionArray):
        if not msg.detections or self.K is None:
            return

        # Time() = Time 0 -> asks tf2 for the latest available transform
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=3.0)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        cam_pos = np.array([t.x, t.y, t.z], dtype=np.float64)

        for d in msg.detections:
            # Bottom-center pixel of bounding box
            u = (d.x_min + d.x_max) / 2.0
            v = d.y_max

            # Undistort to normalized optical coordinates (z_opt = 1.0)
            pt = cv2.undistortPoints(
                np.array([[[u, v]]], dtype=np.float64), self.K, self.D
            )[0, 0]

            # Map optical coordinates (X right, Y down, Z forward) to camera link (X forward, Y left, Z up)
            ray_link = (1.0, -pt[0], -pt[1])

            # Rotate ray into map frame
            ray_map = np.array(rotate_vector(q, ray_link), dtype=np.float64)

            # Ensure ray points downward into the ground
            if ray_map[2] >= -1e-4:
                continue

            # Ground plane intersection
            s = (self.ground_z - cam_pos[2]) / ray_map[2]
            if s <= 0.0:
                continue

            map_pt = cam_pos + s * ray_map

            # Publish result
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.target_frame
            pose.pose.position.x = float(map_pt[0])
            pose.pose.position.y = float(map_pt[1])
            pose.pose.position.z = float(self.ground_z)
            pose.pose.orientation.w = 1.0

            self.pub.publish(pose)
            self.get_logger().info(f"Target located at: ({map_pt[0]:.2f}, {map_pt[1]:.2f})")


def main():
    rclpy.init()
    node = SurvivorLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()