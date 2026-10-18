#!/usr/bin/env python3

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener
from nidar_msgs.msg import PersonDetectionArray


DETECTION_TOPIC = "/person_detections"
CAMERA_INFO_TOPIC = "/camera/camera_info"
OUTPUT_TOPIC = "/localized_person"
MAP_FRAME = "map"


def rotate_vector(q, v):
    """Rotate vector v by quaternion q."""
    x, y, z = v
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)

    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


class SurvivorLocalizer(Node):

    def __init__(self):
        super().__init__("survivor_localizer")

        # Declare tunable parameter for camera frame (empty string defaults to CameraInfo header)
        self.declare_parameter("camera_frame", "")

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            PoseStamped, OUTPUT_TOPIC, 10
        )

        # Uses SensorData QoS (Best Effort) to match camera drivers
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.on_camera_info,
            qos_profile_sensor_data,
        )

        self.detection_sub = self.create_subscription(
            PersonDetectionArray,
            DETECTION_TOPIC,
            self.on_detection,
            10,
        )

    def on_camera_info(self, msg):
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        # Parameter override takes precedence; fallback to msg.header.frame_id
        configured_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        if configured_frame:
            self.camera_frame = configured_frame
        elif msg.header.frame_id:
            self.camera_frame = msg.header.frame_id

        self.get_logger().info(
            f"Intrinsics received. Frame locked to '{self.camera_frame}'. "
            "Destroying CameraInfo subscriber."
        )

        # Intrinsics are static; destroy the subscription to save CPU
        self.destroy_subscription(self.camera_info_sub)
        self.camera_info_sub = None

    def on_detection(self, msg):
        if self.fx is None:
            self.get_logger().warning(
                "Waiting for camera intrinsics on /camera/camera_info"
            )
            return

        # Check in case parameter was updated after node initialization
        param_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        effective_frame = param_frame or self.camera_frame or msg.header.frame_id

        if not effective_frame:
            self.get_logger().warning(
                "Camera frame unavailable: neither 'camera_frame' param nor "
                "CameraInfo/Detection header.frame_id is set."
            )
            return

        stamp = Time.from_msg(msg.header.stamp)

        try:
            # 50 ms timeout prevents dropouts due to inference delay
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                effective_frame,
                stamp,
                timeout=Duration(seconds=0.05),
            )
        except TransformException as e:
            self.get_logger().warning(
                f"Camera-to-map TF unavailable for '{effective_frame}': {e}"
            )
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        camera_origin = (t.x, t.y, t.z)

        for detection in msg.detections:
            u = (detection.x_min + detection.x_max) / 2.0
            v = detection.y_max

            x_n = (u - self.cx) / self.fx
            y_n = (v - self.cy) / self.fy
            ray_camera = (x_n, y_n, 1.0)

            ray_map = rotate_vector(q, ray_camera)

            if ray_map[2] >= -1e-6:
                continue

            scale = -camera_origin[2] / ray_map[2]

            if scale <= 0.0 or not math.isfinite(scale):
                continue

            px = camera_origin[0] + scale * ray_map[0]
            py = camera_origin[1] + scale * ray_map[1]

            if not (math.isfinite(px) and math.isfinite(py)):
                continue

            point = PoseStamped()
            point.header.stamp = msg.header.stamp
            point.header.frame_id = MAP_FRAME

            point.pose.position.x = px
            point.pose.position.y = py
            point.pose.position.z = 0.0
            point.pose.orientation.w = 1.0

            self.publisher.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = SurvivorLocalizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()