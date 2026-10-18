#!/usr/bin/env python3
"""Survivor localizer with debugging, to find out why nothing is published.

in : /person_detections, /camera/camera_info
out: /localized_person (PoseStamped, map frame, one per detection)

Every STATS_PERIOD_SEC it prints a one-line summary of each pipeline stage and a
HINT telling you which stage is the first one failing.

Run: python3 survivor_localizer_node_debug.py --ros-args -p use_sim_time:=true
"""

from collections import Counter

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener
from nidar_msgs.msg import PersonDetectionArray

# ---- Settings ---------------------------------------------------------------
CAMERA_FRAME = "camera_optical_frame"            # TF frame of the camera ("" = use CameraInfo frame_id)
TF_TIMEOUT_SEC = 0.2         # wait for TF at the image stamp
FALLBACK_TO_LATEST = True    # if stamped TF fails, use the newest TF (pose may be old)
STATS_PERIOD_SEC = 5.0
# -----------------------------------------------------------------------------


def rotate(q, v):
    """Rotate vector v by quaternion q."""
    x, y, z = v
    tx = 2.0 * (q.y * z - q.z * y)
    ty = 2.0 * (q.z * x - q.x * z)
    tz = 2.0 * (q.x * y - q.y * x)
    return (
        x + q.w * tx + q.y * tz - q.z * ty,
        y + q.w * ty + q.z * tx - q.x * tz,
        z + q.w * tz + q.x * ty - q.y * tx,
    )


class SurvivorLocalizer(Node):

    def __init__(self):
        super().__init__("survivor_localizer")

        self.frame = CAMERA_FRAME
        self.K = self.D = None
        self.stats = Counter()
        self.last_err = ""
        self.last_tf_lag = None     # image stamp minus TF stamp (s)
        self.warned_axis = False

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        # Own thread, so /tf keeps arriving while a lookup waits in a callback
        TransformListener(self.tf_buffer, self, spin_thread=True)

        self.pub = self.create_publisher(PoseStamped, "/localized_person", 10)
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.on_info, qos_profile_sensor_data
        )
        self.create_subscription(
            PersonDetectionArray, "/person_detections", self.on_detection, 10
        )
        self.create_timer(STATS_PERIOD_SEC, self.print_stats)

        self.get_logger().info(
            "Started. Listening on /person_detections and /camera/camera_info | "
            f"use_sim_time={self.get_parameter('use_sim_time').value} | "
            f"camera_frame override='{CAMERA_FRAME}' | fallback_to_latest={FALLBACK_TO_LATEST}"
        )

    # ------------------------------------------------------------ camera info
    def on_info(self, msg):
        first = self.K is None
        self.stats["info_msgs"] += 1
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros(5)
        if not CAMERA_FRAME:
            self.frame = msg.header.frame_id
        if first:
            self.get_logger().info(
                f"CameraInfo OK: {msg.width}x{msg.height} fx={self.K[0,0]:.1f} "
                f"fy={self.K[1,1]:.1f} cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f} "
                f"distortion={msg.distortion_model} D={list(self.D)} | frame='{self.frame}'"
            )
            if not self.frame:
                self.get_logger().error("CameraInfo header.frame_id is EMPTY. Set CAMERA_FRAME.")

    # --------------------------------------------------------------------- TF
    def get_transform(self, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", self.frame, stamp, timeout=Duration(seconds=TF_TIMEOUT_SEC)
            )
            self.stats["tf_stamped"] += 1
            self.last_tf_lag = 0.0
            return tf
        except TransformException as e:
            stamped_err = str(e)

        if FALLBACK_TO_LATEST:
            try:
                tf = self.tf_buffer.lookup_transform("map", self.frame, Time())
                self.stats["tf_latest"] += 1
                self.last_tf_lag = (stamp - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
                return tf
            except TransformException as e:
                stamped_err = str(e)

        self.stats["tf_fail"] += 1
        self.last_err = stamped_err
        self.get_logger().warning(
            f"TF FAILED for '{self.frame}': {stamped_err}", throttle_duration_sec=3.0
        )
        return None

    # -------------------------------------------------------------- detection
    def on_detection(self, msg):
        self.stats["det_msgs"] += 1
        if not msg.detections:
            self.stats["det_msgs_empty"] += 1
            return
        self.stats["dets"] += len(msg.detections)

        if self.K is None:
            self.stats["drop_no_info"] += 1
            return
        if not self.frame:
            self.stats["drop_no_frame"] += 1
            return

        tf = self.get_transform(Time.from_msg(msg.header.stamp))
        if tf is None:
            return

        t, q = tf.transform.translation, tf.transform.rotation

        # Camera optical axis (z) in the map frame: should point down for a downward camera
        axis = rotate(q, (0.0, 0.0, 1.0))
        self.get_logger().info(
            f"cam pos=({t.x:.2f}, {t.y:.2f}, {t.z:.2f}) "
            f"optical axis in map=({axis[0]:+.2f}, {axis[1]:+.2f}, {axis[2]:+.2f})",
            throttle_duration_sec=2.0,
        )
        if axis[2] > -0.1 and not self.warned_axis:
            self.warned_axis = True
            self.get_logger().warning(
                "Camera optical axis does not point downward. Frame "
                f"'{self.frame}' is probably NOT an optical frame (z forward, x right, "
                "y down). Use an optical-frame child transform and set CAMERA_FRAME."
            )

        for i, d in enumerate(msg.detections):
            u, v = (d.x_min + d.x_max) / 2.0, d.y_max
            x, y = cv2.undistortPoints(np.array([[[u, v]]], np.float64), self.K, self.D)[0, 0]
            ray = rotate(q, (x, y, 1.0))

            if ray[2] >= -1e-6:
                self.stats["rej_ray_not_down"] += 1
                self.get_logger().warning(
                    f"det{i}: ray does not point at the ground (ray.z={ray[2]:+.3f}) "
                    f"pixel=({u:.0f},{v:.0f})",
                    throttle_duration_sec=3.0,
                )
                continue

            s = -t.z / ray[2]
            if s <= 0.0:
                self.stats["rej_bad_scale"] += 1
                self.get_logger().warning(
                    f"det{i}: bad scale s={s:.3f} (camera z={t.z:.2f}, is it above z=0?)",
                    throttle_duration_sec=3.0,
                )
                continue

            px, py = t.x + s * ray[0], t.y + s * ray[1]

            point = PoseStamped()
            point.header.stamp = msg.header.stamp
            point.header.frame_id = "map"
            point.pose.position.x = px
            point.pose.position.y = py
            point.pose.position.z = 0.0
            point.pose.orientation.w = 1.0
            self.pub.publish(point)
            self.stats["published"] += 1

            self.get_logger().info(
                f"det{i} pixel=({u:.0f},{v:.0f}) norm=({x:+.3f},{y:+.3f}) "
                f"ray=({ray[0]:+.2f},{ray[1]:+.2f},{ray[2]:+.2f}) s={s:.2f} "
                f"-> map=({px:.2f}, {py:.2f})",
                throttle_duration_sec=1.0,
            )

    # ------------------------------------------------------------------ stats
    def print_stats(self):
        s = self.stats
        lag = f"{self.last_tf_lag:.2f}s" if self.last_tf_lag is not None else "n/a"
        rej = {k[4:]: v for k, v in s.items() if k.startswith("rej_")}
        self.get_logger().info(
            f"STATS/{STATS_PERIOD_SEC:.0f}s | camera_info={'yes' if self.K is not None else 'NO'} "
            f"frame='{self.frame}' | det_msgs={s['det_msgs']} (empty={s['det_msgs_empty']}) "
            f"dets={s['dets']} | tf stamped={s['tf_stamped']} latest={s['tf_latest']} "
            f"fail={s['tf_fail']} (pose lag {lag}) | rejected={rej or 'none'} | "
            f"PUBLISHED={s['published']}"
        )
        self.get_logger().info(f"HINT: {self.diagnose()}")
        self.stats.clear()

    def diagnose(self):
        s = self.stats
        if s["published"] > 0:
            return "publishing OK. Check the values against the known person position."
        if self.K is None:
            return "no /camera/camera_info received. Check the topic name and QoS (best effort)."
        if s["det_msgs"] == 0:
            return "no /person_detections received. Is the detector node running?"
        if s["dets"] == 0:
            return "detector runs but sees no people. Check /camera/debug; lower conf."
        if s["tf_fail"] > 0 and s["tf_stamped"] + s["tf_latest"] == 0:
            return f"TF lookup is failing ('{self.last_err[:90]}'). See the TF FAILED warnings."
        if s["rej_ray_not_down"] > 0:
            return "rays do not hit the ground: camera frame is not optical, or the mount TF is wrong."
        if s["rej_bad_scale"] > 0:
            return "camera is at or below z=0 in map. Check the pose source and map origin."
        return "no detections reached projection this period. Wait for people in view."


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