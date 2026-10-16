'''Takes detections on /person_detections and creates posestamped message giving location of survivors on the map on topic /localized_person '''

import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_ros import (Buffer, TransformListener, LookupException,
                     ConnectivityException, ExtrapolationException)
from nidar_msgs.msg import PersonDetectionArray
from pymavlink import mavutil

camera_pos = (0.12,0.0,-0.05)  #Camera position relative to base_link
camera_pitch = 0.7854  #Radian down positive 

mavlink_url = "udp:127.0.0.1:15003"
detection_topic = "/person_detections"
camera_info = "/camera/camera_info"
output_topic = "/localized_person"

FX, FY, CX, CY = 381.36, 381.36, 320.0, 240.0

def rotate_x(v, angle):
    """Right-hand rotation about the FORWARD axis — this is roll."""
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = v
    return (x, c * y - s * z, s * y + c * z)


def rotate_y(v, angle):
    """Right-hand rotation about the LEFT axis — this is pitch.
    Positive angle tips the nose DOWN."""
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = v
    return (c * x + s * z, y, -s * x + c * z)


def level(v, roll, pitch):
    v = rotate_y(v, -pitch)   # first
    v = rotate_x(v, -roll)    # second
    return v

class SurvivorLocalizer(Node):
    def __init__(self):
        super().__init__("survivor_localizer_node")
        self.roll = self.pitch = self.rel_alt = None
        self.conn = mavutil.mavlink_connection(mavlink_url)
        self.create_timer(0.05, self.poll_mavlink)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PoseStamped, output_topic, 10)
        self.create_subscription(PersonDetectionArray, detection_topic,
                                 self.on_detection, 10)

    def poll_mavlink(self):
        while True:
            m = self.conn.recv_match(type=["ATTITUDE", "GLOBAL_POSITION_INT"],
                                     blocking=False)
            if m is None:
                return
            if m.get_type() == "ATTITUDE":
                self.roll, self.pitch = m.roll, -m.pitch
            else:
                self.rel_alt = m.relative_alt / 1000.0
    def on_detection(self, msg):
        if self.roll is None or self.pitch is None or self.rel_alt is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        for d in msg.detections:
            self.publish_point(d, self.roll, self.pitch, self.rel_alt, stamp)

    def publish_point(self, d, roll, pitch, alt, stamp):
        # bbox bottom-center (ground contact) -> optical ray
        u, v = (d.x_min + d.x_max) / 2.0, d.y_max
        r = ((u - CX) / FX, (v - CY) / FY, 1.0)

        # optical (x right, y down, z fwd) -> camera link (x fwd, y left, z up)
        r = (r[2], -r[0], -r[1])

        # camera link -> base_link: mount pitch
        c, s = math.cos(camera_pitch), math.sin(camera_pitch)
        r = (c * r[0] + s * r[2], r[1], -s * r[0] + c * r[2])

        # gravity-aligned: ray direction + camera position (ray starts AT the camera)
        r = level(r, roll, pitch)
        cam = level(camera_pos, roll, pitch)
        cam_z = alt + cam[2]

        # intersect ground plane z = 0
        if r[2] >= -1e-6:
            return
        t = -cam_z / r[2]
        px = cam[0] + t * r[0]        # full camera offset, relative to base_link
        py = cam[1] + t * r[1]
        if not (math.isfinite(px) and math.isfinite(py)):
            return

        # gravity-aligned -> map (TF at detection stamp, else latest)
        try:
            tr = self.tf.lookup_transform("map", "base_link", stamp)
        except (LookupException, ConnectivityException, ExtrapolationException):
            try:
                tr = self.tf.lookup_transform("map", "base_link", Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                return
        tx, ty = tr.transform.translation.x, tr.transform.translation.y
        q = tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))

        out = PoseStamped()
        out.header.stamp = stamp.to_msg()
        out.header.frame_id = "map"
        out.pose.position.x = tx + math.cos(yaw) * px - math.sin(yaw) * py
        out.pose.position.y = ty + math.sin(yaw) * px + math.cos(yaw) * py
        out.pose.position.z = 0.0
        out.pose.orientation.w = 1.0
        self.pub.publish(out)

def main():
    rclpy.init()
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