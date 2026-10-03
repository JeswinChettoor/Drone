#!/usr/bin/env python3


import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from pymavlink import mavutil


class LidarTiltFilter(Node):
    def __init__(self):
        super().__init__('lidar_tilt_filter')

        self.declare_parameter('mavlink_connection', 'udp:127.0.0.1:15000')
        self.declare_parameter('scan_in_topic', '/scan')
        self.declare_parameter('scan_out_topic', '/scan_stabilized')
        self.declare_parameter('min_height', -2.0)
        self.declare_parameter('max_height', 2.50)

        self.mav_conn_str = self.get_parameter('mavlink_connection').value
        scan_in = self.get_parameter('scan_in_topic').value
        scan_out = self.get_parameter('scan_out_topic').value
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)

        self.roll = 0.0
        self.pitch = 0.0
        self._lock = threading.Lock()
        self._running = True

        self.get_logger().info(f'Connecting to MAVLink at {self.mav_conn_str} ...')
        self.mav = mavutil.mavlink_connection(self.mav_conn_str)
        self.mav.wait_heartbeat(timeout=10)
        self.get_logger().info('MAVLink heartbeat received.')
        self._mav_thread = threading.Thread(target=self._mavlink_loop, daemon=True)
        self._mav_thread.start()

        self.pub = self.create_publisher(LaserScan, scan_out, 10)
        self.sub = self.create_subscription(
            LaserScan, scan_in, self.scan_cb, qos_profile_sensor_data)

    def destroy_node(self):
        self._running = False
        super().destroy_node()

    def _mavlink_loop(self):
        while self._running:
            msg = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
            if msg is None:
                continue
            with self._lock:
                # MAVLink ATTITUDE is FRD (Front-Right-Down) body frame.
                # ROS uses FLU (Front-Left-Up), so pitch sign flips.
                self.roll = msg.roll
                self.pitch = -msg.pitch

    def scan_cb(self, scan: LaserScan):
        with self._lock:
            roll, pitch = self.roll, self.pitch

        out_ranges = list(scan.ranges)
        angle = scan.angle_min

        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                # Point in the sensor's flat scan plane.
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                # Height of that point once vehicle tilt is accounted for.
                # (small-angle-free full trig, just the z-component of a
                # roll-then-pitch rotation applied to a point that starts
                # at z=0 in the sensor frame)
                z = -x * math.sin(pitch) + y * math.sin(roll) * math.cos(pitch)

                if z < self.min_height or z > self.max_height:
                    out_ranges[i] = math.inf

            angle += scan.angle_increment

        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = out_ranges
        out.intensities = list(scan.intensities)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTiltFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
