#!/usr/bin/env python3
"""
odom_to_mavlink_node.py

Subscribes to a nav_msgs/Odometry topic (e.g. /odom_rf2o from rf2o_laser_odometry)
and forwards it to ArduPilot (SITL or real hardware) as a MAVLink ODOMETRY message,
which ArduPilot's EKF3 can use as an "ExternalNav" position/velocity/yaw source.

Frame handling
--------------
ROS (REP-103) odometry frame:      x = forward, y = left,  z = up      (right-handed)
MAVLink MAV_FRAME_*_FRD:           x = forward, y = right, z = down    (right-handed)

Going from ROS to FRD is a 180 degree rotation about the forward (X) axis:
    x_frd =  x_ros
    y_frd = -y_ros
    z_frd = -z_ros
    yaw_frd = -yaw_ros   (only yaw matters for 2D lidar odometry -> roll = pitch = 0)

The ODOMETRY message conveniently lets pose and twist live in *different* frames
(frame_id vs child_frame_id), which matches rf2o's message exactly:
    - pose  is in the fixed "odom" frame  -> MAV_FRAME_LOCAL_FRD
    - twist is in the robot's body frame  -> MAV_FRAME_BODY_FRD
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from pymavlink import mavutil


class OdomToMavlink(Node):

    def __init__(self):
        super().__init__('odom_to_mavlink')

        self.declare_parameter('odom_topic', '/odom_rf2o')
        self.declare_parameter('connection_string', 'udpin:127.0.0.1:14552')
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)
        self.declare_parameter('wait_for_heartbeat', True)

        odom_topic = self.get_parameter('odom_topic').value
        connection_string = self.get_parameter('connection_string').value
        self.target_system = int(self.get_parameter('target_system').value)
        self.target_component = int(self.get_parameter('target_component').value)
        wait_for_heartbeat = bool(self.get_parameter('wait_for_heartbeat').value)

        self.reset_counter = 0
        self._last_stamp = None

        self.get_logger().info(f'Opening MAVLink connection: {connection_string}')
        self.mav = mavutil.mavlink_connection(
            connection_string,
            source_system=self.target_system,
            source_component=196,  # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY-ish "companion" id
        )

        if wait_for_heartbeat:
            self.get_logger().info('Waiting for ArduPilot heartbeat...')
            self.mav.wait_heartbeat()
            self.get_logger().info(
                f'Heartbeat received from system {self.mav.target_system}, '
                f'component {self.mav.target_component}'
            )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, qos)
        self.get_logger().info(f'Subscribed to {odom_topic}, streaming ODOMETRY to ArduPilot')

    def odom_cb(self, msg: Odometry) -> None:
        # ---- position: ROS (fwd, left, up) -> MAV_FRAME_LOCAL_FRD (fwd, right, down)
        px = msg.pose.pose.position.x
        py = -msg.pose.pose.position.y
        pz = -msg.pose.pose.position.z

        # ---- yaw only (rf2o is a 2D lidar odometry source: no roll/pitch)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_ros = math.atan2(siny_cosp, cosy_cosp)
        yaw_frd = -yaw_ros

        qw = math.cos(yaw_frd / 2.0)
        qz = math.sin(yaw_frd / 2.0)
        q_frd = [qw, 0.0, 0.0, qz]  # ODOMETRY wants [w, x, y, z]

        # ---- twist: ROS body frame (fwd, left, up) -> MAV_FRAME_BODY_FRD (fwd, right, down)
        vx = msg.twist.twist.linear.x
        vy = -msg.twist.twist.linear.y
        vz = -msg.twist.twist.linear.z
        yawspeed = -msg.twist.twist.angular.z

        time_usec = int(self.get_clock().now().nanoseconds / 1000)

        nan21 = [float('nan')] * 21

        self.mav.mav.odometry_send(
            time_usec,
            mavutil.mavlink.MAV_FRAME_LOCAL_FRD,   # frame_id      -> pose is in this frame
            mavutil.mavlink.MAV_FRAME_BODY_FRD,    # child_frame_id-> twist is in this frame
            px, py, pz,
            q_frd,
            vx, vy, vz,
            0.0, 0.0, yawspeed,                    # rollspeed, pitchspeed, yawspeed
            nan21,                                 # pose_covariance   (NaN = "use VISO_*_M_NSE defaults")
            nan21,                                 # velocity_covariance (unused by ArduPilot)
            self.reset_counter,
            estimator_type=mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
            quality=100,
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomToMavlink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
