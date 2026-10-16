#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from pymavlink import mavutil

NAN = float('nan')
IDENTITY_QUATERNION = [1.0, 0.0, 0.0, 0.0]   
UNKNOWN_COVARIANCE = [NAN] + [0.0] * 20      


class APSlamBridge(Node):
    def __init__(self):
        super().__init__('ap_slam_bridge')

        self.declare_parameter('connection_url', 'udp:127.0.0.1:15001')
        self.declare_parameter('pose_topic', '/pose')

        url = self.get_parameter('connection_url').value
        self.mav = mavutil.mavlink_connection(url)
        self.mav.wait_heartbeat()
        self.get_logger().info(f'MAVLink connected to system {self.mav.target_system}')

        qos = QoSProfile(depth=4,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('pose_topic').value,
            self.on_pose,
            qos,
        )

    def on_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        stamp = msg.header.stamp

        # ROS ENU -> ArduPilot NED
        x_ned = p.y
        y_ned = p.x
        z_ned = -p.z

        self.mav.mav.odometry_send(
            int(stamp.sec * 1_000_000 + stamp.nanosec / 1000), 
            mavutil.mavlink.MAV_FRAME_LOCAL_FRD,              
            mavutil.mavlink.MAV_FRAME_BODY_FRD,              
            x_ned, y_ned, z_ned,
            IDENTITY_QUATERNION,
            0.0, 0.0, 0.0,                                    
            0.0, 0.0, 0.0,                                      
            UNKNOWN_COVARIANCE,
            UNKNOWN_COVARIANCE,
            0,                                                 
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
            100,                                                 
        )


def main():
    rclpy.init(args=sys.argv)
    node = APSlamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()