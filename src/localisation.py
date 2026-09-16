import math
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from pymavlink import mavutil

NAN = float('nan')
SLAM_JUMP_THRESHOLD_M = 1.0
NULL_ROTATION_QUATERNION = [1.0, 0.0, 0.0, 0.0]
UNKNOWN_COVARIANCE = [NAN] + [0.0] * 20


def ros_enu_to_ap_ned(x, y, z):
    return y, x, -z


class APSlamBridge(Node):
    def __init__(self):
        super().__init__('ap_slam_bridge')

        self.declare_parameter('connection_url', 'udp:127.0.0.1:15001')
        self.declare_parameter('pose_topic', '/pose')

        url = self.get_parameter('connection_url').value
        self.mav = mavutil.mavlink_connection(url)
        self.mav.wait_heartbeat()
        print(f'[MAVLINK] connected to system {self.mav.target_system}', flush=True)

        self.mav_lock = threading.Lock()
        self.last_slam_pos = None
        self.reset_counter = 0
        self.previous_slam_xy = None
        self._sent_count = 0

        pose_qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)
        pose_topic = self.get_parameter('pose_topic').value
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self._on_slam_pose, pose_qos)

        self.create_timer(1.0 / 10.0, self._send_pose_to_ardupilot)
        self.create_timer(2.0, self._report_send_rate)

    def _on_slam_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.last_slam_pos = (p.x, p.y, p.z, msg.header.stamp)

    def _send_pose_to_ardupilot(self):
        if self.last_slam_pos is None:
            return

        x, y, z, stamp = self.last_slam_pos
        x_ned, y_ned,z_ned = ros_enu_to_ap_ned(x, y, z)

        if self.previous_slam_xy is not None:
            prev_x, prev_y = self.previous_slam_xy
            if math.hypot(x_ned - prev_x, y_ned - prev_y) > SLAM_JUMP_THRESHOLD_M:
                self.reset_counter = (self.reset_counter + 1) % 256
                print(f'[JUMP] reset_counter={self.reset_counter}', flush=True)
        self.previous_slam_xy = (x_ned, y_ned)

        try:
            with self.mav_lock:
                print("The position sent last : ", x_ned , y_ned)
                self.mav.mav.odometry_send(
                    int(stamp.sec * 1_000_000 + stamp.nanosec / 1000),
                    mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
                    mavutil.mavlink.MAV_FRAME_BODY_FRD,
                    x_ned, y_ned,z_ned,
                    NULL_ROTATION_QUATERNION,
                    NAN, NAN, NAN,
                    NAN, NAN, NAN,
                    UNKNOWN_COVARIANCE,
                    UNKNOWN_COVARIANCE,
                    self.reset_counter,
                    mavutil.mavlink.MAV_ESTIMATOR_TYPE_LIDAR,
                    0,
                )
            self._sent_count += 1
        except Exception as e:
            self.get_logger().warn(f'ODOMETRY send failed: {e}', throttle_duration_sec=2.0)

    def _report_send_rate(self):
        print(f'[RATE] {self._sent_count} msgs/2s', flush=True)
        self._sent_count = 0


def main():
    rclpy.init(args=sys.argv)
    node = APSlamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('[MAIN] shutting down', flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()