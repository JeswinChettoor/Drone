import math
import sys
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse
from pymavlink import mavutil

NAN = float('nan')

SLAM_JUMP_THRESHOLD_M = 1.0
ODOM_STALE_SEC = 0.3

NED_ENU_ROTATION = quaternion_from_euler(math.pi, 0.0, math.pi / 2.0)
BODY_FRAME_FLIP = quaternion_from_euler(math.pi, 0.0, 0.0)


def ned2enu(x, y, z):
    return y, x, -z


def ned_orientation_to_enu(q):
    return quaternion_multiply(NED_ENU_ROTATION, quaternion_multiply(q, BODY_FRAME_FLIP))


class APSlamBridge(Node):
    def __init__(self):
        super().__init__('ap_slam_bridge')
        print('[INIT] node created', flush=True)

        self.declare_parameter('connection_url', 'udp:127.0.0.1:14551')
        url = self.get_parameter('connection_url').value
        print(f'[INIT] connection_url param = {url}', flush=True)

        print(f'[MAVLINK] opening connection to {url} ...', flush=True)
        self.mav = mavutil.mavlink_connection(url)
        print('[MAVLINK] connection object created, waiting for heartbeat '
              '(this blocks until ArduPilot/SITL responds) ...', flush=True)

        t0 = time.time()
        self.mav.wait_heartbeat()
        print(f'[MAVLINK] heartbeat received after {time.time() - t0:.2f}s - '
              f'target_system={self.mav.target_system}, '
              f'target_component={self.mav.target_component}', flush=True)

        self.mav_lock = threading.Lock()

        print('[SETUP] requesting LOCAL_POSITION_NED / ATTITUDE at 30Hz ...', flush=True)
        self._ask_ardupilot_for_position_updates()
        print('[SETUP] message interval requests sent', flush=True)

        print('[SETUP] sending EKF origin ...', flush=True)
        self._set_ekf_origin()
       
        # ---- ArduPilot -> ROS (odom + TF) ----
        odom_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', odom_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_position = None
        self.last_orientation = None
        self.last_stamp = None
        self._mav_msg_count = 0
        self._publish_skip_count = 0

        print('[SETUP] starting MAVLink read thread ...', flush=True)
        threading.Thread(target=self._read_ardupilot_loop, daemon=True).start()
        self.create_timer(1.0 / 30.0, self._publish_odometry)

        # ---- slam_toolbox -> ArduPilot ----
        self.last_slam_pose = None
        self.reset_counter = 0
        self._slam_msg_count = 0
        self._send_skip_count = 0

        pose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self._on_slam_pose, pose_qos)
        print('[SETUP] subscribed to /pose (QoS=RELIABLE) - '
              'if slam_toolbox publishes with a different QoS, this subscription '
              'will silently never connect - run `ros2 topic info /pose --verbose` '
              'in another terminal to check', flush=True)

        self.previous_slam_xy = None
        self.create_timer(1.0 / 10.0, self._send_pose_to_ardupilot)

        print('[SETUP] all timers and threads started, handing off to rclpy.spin() ...', flush=True)

    # ---------------- setup ----------------

    def _ask_ardupilot_for_position_updates(self):
        interval_us = int(1_000_000 / 30)
        with self.mav_lock:
            for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                           mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
                self.mav.mav.command_long_send(
                    self.mav.target_system, self.mav.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, msg_id, interval_us, 0, 0, 0, 0, 0
                )
                print(f'[SETUP] requested message id {msg_id} at {interval_us}us interval', flush=True)

    def _set_ekf_origin(self):
        with self.mav_lock:
            self.mav.mav.set_gps_global_origin_send(
                self.mav.target_system,
                int(-35.363262 * 1e7), int(149.165237 * 1e7), int(584 * 1000)
            )

    # ---------------- ArduPilot -> ROS ----------------

    def _read_ardupilot_loop(self):
        print('[MAVLINK-THREAD] read loop starting', flush=True)
        last_status_print = time.time()
        while rclpy.ok():
            with self.mav_lock:
                msg = self.mav.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE'],
                                           blocking=True, timeout=0.05)
            if msg is None:
                if time.time() - last_status_print > 3.0:
                    print(f'[MAVLINK-THREAD] still waiting for LOCAL_POSITION_NED/ATTITUDE - '
                          f'{self._mav_msg_count} messages received so far. '
                          f'If this stays at 0, ArduPilot is not sending these messages - '
                          f'check EK3_SRC1_POSXY/EKF setup and that SET_MESSAGE_INTERVAL succeeded.',
                          flush=True)
                    last_status_print = time.time()
                continue

            self._mav_msg_count += 1
            stamp = self.get_clock().now().to_msg()
            if msg.get_type() == 'LOCAL_POSITION_NED':
                self.last_position = ned2enu(msg.x, msg.y, msg.z)
                self.last_stamp = stamp
                if self._mav_msg_count % 30 == 1:  # print roughly once a second at 30Hz
                    print(f'[MAVLINK-THREAD] LOCAL_POSITION_NED x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f}',
                          flush=True)
            elif msg.get_type() == 'ATTITUDE':
                q_ned = quaternion_from_euler(msg.roll, msg.pitch, msg.yaw)
                self.last_orientation = ned_orientation_to_enu(q_ned)
                self.last_stamp = stamp

    def _publish_odometry(self):
        if self.last_position is None or self.last_orientation is None:
            self._publish_skip_count += 1
            if self._publish_skip_count % 30 == 1:  # print roughly once a second
                print(f'[PUBLISH] skipping - no ArduPilot data yet '
                      f'(position={self.last_position is not None}, '
                      f'orientation={self.last_orientation is not None})', flush=True)
            return

        age_sec = self.get_clock().now().nanoseconds / 1e9 - (
            self.last_stamp.sec + self.last_stamp.nanosec / 1e9
        )
        if age_sec > ODOM_STALE_SEC:
            self.get_logger().warn(
                f'ArduPilot odometry is stale ({age_sec:.2f}s old) - '
                f'not publishing. Check MAVLink link.',
                throttle_duration_sec=2.0
            )
            return

        x, y, z = self.last_position
        qx, qy, qz, qw = self.last_orientation

        odom = Odometry()
        odom.header.stamp = self.last_stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_publisher.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = self.last_stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    # ---------------- slam_toolbox -> ArduPilot ----------------

    def _on_slam_pose(self, msg: PoseWithCovarianceStamped):
        self._slam_msg_count += 1
        p = msg.pose.pose.position
        self.last_slam_pose = (p.x, p.y, p.z, msg.pose.covariance, msg.header.stamp)
        if self._slam_msg_count % 10 == 1:  # roughly once a second if /pose is ~10Hz
            print(f'[SLAM] received /pose #{self._slam_msg_count}: x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}',
                  flush=True)

    def _send_pose_to_ardupilot(self):
        if self.last_slam_pose is None:
            self._send_skip_count += 1
            if self._send_skip_count % 10 == 1:  # roughly once a second at 10Hz
                print('[SEND] skipping - no /pose message received yet. '
                      'Is slam_toolbox running and publishing /pose? '
                      'Run `ros2 topic hz /pose` in another terminal to check.', flush=True)
            return

        x, y, z, cov, stamp = self.last_slam_pose
        x_ned, y_ned, _ = ned2enu(x, y, z)

        if self.previous_slam_xy is not None:
            prev_x, prev_y = self.previous_slam_xy
            distance_moved = math.hypot(x_ned - prev_x, y_ned - prev_y)
            if distance_moved > SLAM_JUMP_THRESHOLD_M:
                self.reset_counter = (self.reset_counter + 1) % 256
                self.get_logger().warn(
                    f'SLAM pose jumped {distance_moved:.2f} m - '
                    f'reset_counter now {self.reset_counter}'
                )
        self.previous_slam_xy = (x_ned, y_ned)

        pose_covariance = [NAN] * 21
        pose_covariance[0] = cov[7]
        pose_covariance[6] = cov[0]

        with self.mav_lock:
            self.mav.mav.odometry_send(
                int(stamp.sec * 1_000_000 + stamp.nanosec / 1000),
                mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                x_ned, y_ned,
                NAN,
                [NAN, NAN, NAN, NAN],
                NAN, NAN, NAN,
                NAN, NAN, NAN,
                pose_covariance,
                [NAN] * 21,
                self.reset_counter,
                mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
                100,
            )
        if self._slam_msg_count % 10 == 1:
            print(f'[SEND] sent ODOMETRY to ArduPilot: x_ned={x_ned:.2f} y_ned={y_ned:.2f}', flush=True)


def main():
    print('[MAIN] rclpy.init() ...', flush=True)
    rclpy.init(args=sys.argv)
    print('[MAIN] creating node (this will block on wait_heartbeat if ArduPilot is unreachable) ...',
          flush=True)
    node = APSlamBridge()
    print('[MAIN] node created, spinning ...', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('[MAIN] KeyboardInterrupt, shutting down', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()