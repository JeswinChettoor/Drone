import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener, TransformException
from pymavlink import mavutil


NAN = float('nan')
UNKNOWN_COVARIANCE = [NAN] + [0.0] * 20


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion (x, y, z, w) to roll, pitch, yaw."""

    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y)
    )

    sin_pitch = 2.0 * (w * y - z * x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)

    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
    )

    return roll, pitch, yaw


class APSlamBridge(Node):
    def __init__(self):
        super().__init__('ap_slam_bridge')

        self.declare_parameter(
            'connection_url', 'udp:127.0.0.1:15001'
        )
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('send_rate', 10.0)

        url = self.get_parameter('connection_url').value
        self.target_frame = self.get_parameter(
            'target_frame'
        ).value
        self.source_frame = self.get_parameter(
            'source_frame'
        ).value
        send_rate = float(
            self.get_parameter('send_rate').value
        )

        if send_rate <= 0.0:
            raise ValueError('send_rate must be greater than zero')

        # Connect to ArduPilot.
        self.mav = mavutil.mavlink_connection(url)
        self.mav.wait_heartbeat()

        self.get_logger().info(
            f'MAVLink connected to system {self.mav.target_system}'
        )

        # Receive TF transformations.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        self.tf_error_reported = False

        # Attempt a TF lookup and MAVLink transmission at 10 Hz.
        self.timer = self.create_timer(
            1.0 / send_rate,
            self.send_tf_pose
        )

        self.get_logger().info(
            f'Sending {self.target_frame} -> '
            f'{self.source_frame} pose at {send_rate} Hz'
        )

    def send_tf_pose(self):
        # Get the latest pose of base_link expressed in map.
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                Time()
            )
        except TransformException as exc:
            if not self.tf_error_reported:
                self.get_logger().warning(
                    f'Cannot look up TF: {exc}'
                )
                self.tf_error_reported = True
            return

        self.tf_error_reported = False

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp

        # Use the TF measurement timestamp in microseconds.
        timestamp_us = (
            stamp.sec * 1_000_000
            + stamp.nanosec // 1_000
        )

        x_ned = translation.x
        y_ned = -translation.y
        z_ned = -translation.z

        # Send the pose estimate to ArduPilot.
        self.mav.mav.vision_position_estimate_send(
            timestamp_us,
            x_ned,
            y_ned,
            z_ned,
            roll,
            pitch,
            yaw,
            UNKNOWN_COVARIANCE,
            0
        )

        #self.get_logger().info(
         #   f'NED position: '
          #  f'x={x_ned:.3f}, '
          #  f'y={y_ned:.3f}, '
          #  f'z={z_ned:.3f}',
          #  throttle_duration_sec=1.0
        #)


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