
#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil


class CmdVelToArduPilot(Node):

    def __init__(self):
        super().__init__("cmd_vel_to_ardupilot")

        # Configurable parameters
        self.declare_parameter(
            "connection_url", "udpin:0.0.0.0:14551"
        )
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("send_rate_hz", 20.0)
        self.declare_parameter("stale_timeout_sec", 0.5)
        self.declare_parameter("max_xy_speed", 0.5)
        self.declare_parameter("max_z_speed", 0.2)
        self.declare_parameter("max_yaw_rate", 0.3)

        self.connection_url = (
            self.get_parameter("connection_url")
            .get_parameter_value().string_value
        )
        self.cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic")
            .get_parameter_value().string_value
        )

        self.send_rate = float(
            self.get_parameter("send_rate_hz").value
        )
        self.stale_timeout = float(
            self.get_parameter("stale_timeout_sec").value
        )
        self.max_xy_speed = float(
            self.get_parameter("max_xy_speed").value
        )
        self.max_z_speed = float(
            self.get_parameter("max_z_speed").value
        )
        self.max_yaw_rate = float(
            self.get_parameter("max_yaw_rate").value
        )

        if self.send_rate <= 0.0:
            raise ValueError("send_rate_hz must be positive")

        if self.stale_timeout <= 0.0:
            raise ValueError("stale_timeout_sec must be positive")

        if min(
            self.max_xy_speed,
            self.max_z_speed,
            self.max_yaw_rate
        ) < 0.0:
            raise ValueError("Velocity limits cannot be negative")

        # Latest command received from ROS.
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        self.last_cmd_time = None
        self.last_stale_warning_time = 0.0

        # Establish the MAVLink connection.
        self.get_logger().info(
            f"Connecting to ArduPilot via {self.connection_url}"
        )

        self.mav = mavutil.mavlink_connection(
            self.connection_url,
            source_system=255,
            source_component=190
        )

        heartbeat = self.mav.wait_heartbeat(timeout=30)

        if heartbeat is None:
            self.mav.close()
            raise RuntimeError(
                "No ArduPilot heartbeat received within 30 seconds. "
                "Check the MAVLink connection URL and SITL/vehicle."
            )

        self.target_system = self.mav.target_system
        self.target_component = self.mav.target_component

        self.get_logger().info(
            "Heartbeat received: "
            f"system={self.target_system}, "
            f"component={self.target_component}"
        )

        # Subscribe to Nav2's velocity command.
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Resend setpoints periodically.
        self.timer = self.create_timer(
            1.0 / self.send_rate,
            self.send_setpoint
        )

        self.get_logger().info(
            f"Listening to {self.cmd_vel_topic}; "
            f"sending setpoints at {self.send_rate} Hz"
        )

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def cmd_vel_callback(self, msg: Twist):
        """
        Assumes the incoming Twist is expressed in the
        drone's body frame using ROS FLU convention:

        +X = forward
        +Y = left
        +Z = up
        +angular.Z = counter-clockwise yaw rate
        """

        # ROS FLU -> ArduPilot body forward/right/down.
        self.vx = self.clamp(
            msg.linear.x, self.max_xy_speed
        )
        self.vy = self.clamp(
            -msg.linear.y, self.max_xy_speed
        )
        self.vz = self.clamp(
            -msg.linear.z, self.max_z_speed
        )
        self.yaw_rate = self.clamp(
            -msg.angular.z, self.max_yaw_rate
        )

        self.last_cmd_time = time.monotonic()

    def send_setpoint(self):
        # Do not send movement commands before the first
        # actual ROS command has arrived.
        if self.last_cmd_time is None:
            return

        age = time.monotonic() - self.last_cmd_time

        if age > self.stale_timeout:
            # Never keep forwarding a stale nonzero command.
            vx = 0.0
            vy = 0.0
            vz = 0.0
            yaw_rate = 0.0

            now = time.monotonic()
            if now - self.last_stale_warning_time > 2.0:
                self.get_logger().warning(
                    "cmd_vel is stale; sending zero-velocity "
                    "setpoints. Check Nav2 and the command topic."
                )
                self.last_stale_warning_time = now
        else:
            vx = self.vx
            vy = self.vy
            vz = self.vz
            yaw_rate = self.yaw_rate

        # Ignore position (bits 0-2), acceleration (bits 6-8),
        # and yaw angle (bit 10).
        # Keep velocity (bits 3-5) and yaw rate (bit 11) active.
        type_mask = 1479

        try:
            self.mav.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms; zero used by common examples
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                type_mask,

                0.0, 0.0, 0.0,       # position, ignored
                vx, vy, vz,          # velocity targets, m/s
                0.0, 0.0, 0.0,       # acceleration, ignored
                0.0,                 # yaw angle, ignored
                yaw_rate              # yaw rate, rad/s
            )

        except Exception as exc:
            self.get_logger().error(
                f"Failed to send MAVLink setpoint: {exc}"
            )

    def destroy_node(self):
        try:
            self.mav.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CmdVelToArduPilot()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        print(f"Node startup/runtime error: {exc}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
