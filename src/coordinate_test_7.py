import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from pymavlink import mavutil

NAN = float('nan')
UNKNOWN_COVARIANCE = [NAN] + [0.0] * 20

class APSlamBridge(Node):
    def __init__(self):
        super().__init__('ap_slam_bridge')

        self.declare_parameter('connection_url', 'udp:127.0.0.1:15001')
        self.declare_parameter('rate_hz', 20.0)

        url = self.get_parameter('connection_url').value
        self.mav = mavutil.mavlink_connection(url)
        self.mav.wait_heartbeat()
        self.get_logger().info(f'MAVLink connected to system {self.mav.target_system}')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_stamp = None

        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.on_timer)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return

        stamp = t.header.stamp
        if self.last_stamp == (stamp.sec, stamp.nanosec):
            return  
        self.last_stamp = (stamp.sec, stamp.nanosec)

        p = t.transform.translation
        x_ned = p.x
        y_ned = -p.y
        z_ned = -p.z

        self.mav.mav.vision_position_estimate_send(
            int(stamp.sec * 1_000_000 + stamp.nanosec / 1000),
            x_ned, y_ned, z_ned,
            0.0, 0.0, 0.0,
            UNKNOWN_COVARIANCE,
            0
        )
        print(x_ned, y_ned, z_ned)

def main():
    rclpy.init()
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