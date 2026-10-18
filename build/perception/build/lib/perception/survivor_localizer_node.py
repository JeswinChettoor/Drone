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
 
CAMERA_FRAME = ""        
TF_TIMEOUT_SEC = 5
 
 
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
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pub = self.create_publisher(PoseStamped, "/localized_person", 10)
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.on_info, qos_profile_sensor_data
        )
        self.create_subscription(
            PersonDetectionArray, "/person_detections", self.on_detection, 10
        )
    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros(5)
        if not CAMERA_FRAME:
            self.frame = msg.header.frame_id

    def on_detection(self, msg):
        if not msg.detections or self.K is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", self.frame, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=TF_TIMEOUT_SEC),
            )
        except TransformException as e:
            self.get_logger().warning(f"TF unavailable: {e}", throttle_duration_sec=5.0)
            return
 
        t, q = tf.transform.translation, tf.transform.rotation
 
        for d in msg.detections:
            u, v = (d.x_min + d.x_max) / 2.0, d.y_max
            x, y = cv2.undistortPoints(np.array([[[u, v]]], np.float64), self.K, self.D)[0, 0]
            ray = rotate(q, (x, y, 1.0))  # optical frame: z forward, x right, y down
 
            if ray[2] >= -1e-6:  # not pointing at the ground
                continue
            s = -t.z / ray[2]  # distance along the ray to z = 0
            if s <= 0.0:
                continue
 
            point = PoseStamped()
            point.header.stamp = msg.header.stamp
            point.header.frame_id = "map"
            point.pose.position.x = t.x + s * ray[0]
            point.pose.position.y = t.y + s * ray[1]
            point.pose.position.z = 0.0
            point.pose.orientation.w = 1.0
            self.pub.publish(point)

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