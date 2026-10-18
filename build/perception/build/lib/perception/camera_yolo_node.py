"""
This node takes /camera/image_raw from ros2 , converts it into a numpy array format which can be used by ultralytics , runs object detection for 
class 0 (person) , gives bounding boxes and passes it .
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from nidar_msgs.msg import PersonDetection, PersonDetectionArray
import numpy as np
import cv2

class CameraYoloNode(Node):
    def __init__(self):
        super().__init__("camera_yolo_node")
        self.debug = True
        self.model = YOLO("yolo11m.pt")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,depth=1)
        self.subscription=self.create_subscription(Image,'/camera/image_raw',self.image_callback,qos)
        self.publisher=self.create_publisher(PersonDetectionArray,'/person_detections',10)
        if self.debug:
            self.debug_publisher = self.create_publisher(Image,'/camera/debug',10)
    # For debugging
    def image_callback(self,msg):
         image=np.frombuffer(msg.data,dtype=np.uint8)
         image=image.reshape(msg.height,msg.width,3)
         image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
         results = self.model.predict(image,conf=0.6,classes=[0],verbose=False)
         result=results[0]

         # Detection message
         output = PersonDetectionArray()
         output.header = msg.header
         for box in result.boxes:
            detection = PersonDetection()
            detection.header = msg.header
            detection.confidence = float(box.conf[0])
            detection.x_min = float(box.xyxy[0][0])
            detection.y_min = float(box.xyxy[0][1])
            detection.x_max = float(box.xyxy[0][2])
            detection.y_max = float(box.xyxy[0][3])
            output.detections.append(detection)

         self.publisher.publish(output)

    # DEBUGGING
         if self.debug:
            debug_image = np.ascontiguousarray(result.plot())
            debug_msg = Image()
            debug_msg.header = msg.header
            debug_msg.height, debug_msg.width = debug_image.shape[:2]
            debug_msg.encoding = 'bgr8'
            debug_msg.is_bigendian = False
            debug_msg.step = debug_image.strides[0]
            debug_msg.data = debug_image.tobytes()
            self.debug_publisher.publish(debug_msg)

def main():
    rclpy.init()
    node = CameraYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()