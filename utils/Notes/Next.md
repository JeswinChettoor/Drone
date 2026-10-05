Complete Step 1 Implementation Guide
1. Files to Create/Modify
File	Action
src/perception/package.xml	Modify - add dependencies
src/perception/setup.py	Modify - add entry point
src/perception/perception/camera_yolo_node.py	Create - main node
src/perception/config/nidar.yaml	Create - config
src/perception/launch/nidar_system.launch.py	Create - launch
2. Modified package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>perception</name>
  <version>0.0.0</version>Plan·Nemotron 3 Ultra FreeOpenCode Zen
  <description>NIDAR Perception - Person Detection with YOLO11m</description>
~/NIDAR:Survivor-detection  <maintainer email="jeswin.j.chettoor@gmail.com">developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>nidar_msgs</exec_depend>
  <exec_depend>ultralytics</exec_depend>
  <exec_depend>opencv-python</exec_depend>
  <exec_depend>numpy</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test
⚠ Retry due · attempt 4 · Provider reported an error (finish_reason: error)
Plan · Nemotron 3 Ultra Free · 17.9s
Thought: 1m 10s
The user wants me to continue from where I left off. Let me provide the complete implementation guide including the rest of the package.xml, setup.py, config, launch, and the full camera_yolo_node.py code.
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
3. Modified setup.py
from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nidar.yaml']),
        ('share/' + package_name + '/launch', ['launch/nidar_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='jeswin.j.chettoor@gmail.com',
    description='NIDAR Perception - Person Detection with YOLO11m',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'camera_yolo_node = perception.camera_yolo_node:main'
        ],
    },
)
4. Create config/nidar.yaml
camera_yolo:
  model_path: "/models/yolo11m.pt"
  confidence_threshold: 0.5
  iou_threshold: 0.45
  device: "auto"
  image_topic: "/camera/image_raw"
  detections_topic: "/person_detections"
  debug_image_topic: "/camera_yolo/debug_image"
  class_id: 0
5. Create launch/nidar_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('perception'), 'config', 'nidar.yaml'
    ])

    return LaunchDescription([
        Node(
            package='perception',
            executable='camera_yolo_node',
            name='camera_yolo_node',
            parameters=[config],
            output='screen',
        ),
    ])
6. Full camera_yolo_node.py
#!/usr/bin/env python3
"""
NIDAR Perception - Camera YOLO Node
Camera image -> YOLO11m -> timestamped person detections + debug visualization
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

from nidar_msgs.msg import PersonDetection, PersonDetectionArray


class CameraYoloNode(Node):
    def __init__(self):
        super().__init__('camera_yolo_node')

        # Declare parameters
        self.declare_parameter('model_path', '/models/yolo11m.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/person_detections')
        self.declare_parameter('debug_image_topic', '/camera_yolo/debug_image')
        self.declare_parameter('class_id', 0)

        # Get parameters
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.debug_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self.person_class_id = self.get_parameter('class_id').get_parameter_value().integer_value

        # CV Bridge
        self.bridge = CvBridge()

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model from: {self.model_path}')
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise

        # QoS for sensor data (best effort, keep last)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos
        )

        # Publishers
        self.detections_pub = self.create_publisher(
            PersonDetectionArray,
            self.detections_topic,
            10
        )
        self.debug_pub = self.create_publisher(
            Image,
            self.debug_topic,
            10
        )

        self.get_logger().info(f'Subscribed to: {self.image_topic}')
        self.get_logger().info(f'Publishing detections to: {self.detections_topic}')
        self.get_logger().info(f'Publishing debug images to: {self.debug_topic}')
        self.get_logger().info(f'Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'IoU threshold: {self.iou_threshold}')
        self.get_logger().info(f'Person class ID: {self.person_class_id}')

    def image_callback(self, msg: Image):
        """Process incoming camera image"""
        capture_stamp = msg.header.stamp  # PRESERVE ORIGINAL TIMESTAMP

        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Run YOLO inference
        try:
            results = self.model.predict(
                cv_image,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                device=self.device,
                verbose=False,
                classes=[self.person_class_id] if self.person_class_id >= 0 else None
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return

        # Build detection array
        detections = []
        debug_image = cv_image.copy()

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                # Only process person class
                if int(box.cls[0]) != self.person_class_id:
                    continue

                conf = float(box.conf[0])
                if conf < self.conf_threshold:
                    continue

                # Get bounding box coordinates (xyxy format)
                xyxy = box.xyxy[0].cpu().numpy()
                x_min, y_min, x_max, y_max = map(float, xyxy)

                # Create PersonDetection message
                det = PersonDetection()
                det.header.stamp = capture_stamp  # CRITICAL: preserve original timestamp
                det.header.frame_id = msg.header.frame_id
                det.confidence = conf
                det.x_min = x_min
                det.y_min = y_min
                det.x_max = x_max
                det.y_max = y_max
                detections.append(det)

                # Draw on debug image
                self._draw_detection(debug_image, x_min, y_min, x_max, y_max, conf)

        # Publish detection array
        array_msg = PersonDetectionArray()
        array_msg.header.stamp = capture_stamp
        array_msg.header.frame_id = msg.header.frame_id
        array_msg.detections = detections
        self.detections_pub.publish(array_msg)

        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header.stamp = capture_stamp
            debug_msg.header.frame_id = msg.header.frame_id
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')

    def _draw_detection(self, image: np.ndarray, x_min: float, y_min: float,
                        x_max: float, y_max: float, conf: float):
        """Draw bounding box, confidence, and bottom-center point on debug image"""
        h, w = image.shape[:2]

        # Convert to int
        x1, y1 = int(x_min), int(y_min)
        x2, y2 = int(x_max), int(y_max)

        # Clamp to image bounds
        x1 = max(0, min(x1, w - 1))
        y1 = max(0, min(y1, h - 1))
        x2 = max(0, min(x2, w - 1))
        y2 = max(0, min(y2, h - 1))

        # Draw bounding box (green)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Draw confidence label
        label = f'Person: {conf:.2f}'
        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(image, (x1, y1 - label_size[1] - 4), (x1 + label_size[0], y1), (0, 255, 0), -1)
        cv2.putText(image, label, (x1, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # Draw bottom-center point (red circle) - approximates foot contact
        bottom_center_x = (x1 + x2) // 2
        bottom_center_y = y2
        cv2.circle(image, (bottom_center_x, bottom_center_y), 5, (0, 0, 255), -1)
        cv2.circle(image, (bottom_center_x, bottom_center_y), 5, (255, 255, 255), 1)

        # Draw crosshair at bottom-center
        crosshair_len = 10
        cv2.line(image,
                 (bottom_center_x - crosshair_len, bottom_center_y),
                 (bottom_center_x + crosshair_len, bottom_center_y),
                 (0, 0, 255), 2)
        cv2.line(image,
                 (bottom_center_x, bottom_center_y - crosshair_len),
                 (bottom_center_x, bottom_center_y + crosshair_len),
                 (0, 0, 255), 2)


def main(args=None):
    rclpy.init(args=args)
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
7. Commands to Execute (in order)
# 1. Download YOLO11m model
mkdir -p /models
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m.pt -P /models/

# 2. Build messages first
cd /home/jeswin/NIDAR
colcon build --packages-select nidar_msgs --symlink-install
source install/setup.bash

# 3. Build perception package
colcon build --packages-select perception --symlink-install
source install/setup.bash

# 4. Run the detection node
ros2 launch perception nidar_system.launch.py
8. Verification Commands
# Terminal 1: Run the launch file
ros2 launch perception nidar_system.launch.py

# Terminal 2: Check detections (verify timestamp preservation)
ros2 topic echo /person_detections

# Terminal 3: View debug image with boxes
ros2 run rqt_image_view rqt_image_view /camera_yolo/debug_image

# Terminal 4: Check frequency
ros2 topic hz /person_detections

# Terminal 5: Verify timestamp matches (play a bag if available)
ros2 bag play test.bag --clock
ros2 topic echo /person_detections | grep stamp
9. Critical Verification Points
Check	Expected
Node starts without errors	✅ YOLO model loads
Subscribes to /camera/image_raw	✅
Publishes /person_detections	✅ PersonDetectionArray with correct structure
Timestamp preservation	Detection header.stamp == Image header.stamp
Publishes /camera_yolo/debug_image	✅ Annotated with boxes, conf, bottom-center points
Confidence threshold works	✅ Adjust via ros2 param set
No memory leaks	✅ Run 10+ minutes
10. Parameter Tuning (Runtime)
# Lower confidence threshold
ros2 param set /camera_yolo_node confidence_threshold 0.4

# Change IoU threshold
ros2 param set /camera_yolo_node iou_threshold 0.5

# Switch device
ros2 param set /camera_yolo_node device "cuda:0"
That's everything for Step 1. Once verified, you can proceed to Step 2 (Localization Node) which will need the additional messages added to nidar_msgs (LocalizedPerson.msg, Survivor.msg, SurvivorArray.msg).
Plan · Nemotron 3 Ultra Free · 1m 39s · 40.6 tok/s
No need to debug stuff now , just the other things. Make the rest as simple as possible. 
Thought: 49.5s
The user wants a simplified version - just the essential code and steps, no debug visualization, no extra verification commands. Let me provide a minimal, clean implementation.
Minimal Step 1 - Essential Only
1. src/perception/package.xml (replace entirely)
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>perception</name>
  <version>0.0.0</version>
  <description>NIDAR Person Detection</description>
  <maintainer email="jeswin.j.chettoor@gmail.com">developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>nidar_msgs</exec_depend>
  <exec_depend>ultralytics</exec_depend>
  <exec_depend>opencv-python</exec_depend>
  <exec_depend>numpy</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
2. src/perception/setup.py (replace entirely)
from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nidar.yaml']),
        ('share/' + package_name + '/launch', ['launch/nidar_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='jeswin.j.chettoor@gmail.com',
    description='NIDAR Person Detection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_yolo_node = perception.camera_yolo_node:main'
        ],
    },
)
3. src/perception/config/nidar.yaml (create)
camera_yolo:
  model_path: "/models/yolo11m.pt"
  confidence_threshold: 0.5
  iou_threshold: 0.45
  device: "auto"
  image_topic: "/camera/image_raw"
  detections_topic: "/person_detections"
  class_id: 0
4. src/perception/launch/nidar_system.launch.py (create)
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('perception'), 'config', 'nidar.yaml'])
    return LaunchDescription([
        Node(package='perception', executable='camera_yolo_node', name='camera_yolo_node',
             parameters=[config], output='screen'),
    ])
5. src/perception/perception/camera_yolo_node.py (create - minimal)
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from nidar_msgs.msg import PersonDetection, PersonDetectionArray

class CameraYoloNode(Node):
    def __init__(self):
        super().__init__('camera_yolo_node')
        self.declare_parameter('model_path', '/models/yolo11m.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/person_detections')
        self.declare_parameter('class_id', 0)

        p = self.get_parameter
        self.model = YOLO(p('model_path').value)
        self.conf = p('confidence_threshold').value
        self.iou = p('iou_threshold').value
        self.device = p('device').value
        self.class_id = p('class_id').value
        self.bridge = CvBridge()

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, p('image_topic').value, self.cb, qos)
        self.pub = self.create_publisher(PersonDetectionArray, p('detections_topic').value, 10)

    def cb(self, msg: Image):
        stamp = msg.header.stamp  # preserve
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model.predict(cv_img, conf=self.conf, iou=self.iou, device=self.device,
                                     verbose=False, classes=[self.class_id] if self.class_id >= 0 else None)

        arr = PersonDetectionArray()
        arr.header.stamp = stamp
        arr.header.frame_id = msg.header.frame_id

        for r in results:
            if r.boxes is None: continue
            for b in r.boxes:
                if int(b.cls[0]) != self.class_id: continue
                c = float(b.conf[0])
                if c < self.conf: continue
                x1, y1, x2, y2 = map(float, b.xyxy[0].cpu().numpy())
                d = PersonDetection()
                d.header.stamp = stamp
                d.header.frame_id = msg.header.frame_id
                d.confidence = c
                d.x_min, d.y_min, d.x_max, d.y_max = x1, y1, x2, y2
                arr.detections.append(d)

        self.pub.publish(arr)

def main():
    rclpy.init()
    rclpy.spin(CameraYoloNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
6. Run (3 commands)
mkdir -p /models && wget -q https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11m.pt -P /models/
cd /home/jeswin/NIDAR && colcon build --packages-select nidar_msgs perception --symlink-install && source install/setup.bash
ros2 launch perception nidar_system.launch.py
7. Verify (1 command)
ros2 topic echo /person_detections
Check: header.stamp matches input image stamp exactly.
That's it. 5 files, 3 commands.