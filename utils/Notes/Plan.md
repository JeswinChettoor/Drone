NIDAR Airmouse - Perception Pipeline Architecture Plan
Three-Node Evidence-Based Survivor Localization System
SYSTEM OVERVIEW
Three ROS 2 nodes forming a linear pipeline:
CAMERA → camera_yolo_node → survivor_localizer_node → survivor_evidence_node
              ↓                    ↓                        ↓
        /person_detections  /localized_person_observation  /confirmed_survivors
                                                         /survivor_grid
Node Responsibilities:
1. camera_yolo_node - Perception: Image → YOLO11m → timestamped person detections
2. survivor_localizer_node - Geometry: Detections + Camera Model + TF → Map coordinates
3. survivor_evidence_node - Decision: Map coordinates → Grid cells → Evidence → Confirmed survivors
PACKAGE STRUCTURE (2 Packages)
nidar_ws/
├── src/
│   ├── nidar_msgs/                    # CMake package - Custom messages
│   │   ├── msg/
│   │   │   ├── PersonDetection.msg
│   │   │   ├── PersonDetectionArray.msg
│   │   │   ├── LocalizedPerson.msg
│   │   │   ├── Survivor.msg
│   │   │   └── SurvivorArray.msg
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── nidar_survivor/                # Python package - All 3 nodes
│       ├── nidar_survivor/
│       │   ├── __init__.py
│       │   ├── camera_yolo_node.py
│       │   ├── survivor_localizer_node.py
│       │   └── survivor_evidence_node.py
│       ├── config/
│       │   └── nidar.yaml
│       ├── launch/
│       │   └── nidar_system.launch.py
│       ├── setup.py
│       ├── setup.cfg
│       └── package.xml
MESSAGE DEFINITIONS (nidar_msgs)
PersonDetection.msg
std_msgs/Header header
float32 confidence
float32 x_min
float32 y_min
float32 x_max
float32 y_max
PersonDetectionArray.msg
std_msgs/Header header
PersonDetection[] detections
LocalizedPerson.msg
std_msgs/Header header
float32 map_x
float32 map_y
float32 detection_confidence
Survivor.msg
std_msgs/Header header
int32 grid_x
int32 grid_y
float32 confidence
string status    # "UNKNOWN", "SUSPECTED", "CONFIRMED"
SurvivorArray.msg
std_msgs/Header header
Survivor[] survivors
CONFIGURATION (nidar.yaml)
camera_yolo:
  model_path: "/models/yolo11m.pt"
  confidence_threshold: 0.5
  iou_threshold: 0.45
  device: "auto"
  image_topic: "/camera/image_raw"
  detections_topic: "/person_detections"
  debug_image_topic: "/camera_yolo/debug_image"
  class_id: 0          # COCO person class

survivor_localizer:
  camera_info_topic: "/camera/camera_info"
  detections_topic: "/person_detections"
  output_topic: "/localized_person_observation"
  camera_height: 0.35
  min_ground_distance: 0.5
  max_ground_distance: 20.0
  map_frame: "map"
  camera_frame: "camera_link"
  camera_optical_frame: "camera_optical_frame"
  tf_timeout_ms: 50

survivor_evidence:
  observation_topic: "/localized_person_observation"
  grid_topic: "/survivor_grid"
  confirmed_topic: "/confirmed_survivors"
  grid_origin_x: 0.0
  grid_origin_y: 0.0
  cell_size: 1.0
  detection_threshold: 0.6
  confirmation_threshold: 3
STEP 1: DETECTION NODE WITH DEBUG
Goal: Camera image → YOLO11m → timestamped detections + debug visualization
1.1 Node: camera_yolo_node.py
Inputs:
- /camera/image_raw (sensor_msgs/Image)
- /camera/camera_info (sensor_msgs/CameraInfo) - for debug overlay
Outputs:
- /person_detections (PersonDetectionArray)
- /camera_yolo/debug_image (sensor_msgs/Image) - annotated debug stream
Core Logic:
on_image_received(msg):
    capture_stamp = msg.header.stamp          # PRESERVE THIS
    cv_image = bridge.imgmsg_to_cv2(msg)
    
    results = yolo_model.predict(cv_image, conf=threshold, iou=iou_threshold)
    
    detections = []
    for box in results.boxes:
        if box.cls == person_class_id:
            det = PersonDetection()
            det.header.stamp = capture_stamp   # ORIGINAL TIMESTAMP
            det.confidence = float(box.conf)
            det.x_min = float(box.xyxy[0][0])
            det.y_min = float(box.xyxy[0][1])
            det.x_max = float(box.xyxy[0][2])
            det.y_max = float(box.xyxy[0][3])
            detections.append(det)
    
    # Publish detections
    array_msg = PersonDetectionArray()
    array_msg.header.stamp = capture_stamp
    array_msg.detections = detections
    pub.publish(array_msg)
    
    # Publish debug image
    debug_img = draw_boxes(cv_image, detections)
    debug_msg = bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
    debug_msg.header.stamp = capture_stamp
    debug_pub.publish(debug_msg)
Key Requirements:
- Timestamp preservation: output header.stamp == input image header.stamp
- Single-threaded executor (no queue, natural backpressure)
- YOLO11m model loaded once at startup
- Configurable confidence/IoU thresholds
- Debug image shows: bounding boxes, confidence scores, bottom-center points
1.2 Launch Integration
Node(
    package='nidar_survivor',
    executable='camera_yolo_node',
    name='camera_yolo_node',
    parameters=[config_file],
    output='screen',
    remappings=[
        ('/camera/image_raw', '/camera/image_raw'),
        ('/person_detections', '/person_detections'),
    ]
)
1.3 Step 1 Verification Checklist
- Node starts, loads YOLO11m model
- Subscribes to /camera/image_raw
- Publishes /person_detections with correct message structure
- Critical: Detection header.stamp == image header.stamp (not inference completion time)
- Debug image topic publishes annotated frames
- RViz shows: raw image + debug image side by side
- Confidence threshold parameter works at runtime
- No memory leaks over 10+ minutes
STEP 2: LOCALIZATION NODE
Goal: Detections + Camera Geometry + TF → Map coordinates
2.1 Node: survivor_localizer_node.py
Inputs:
- /person_detections (PersonDetectionArray)
- /camera/camera_info (sensor_msgs/CameraInfo)
- /tf, /tf_static (TF2 transforms)
Outputs:
- /localized_person_observation (LocalizedPerson)
Core Logic:
on_detections_received(array_msg):
    # Get camera model from CameraInfo (cached after first receipt)
    if not camera_model_ready:
        wait_for_camera_info()
    
    for det in array_msg.detections:
        # 1. Bottom-center pixel
        u = (det.x_min + det.x_max) / 2.0
        v = det.y_max
        
        # 2. Undistort pixel → normalized ray in camera optical frame
        ray = camera_model.project_pixel_to_3d_ray((u, v))
        # ray is (x, y, 1.0) in optical frame (Z forward)
        
        # 3. Intersect with ground plane
        # Ground plane in base_link: z = -camera_height
        # Transform ray to base_link frame via camera extrinsics
        ray_base = transform_ray_to_base_link(ray)
        ground_point_base = intersect_ray_ground(ray_base, camera_height)
        
        if ground_point_base is None:
            continue  # Invalid projection
        
        # 4. Transform to map frame AT DETECTION TIMESTAMP
        try:
            transform = tf_buffer.lookup_transform(
                'map', 'base_link',
                det.header.stamp,
                timeout=Duration(milliseconds=50)
            )
            ground_point_map = do_transform_point(ground_point_base, transform)
        except TransformException:
            continue  # Drop this detection
        
        # 5. Validate
        if not (min_dist < ground_point_map.distance < max_dist):
            continue
        
        # 6. Publish
        obs = LocalizedPerson()
        obs.header.stamp = det.header.stamp
        obs.map_x = ground_point_map.x
        obs.map_y = ground_point_map.y
        obs.detection_confidence = det.confidence
        pub.publish(obs)
Camera Geometry Details:
- Use image_geometry.PinholeCameraModel from /camera/camera_info
- project_pixel_to_3d_ray() handles distortion automatically
- Camera optical frame: X right, Y down, Z forward
- Static transform base_link → camera_link must exist in TF (from launch)
- Ground plane intersection: ray param t where z = -camera_height in base_link
Key Requirements:
- TF lookup at detection timestamp, not current time
- CameraInfo cached after first message
- Reject: near-horizontal rays, behind-camera intersections, out-of-range
- Output timestamp = original detection timestamp
2.2 Launch Integration
# Static camera extrinsic (measure once)
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0.10', '0.0', '0.05', '-0.2618', '0.0', '0.0',
               'base_link', 'camera_link']
),

Node(
    package='nidar_survivor',
    executable='survivor_localizer_node',
    name='survivor_localizer_node',
    parameters=[config_file],
    output='screen'
)
2.3 Step 2 Verification Checklist
- Node receives detections, publishes localized observations
- Map coordinates visually match person positions in RViz (MarkerArray on map frame)
- TF lookup uses detection timestamp (test with moving drone bag)
- Ground plane intersection math verified with known positions
- Invalid projections rejected (no NaN, no extreme distances)
- Camera height parameter correctly set for platform
STEP 3: EVIDENCE ACCUMULATION NODE
Goal: Map observations → 1m grid → evidence → confirmed survivors
3.1 Node: survivor_evidence_node.py
Inputs:
- /localized_person_observation (LocalizedPerson)
Outputs:
- /survivor_grid (SurvivorArray) - all cells with evidence (1 Hz)
- /confirmed_survivors (SurvivorArray) - only CONFIRMED cells (on change)
Core Logic:
class EvidenceMap:
    def __init__(self, origin_x, origin_y, cell_size):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.cell_size = cell_size
        self.cells = {}  # (gx, gy) -> {'count': int, 'last_stamp': Time, 'status': str}
    
    def to_grid(self, map_x, map_y):
        gx = int(math.floor((map_x - self.origin_x) / self.cell_size))
        gy = int(math.floor((map_y - self.origin_y) / self.cell_size))
        return (gx, gy)
    
    def add_observation(self, map_x, map_y, confidence, stamp, det_threshold, conf_threshold):
        if confidence < det_threshold:
            return
        
        cell = self.to_grid(map_x, map_y)
        if cell not in self.cells:
            self.cells[cell] = {'count': 0, 'last_stamp': stamp, 'status': 'UNKNOWN'}
        
        # Simple count-based evidence (V1)
        self.cells[cell]['count'] += 1
        self.cells[cell]['last_stamp'] = stamp
        
        if self.cells[cell]['count'] >= conf_threshold:
            self.cells[cell]['status'] = 'CONFIRMED'
        elif self.cells[cell]['count'] >= 1:
            self.cells[cell]['status'] = 'SUSPECTED'
    
    def get_all_cells(self):
        return [(gx, gy, data['count'], data['status']) 
                for (gx, gy), data in self.cells.items()]
    
    def get_confirmed(self):
        return [(gx, gy, data['count']) 
                for (gx, gy), data in self.cells.items() 
                if data['status'] == 'CONFIRMED']
on_observation_received(msg):
    evidence_map.add_observation(
        msg.map_x, msg.map_y, 
        msg.detection_confidence, 
        msg.header.stamp,
        detection_threshold, 
        confirmation_threshold
    )

timer_callback_1hz():
    # Publish full grid for visualization
    grid_msg = SurvivorArray()
    grid_msg.header.stamp = now()
    for gx, gy, count, status in evidence_map.get_all_cells():
        s = Survivor()
        s.header.stamp = grid_msg.header.stamp
        s.grid_x = gx
        s.grid_y = gy
        s.confidence = min(1.0, count / confirmation_threshold)
        s.status = status
        grid_msg.survivors.append(s)
    grid_pub.publish(grid_msg)

    # Publish confirmed only (for downstream consumers)
    confirmed = evidence_map.get_confirmed()
    if confirmed_changed_since_last_publish():
        conf_msg = SurvivorArray()
        conf_msg.header.stamp = now()
        for gx, gy, count in confirmed:
            s = Survivor()
            s.grid_x = gx
            s.grid_y = gy
            s.confidence = min(1.0, count / confirmation_threshold)
            s.status = 'CONFIRMED'
            conf_msg.survivors.append(s)
        confirmed_pub.publish(conf_msg)
Key Requirements:
- Grid quantization: floor((map_x - origin) / cell_size)
- Evidence = simple count (V1), threshold = 3
- States: UNKNOWN (0) → SUSPECTED (1-2) → CONFIRMED (3+)
- Grid published at 1 Hz, confirmed published on change
- No temporal filtering, no spatial suppression in V1
3.2 Launch Integration
Node(
    package='nidar_survivor',
    executable='survivor_evidence_node',
    name='survivor_evidence_node',
    parameters=[config_file],
    output='screen'
)
3.3 Step 3 Verification Checklist
- Node receives observations, quantizes to grid correctly
- Evidence counts increment per cell
- State transitions: UNKNOWN → SUSPECTED → CONFIRMED at threshold
- /survivor_grid shows all cells with counts and status
- /confirmed_survivors shows only CONFIRMED cells
- Full pipeline test: fly drone over 6 dummies → 6 confirmed cells
- No duplicate confirmed cells (one per grid cell)
LAUNCH FILE: nidar_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('nidar_survivor'), 'config', 'nidar.yaml'
    ])
    
    return LaunchDescription([
        # Static camera extrinsic (measure once for platform)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_extrinsic',
            arguments=['0.10', '0.0', '0.05', '-0.2618', '0.0', '0.0',
                       'base_link', 'camera_link'],
        ),
        
        # Step 1: Detection
        Node(
            package='nidar_survivor',
            executable='camera_yolo_node',
            name='camera_yolo_node',
            parameters=[config],
            output='screen',
        ),
        
        # Step 2: Localization
        Node(
            package='nidar_survivor',
            executable='survivor_localizer_node',
            name='survivor_localizer_node',
            parameters=[config],
            output='screen',
        ),
        
        # Step 3: Evidence
        Node(
            package='nidar_survivor',
            executable='survivor_evidence_node',
            name='survivor_evidence_node',
            parameters=[config],
            output='screen',
        ),
    ])
DEVELOPMENT WORKFLOW
Build Commands
# Build messages first
colcon build --packages-select nidar_msgs --symlink-install
source install/setup.bash

# Build survivor nodes
colcon build --packages-select nidar_survivor --symlink-install
source install/setup.bash
Run Commands
# Full pipeline
ros2 launch nidar_survivor nidar_system.launch.py

# Individual nodes (for debugging)
ros2 run nidar_survivor camera_yolo_node --ros-args -p model_path:=/models/yolo11m.pt
ros2 run nidar_survivor survivor_localizer_node
ros2 run nidar_survivor survivor_evidence_node
Debug Topics
# Step 1
ros2 topic echo /person_detections
ros2 topic hz /person_detections
ros2 run rqt_image_view rqt_image_view /camera_yolo/debug_image

# Step 2
ros2 topic echo /localized_person_observation
ros2 run rviz2 rviz2 -d config/localizer_debug.rviz  # Shows map points

# Step 3
ros2 topic echo /survivor_grid
ros2 topic echo /confirmed_survivors
Parameter Tuning (Live)
ros2 param set /camera_yolo_node confidence_threshold 0.4
ros2 param set /survivor_evidence_node confirmation_threshold 2
ros2 param set /survivor_evidence_node detection_threshold 0.5
COORDINATE FRAME CONVENTIONS
camera_optical_frame (ROS standard):
  X → right, Y → down, Z → forward (into scene)

camera_link (mount frame):
  Defined by static transform from base_link

base_link (drone body):
  X → forward, Y → left, Z → up (FLU)

odom → map (SLAM):
  2D pose, Z=0, yaw only
Ray Direction: In optical frame, ray = normalize(u-cx, v-cy, fx) ≈ x, y, 1.0
Ground Plane: In base_link frame, plane at z = -camera_height
CRITICAL IMPLEMENTATION NOTES
1. Timestamp Preservation - camera_yolo_node MUST copy image header.stamp to detection header.stamp. This is the single most important requirement.
2. TF at Detection Time - survivor_localizer_node MUST use detection.header.stamp for TF lookup, not current time.
3. Bottom-Center Pixel - Use v = y_max (bottom of bbox), not box center. This approximates foot contact point.
4. Camera Height - Measure from floor to camera optical center, not mounting point.
5. Grid Origin - Keep at (0,0) = map origin. Change only if map has known offset.
6. Static Transform - base_link → camera_link must be accurate. Measure with AprilTag on floor.
VERIFICATION SEQUENCE
After Step 1:
ros2 bag play test.bag --clock
ros2 topic echo /person_detections | grep stamp
# Verify: stamp matches bag message stamp, not wall clock
After Step 2:
rviz2 -d localizer_debug.rviz
# Add: /localized_person_observation as PoseArray on map frame
# Verify: Points appear at correct map locations
After Step 3:
ros2 topic echo /confirmed_survivors
# After full survey flight: Should show exactly 6 cells
# Grid coordinates should be stable across revisits
DEPENDENCIES
nidar_msgs (package.xml):
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
nidar_survivor (package.xml):
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>tf2_geometry_msgs</exec_depend>
<exec_depend>cv_bridge</exec_depend>
<exec_depend>image_geometry</exec_depend>
<exec_depend>nidar_msgs</exec_depend>
<exec_depend>ultralytics</exec_depend>
<exec_depend>opencv-python</exec_depend>
<exec_depend>numpy</exec_depend>
SUCCESS CRITERIA (END OF ALL 3 STEPS)
1. Detection: >90% recall on person dummies at 5-15m, <5% false positive rate
2. Localization: Median error <0.5m on static test positions
3. Evidence: All 6 survivors → CONFIRMED within 2 passes each
4. No Duplicates: Exactly 6 confirmed cells, no adjacent-cell splitting
5. Latency: Image capture → confirmed survivor update <500ms
6. Stability: Runs 30+ minutes without memory leaks or crashes
FILE LIST SUMMARY
nidar_msgs/
├── msg/PersonDetection.msg
├── msg/PersonDetectionArray.msg
├── msg/LocalizedPerson.msg
├── msg/Survivor.msg
├── msg/SurvivorArray.msg
├── CMakeLists.txt
└── package.xml

nidar_survivor/
├── nidar_survivor/
│   ├── __init__.py
│   ├── camera_yolo_node.py
│   ├── survivor_localizer_node.py
│   └── survivor_evidence_node.py
├── config/nidar.yaml
├── launch/nidar_system.launch.py
├── setup.py
├── setup.cfg
└── package.xml
Total: 14 files to create. Start with nidar_msgs, then Step 1 node, verify, Step 2, verify, Step 3, verify.