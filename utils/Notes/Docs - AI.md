The mission requirements are to autonomously go through a post disaster indoor environment without GPS or any external network , find survivors , tag their locations accurately , and return to the launch pad. 

```text
Sensors
├── 2D LiDAR ──────────────────────┐
├── Raspberry Pi Camera 3 ─────────┤
└── Onboard IMU                    │
      │                            │
      ▼                            ▼
Flight Controller ◄────────────────── Raspberry Pi 5 (Companion Computer)
├── ArduPilot (ArduCopter)     ├── ROS 2 (Humble)
│   ├── Internal IMU Fusion    ├── LiDAR Tilt Compensation +  Noise removal
│   ├── Downward Rangefinder   ├── RF2O Laser Odometry
							│  ├── SLAM Toolbox (Mapping + Localization
│   └── EKF3 State Estimation  ├── YOLO11m Inference (Target Detection)
└── Low-Level Flight Control   ├── Survivor Localization 
      │                        └── Frontier Exploration & Nav2 Stack
      │                                    │
      └─────────── UART / TELEM ───────────┘
                  (PyMAVLink Bridge)
                           │
                           ▼
                     Ground Station
      ├── Real-Time 2D Occupancy Map & Pose
      ├── Survivor Detection Markers
      ├── System Telemetry & Flight Modes
      └── Debugging & Visualization 
      --- Video feed 
```


 #### Localisation 

Since the mission objective is about localizing a drone without GPS , we are using an approach which combines our flight-controller with a 360 degree 2D LIDAR . We run SLAM on the companion computer , using SLAM_TOOLBOX to use scans from the 2D LIDAR to make a 2D map of the maze + give (X,Y) coordinates of where the drone is.
This X,Y coordinates is fed back into the flight controller  via MAVLINK VISION_POSITION_ESTIMATE message back into Ardupilot to act as a source of drift free localization. This is fused by ardupilot's ekf3 along with readings from IMU , compass , rangefinder on the drone.

Thus , we are able to get coordinates of drone at any point in time.
##### LIDAR Processing / Tilt Compensation.
Since slam_toolbox is meant for 2D ground robots , it does not natively account for the lidar scans from a tilted drone . To compensate for this , we are using the attitude of the drone from the onboard IMU , and projecting LIDAR points back to a 2D space and using that.  

**Survivor Detection**
To detect survivors in the maze , we are using Ultralytics YOLO 11m model on the camera feed coming from the two cameras we have on the drone.
Reasons for choosing YOLO 11m was excellent accuracy on detecting humans as seen on online benchmarks +  a light computational footprint compared .
The model gives us bounding boxes as pixels in the image .
We use the drone's position and camera intrinsics to get the position of the survivors on the map.

Survivor detection runs on a Raspberry Pi AI HAT+ (26 TOPS, Hailo-8) attached to the Pi 5, so YOLO11m inference is offloaded from the CPU. That leaves the CPU free for SLAM, Nav2 and the MAVLink bridge, and gives us a much higher detection frame rate than CPU-only inference

The same person can be detected in multiple frames , so we split detections into clusters based on distance from the mean of the cluster. For each new detection , we check if there is a cluster within a set radius from it , if yes , we add the new detected location to the cluster , then recompute the average of all points within the cluster .
If no cluster exists within that radius , we create a new cluster. 



(As of now , we haven't done any sort of fine tuning on dummies , will test out accuracy in real life and if info related to dummies is known , we can do that also )

(Later , we can add benchmarks on the FPS of the model , accuracy , and stuff like that . Don't have that right now  )




