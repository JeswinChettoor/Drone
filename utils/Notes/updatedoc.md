# NIDAR — Project Guide

A single entry point to this notes vault. It explains what the project is,
how the pieces fit together, how to get the simulation running, and where to
find the deeper notes.

---

## 1. What This Project Is

**NIDAR** is an indoor-autonomy research project: a simulated drone that has
to **navigate a maze-like arena without GPS**.

Instead of relying on satellite positioning, the drone localizes itself using:

- a **2D LiDAR** mounted on the airframe
- **SLAM** (Simultaneous Localization and Mapping) via `slam_toolbox`
- the flight controller's onboard **EKF3** state estimator, fed by the IMU
  and barometer

The whole stack runs in **Docker** against **Gazebo** (the physics simulator)
and **ArduPilot SITL** (the autopilot running in software-in-the-loop), with
**ROS 2 Humbridge** gluing the simulator to the robotics code.

### The core idea

> Keep the simulator and autopilot as the source of truth for state
> estimation, and use ROS 2 only for LiDAR processing, mapping, and high-level
> navigation.

---

## 2. Tech Stack at a Glance

| Component | Role in the project |
|---|---|
| **ArduCopter SITL** | The autopilot — flight modes, EKF3 state estimation, arming logic, failsafes |
| **Gazebo Fortress** | Physics/world simulator — runs `combined_arena.sdf` (the maze arena) and the `gazebo-iris` drone model |
| **`ros_gz_bridge`** | Moves messages (LiDAR scans, clock, control) between Gazebo and ROS 2 |
| **`slam_toolbox`** | Builds the map and produces `odom → map` localization from the 2D LiDAR |
| **`rf2o_laser_odometry`** | Pure LiDAR odometry — provides `odom → base_link` |
| **`lidar_tilt_compensator`** | Projects the 2D scan onto the IMU plane so drone tilt doesn't distort the scan |
| **MAVROS / MAVLink** | ROS 2 ↔ ArduPilot communication channel |
| **EKF3 (in ArduPilot)** | Fuses IMU + barometer (+ optional external nav) into position/attitude |
| **Docker** (`nidar-container`) | Reproducible environment with ROS 2 Humble, ArduPilot, and Gazebo pre-wired |

---

## 3. How This Folder Is Organized

| Note | What it holds |
|---|---|
| **`updatedoc.md`** (this file) | The overview and quick-start |
| [`To run the simulation.md`](<To run the simulation.md>) | Raw command cheat-sheet for launching everything |
| [`Untitled 4/How to launch the world and drone.md`](<Untitled 4/How to launch the world and drone.md>) | Step-by-step first-time setup (clone → Docker → ArduPilot → launch) |
| [`Untitled 4/Getting stuff to work.md`](<Untitled 4/Getting stuff to work.md>) | The environment-variable / sourcing one-liners that fix broken shells |
| [`Untitled 4/Ardupilot Params.md`](<Untitled 4/Ardupilot Params.md>) | Exhaustive reference for every parameter touched |
| [`Untitled 4/Localization final.md`](<Untitled 4/Localization final.md>) | Static transforms and the ROS packages that were created |
| [`Untitled 4/Setting up  localization.md`](<Untitled 4/Setting up  localization.md>) | `apt install` commands and package scaffolding |
| [`Untitled 4/Localization + Maze Solving.md`](<Untitled 4/Localization + Maze Solving.md>) | Design notes — sensor choices, ToF/optical-flow ideas, maze solving |
| [`Untitled 4/Mavlink Based.md`](<Untitled 4/Mavlink Based.md>) | Running against Mission Planner over MAVLink |
| [`Untitled 4/Getting the drone model.md`](<Untitled 4/Getting the drone model.md>) | Gazebo/ArduPilot reference links |
| [`Untitled 4/Person Detection.md`](<Untitled 4/Person Detection.md>) | Research papers on vision-based person detection |
| [`Untitled 4/Updates.md`](<Untitled 4/Updates.md>) | Day-by-day changelog (Sep 7 – Sep 10) |
| `Yolo Comparison/`, `Untitled.md` | Empty placeholders |

---

## 4. One-Time Setup

Run these once on a fresh machine. Everything happens **inside the repo**,
and later steps happen **inside the Docker container**.

### 4.1 Clone the repository

```bash
git clone <repo-url>
cd NIDAR
```

### 4.2 Build and start Docker

```bash
xhost +local:root          # allow the container to use your display (Gazebo GUI)
docker compose build
docker compose up -d
```

### 4.3 Build ArduPilot and ArduPilot-Gazebo

Install the ROS 2 Humble + SLAM dependencies:

```bash
sudo apt update
sudo apt install ros-humble-tf2-ros ros-humble-tf2-tools
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-robot-state-publisher
sudo apt install mavros-extras
```

### 4.4 Install ArduPilot prerequisites

```bash
cd /workspace/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### 4.5 Put the tools on your PATH

```bash
export PATH=/workspace/ardupilot/Tools/autotest:$PATH
export PATH=$HOME/.local/bin:$PATH
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
```

### 4.6 Open a container terminal

```bash
docker exec -it nidar-container bash
```

> All commands in Section 5 are run from **inside** this container.

---

## 5. Running the Simulation (daily workflow)

Each numbered step runs in its own terminal. Start them **in order** —
later steps depend on earlier ones being up.

### Step 1 — Launch the world (terminal 1)

```bash
cd /workspace/SIM/Worlds && gz sim -v4 -r combined_arena.sdf
```

### Step 2 — Launch the autopilot (terminal 2)

```bash
cd /workspace/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w \
  --add-param-file=/workspace/no_gps.parm --console \
  --out=udpout:127.0.0.1:15000 --out=udpout:127.0.0.1:15001
```

- `-f gazebo-iris` selects the Gazebo Iris airframe
- `-w` wipes the prior flight log so runs are comparable
- `--add-param-file` loads the tuned parameters for this build
- `--out=...` streams MAVLink to the ground station / ROS

### Step 3 — Publish static transforms (terminal 3)

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 \
  base_link iris/lidar_link/lidar_2d

ros2 run tf2_ros static_transform_publisher 0.20 0.0 0.10 0 0 0 \
  base_link laser_link
# args: x y z  roll pitch yaw   parent child
```

The first tells ROS where the Gazebo LiDAR sits on the frame. The second is
the physical offset of the real LiDAR sensor from the centre of gravity.

### Step 4 — Bridge Gazebo ↔ ROS 2 (terminal 3)

```bash
ros2 launch gz_ros_bridge bridge.launch.py
```

### Step 5 — Stabilize the LiDAR scan (terminal 3)

```bash
ros2 launch lidar_tilt_compensator lidar_stabilization.launch.py
```

### Step 6 — Laser odometry (terminal 3)

```bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```

### Step 7 — SLAM (terminal 3)

```bash
ros2 launch drone_control slam.launch.py
```

### Step 8 — Bringup + visualization (terminal 3)

```bash
cd /workspace
ros2 launch drone_bringup drone_bringup.launch.py
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

> In RViz, set the robot model's base to `base_footprint`.

### Alternative: MAVLink / Mission Planner

To drive it from a ground station instead of ROS:

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w \
  --add-param-file=/workspace/no_gps.parm

ros2 launch gz_ros_bridge bridge.launch.py
```

Or via MAVROS directly:

```bash
ros2 launch mavros apm.launch \
  fcu_url:=udp://127.0.0.1:14550@127.0.0.1:14557 \
  use_sim_time:=true
```

---

## 6. The Localization Architecture

This is the part worth understanding before touching any code.

### The pipeline

```
Gazebo LiDAR scan
      │
      ▼
lidar_tilt_compensator     ← uses IMU roll/pitch to re-project the
      │                        2D scan onto the world plane
      ▼
scan_stabilized topic
      │
      ├──────────────────────────────┐
      ▼                              ▼
rf2o_laser_odometry           slam_toolbox
  → odom → base_link            → map → odom
      │                              │
      └──────────┬───────────────────┘
                 ▼
        TF tree:  map → odom → base_link
                 │
                 ▼
        ArduPilot EKF3  ← fuses IMU + barometer for attitude/height
```

### Why it's built this way (hard-won decisions)

- **Laser odometry was dropped as a separate fusion source.**
  Early on the project fused companion-computer odometry into the
  flight controller. It turned out to be unnecessary — see the
  [Sep 7 changelog](<Untitled 4/Updates.md>). The stack now trusts
  **EKF3 alone** for state estimation.

- **`rf2o` only publishes `odom → base_link`.**
  It is deliberately kept out of the `map` frame so SLAM remains the
  sole authority on global position.

- **No EKF fusion happens on the companion computer.**
  One estimator, one source of truth — this avoids the drift/feedback
  problems that come from running two estimators against each other.

- **Tilt is compensated in ROS, not in ArduPilot.**
  A 2D LiDAR tilted with the drone sees a warped world. The
  compensator projects each scan using the IMU attitude before SLAM
  ever sees it. ArduPilot separately handles tilt compensation for the
  rangefinder on its own side.

- **Height comes from the barometer, not the rangefinder.**
  In the real vehicle this is a single parameter change — ArduPilot
  already supports rangefinder-based height, but barometer was chosen
  as the baseline.

### Sensor upgrades under consideration

From the [design notes](<Untitled 4/Localization + Maze Solving.md>):

- **ToF sensor** — definitely wanted; high update rate, feeds the FC's EKF
- **Optical flow** — optional; would improve EKF accuracy but isn't required
- Fusing both would give better accuracy because of their high update rate

### Maze solving / navigation (planned)

- `slam_toolbox` handles SLAM and mapping
- Navigation layer to be built on top of the resulting map

---

## 7. Key ArduPilot Parameters (curated)

The full annotated list lives in
[`Ardupilot Params.md`](<Untitled 4/Ardupilot Params.md>). This is the
subset that actually matters for this build, grouped by purpose.

### EKF / state estimation

| Param | Setting | Why |
|---|---|---|
| `AHRS_EKF_TYPE` | `3` | Use EKF3 (modern filter, supports external nav) |
| `EK2_ENABLE` | `0` | Legacy EKF2 off — frees CPU/RAM |
| `EK3_ENABLE` | `1` | EKF3 on — the active estimator |
| `EK3_PRIMARY` | `0` | First IMU lane is primary; auto-switches on failure |
| `EK3_IMU_MASK` | `3` | IMU1 + IMU2 → dual-redundant lanes |
| `INS_USE` / `INS_USE2` | `1` | Both IMUs fed into the estimator |

### Position / velocity sources (the non-GPS part)

| Param | Setting | Meaning |
|---|---|---|
| `EK3_SRC1_POSXY` | `6` | Horizontal position from **External Nav** (SLAM/VIO) |
| `EK3_SRC1_VELXY` | `6` | Horizontal velocity from External Nav |
| `EK3_SRC1_POSZ` | `1` | **Vertical position from the barometer** |
| `EK3_SRC1_VELZ` | `0` | No vertical velocity source |
| `EK3_SRC1_YAW` | `1` | Heading from the compass |
| `EK3_SRC2_*` / `EK3_SRC3_*` | — | Backup/fallback sets, switchable via `RCx_OPTION = 90` |
| `AHRS_ORIGIN_LAT/LON/ALT` | — | Local datum for indoor (non-GPS) navigation |

### Arming

| Param | Notes |
|---|---|
| `ARMING_NEED_LOC` | `1` — a valid location (GPS or SLAM origin) must exist before arming |
| `ARMING_ACCTHRESH` | Accel consistency check; `0` disables, default `0.75 m/s²` |
| `ARMING_MAGTHRESH` | Compass consistency; default `100–150` mGauss |
| `ARMING_SKIPCHK` | Bitmask to bypass specific pre-arm checks (`1` Baro, `2` Compass, `4` GPS, `8` INS…) |
| `ARMING_OPTIONS` | Bitmask for arming behaviour quirks |
| `DISARM_DELAY` | Auto-disarm after landing; default `10 s` |

### Failsafes and geofence

| Param | Notes |
|---|---|
| `FS_EKF_ACTION` | `1` Land on EKF variance failure (default) |
| `FS_GCS_ENABLE` | Action on lost ground-station heartbeat — `1` RTL, `5` Land |
| `FENCE_ACTION` | `1` RTL (default), `2` Land, `4` Brake |
| `FENCE_RADIUS` | Max horizontal distance from home; default `300 m` |
| `FENCE_ALT_MAX` | Ceiling; default `100 m` |
| `FENCE_ALT_MAX_TP` | `1` AGL (default), `0` relative to home, `2` AMSL |

### Frame and flight behaviour

| Param | Notes |
|---|---|
| `FRAME_CLASS` | `1` Quad |
| `FRAME_TYPE` | `1` Cross (X) |
| `INITIAL_MODE` | Flight mode on boot |
| `ATC_ANGLE_MAX` | Max lean angle; default `3000–4500` centidegrees (30–45°) |
| `ATC_RATE_Y_MAX` | Max yaw rate; `0` = unconstrained, typically `45–200` deg/s |
| `LOIT_ANG_MAX` | Loiter lean override; `0` = use `ATC_ANGLE_MAX` |
| `WP_RADIUS_M` | Waypoint acceptance radius; default `2.0 m` |
| `WP_YAW_BEHAVIOR` | `2` face next waypoint except on RTL (default) |
| `WP_RFND_USE` | `1` use rangefinder for terrain following |

### Sensors and SITL simulation

| Param | Notes |
|---|---|
| `GPS1_TYPE` / `GPS2_TYPE` | `0` = None — **GPS disabled for indoor flight** |
| `FLOW_TYPE` | Optical-flow driver selection; `0` = none |
| `GND_EFFECT_COMP` | Baro ground-effect correction (on by default) |
| `SIM_FLOW_ENABLE` | `0` — synthetic optical flow off |
| `SIM_FLOW_RATE` | Simulated flow frame rate; default `10 Hz` |
| `SIM_FLOW_RND` | Noise injected into simulated flow; default `0.05 rad/s` |
| `SIM_IMU_COUNT` | Simulated IMUs; default `2` (matches `EK3_IMU_MASK`) |
| `SIM_IMU_ORIENT` | Board mounting rotation in sim; `0` = level |
| `SIM_MAG1_ORIENT` | Magnetometer mounting rotation; `0` = none |

---

## 8. Known Issues & Gotchas

These are the things that cost time — check here before debugging from scratch.

| Symptom | Cause | Fix |
|---|---|---|
| Messages dropped / TF errors | `use_sim_time` not set on every node | Pass `-p use_sim_time:=true` everywhere (including RViz) |
| Odometry jumps or lags | Scan rate (~3 Hz) higher than `rf2o` could handle | Lower the scan rate to match `rf2o`'s update speed |
| Shell can't find ROS Python modules | Container shell not sourced | Use the one-liner in Section 8.1 |
| Robot model misaligned in RViz | Wrong base frame | Set base to `base_footprint` |
| Rangefinder not used by ArduPilot | Wrong height-source param | One param change — ArduPilot handles rangefinder tilt compensation itself |

### 8.1 The "shell is broken" one-liner

When a fresh container terminal has no ROS environment at all:

```bash
export LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH}" && \
export PATH="/opt/ros/humble/bin:${PATH}" && \
export AMENT_PREFIX_PATH="/opt/ros/humble" && \
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH}" && \
source /opt/ros/humble/setup.bash && \
if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && \
ros2 launch drone_bringup drone_bringup.launch.py
```

### 8.2 Packages created for this project

| Package | Type | Purpose |
|---|---|---|
| `fcu_tf_bridge` | `ament_python` | Publishes the flight-controller → ROS transform |
| `lidar_tilt_compensator` | `ament_python` | Re-projects scans using IMU attitude |
| `lidar_preprocessing` | `ament_cmake` | C++ scan cleanup node |

---

## 9. Changelog

From [`Updates.md`](<Untitled 4/Updates.md>), normalized.

### September 7
- Realized separate laser odometry and the surrounding fusion machinery was not needed
- Removed it
- Changed params to rely **only** on EKF3 from the flight controller

### September 8
**Goal:** get `slam_toolbox` running and fly in Guided / Guided-No-GPS mode; make the rangefinder available to ArduPilot; handle conversions; install Mission Planner.

**Done:**
- Stripped back to only the `ros_gz` bridge
- Removed unneeded params
- Discussed Guided mode
- Fixed params to use the **barometer** for height instead of the rangefinder (one-param change in real life)
- Read `slam_toolbox` documentation
- Set up MAVROS again
- Confirmed laser odometry was unnecessary — just tune SLAM pose

### September 9
**Todo:** set up transforms, get `slam_toolbox` running.

**Done:**
- Redesigned the localization approach — `rf2o` now only publishes `odom → base_link`
- No EKF fusion on the companion computer
- Found a better tilt-compensation approach using pointclouds in ROS
- `scan_stabilized` running, transforms running, launch files written

### September 10
- Verified `scan_stabilized` accuracy
- Got `slam_toolbox` running
- Diagnosed message drops: sim time inconsistent across nodes, and scan rate (~3 Hz) too high for `rf2o` — lowered the scan to match

---

## 10. References

### Gazebo
- [Gazebo Sensors](https://gazebosim.org/libs/sensors/)
- [Gazebo Fortress — LiDAR](https://gazebosim.org/docs/fortress/lidar)
- [Gazebo Fortress — ROS 2 installation](https://gazebosim.org/docs/fortress/ros_installation)
- [Gazebo Fortress — ROS 2 integration](https://gazebosim.org/docs/fortress/ros2_integration)
- [SDF 1.9 sensor spec](https://sdformat.org/spec/1.9/sensor/)
- [`ros_gz` (Humble branch)](https://github.com/gazebosim/ros_gz/tree/humble)

### ArduPilot
- [SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- [`ardupilot_gazebo` (fortress branch)](https://github.com/ArduPilot/ardupilot_gazebo/tree/fortress)
- [EKF3 affinity & lane switching](https://ardupilot.org/copter/docs/common-ek3-affinity-lane-switching.html)
- [Non-GPS navigation](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html)
- [ROS 2 with ArduPilot](https://ardupilot.org/dev/docs/ros2.html)

### MAVLink / ROS
- [`mavros` (ROS 2 branch)](https://github.com/mavlink/mavros/tree/ros2)

### Person detection (research papers)
- [MDPI Sensors 24(3):922](https://www.mdpi.com/1424-8220/24/3/922)
- [MDPI Remote Sensing 12(20):3386](https://www.mdpi.com/2072-4292/12/20/3386)
- [MDPI Sensors 19(16):3542](https://www.mdpi.com/1424-8220/19/16/3542)
- [MDPI Sensors 21(6):2180](https://www.mdpi.com/1424-8220/21/6/2180)
