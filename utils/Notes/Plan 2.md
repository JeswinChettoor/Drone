

## 1. Goal

From the running simulation stack, produce **confirmed survivor (dummy) locations in the
`map` frame** as a `geometry_msgs/PoseArray`, one launch file, minimal surface area.

**Output contract**

| Topic | Type | Meaning |
|---|---|---|
| `/localized_person_observation` | `geometry_msgs/PoseStamped` | one ground point per detection, `frame_id=map` |
| `/survivor_grid` | `geometry_msgs/PoseArray` | debug: every grid cell with ≥1 vote (cell centers) |
| `/confirmed_survivors` | `geometry_msgs/PoseArray` | **final answer**: confirmed cell centers, `frame_id=map` |

**Explicitly out of scope** (do not do these):
- Do NOT modify `src/perception/perception/camera_yolo_node.py` (existing Step-1 node, used as-is).
- Do NOT create YAML param files. All tunables are hardcoded constants at the top of each node.
- Do NOT touch `src/nidar_msgs` (it already has `PersonDetection`/`PersonDetectionArray` built; we use `geometry_msgs` for outputs instead of new messages).
- No rviz config, no persistence, no status enums.

---

## 2. Existing architecture (what already runs — do not rebuild)

```
Gazebo world
  │  gz topics
  ▼
gz_ros_bridge  (config: src/gz_ros_bridge/config/bridge.yaml)
  bridges: /camera/image_raw, /camera/camera_info, /clock, /scan  (no rangefinder!)
  │
  ├──▶ camera_yolo_node  (src/perception/perception/camera_yolo_node.py)   [EXISTS — leave alone]
  │        publishes /person_detections  (nidar_msgs/PersonDetectionArray)
  │        hardcodes YOLO("yolo11m.pt") relative to cwd → run from repo root
  │
  └──▶ rf2o_laser_odometry + slam_toolbox
         TF chain: map → odom → base_link   (yaw-only 2D; z is always 0)
         map→base_link therefore gives drone x, y, yaw in map

ArduPilot SITL (MAVLink) ── runbook: utils/Notes/To run the simulation.md
  sim_vehicle.py ... --out=udpout:127.0.0.1:15000 --out=udpout:127.0.0.1:15001
  15000 is already consumed by src/lidar_processing/lidar_processing/lidar_tilt_stabilizer_node.py
  15001 is FREE — the new localizer uses it.
```

### Facts you can rely on (verified)

- **Camera** (`utils/Model files/iris_with_ardupilot/model.sdf`):
  - Pose in `base_link`: `x=0.12, z=-0.05`, rotation `pitch=+0.7854` (45° down, about +Y so
    +X tips toward −Z).
  - Image: 640×480, `hfov=1.3962634`, topic `/camera/image_raw`.
  - CameraInfo (from recorded bag `camera_test_2/`): `fx ≈ 381.36`, `cx = 320`, `cy = 240`,
    `D = [0,0,0,0,0]` (zero distortion → ignore D). `fy` read from `K[4]` (square pixels, ≈ fx).
  - Image `frame_id` = `iris/front_camera_link/front_down_camera` (do NOT depend on it; we
    don't TF from the image frame — camera extrinsics are code constants).
- **Altitude**: no rangefinder is bridged. The z source is **ArduPilot EKF3 height via
  MAVLink**: `GLOBAL_POSITION_INT.relative_alt` — **units are millimetres** (`/1000.0`).
- **Attitude**: `ATTITUDE` message, radians. FRD → FLU conversion (same as lidar node):
  `roll = msg.roll`, `pitch = -msg.pitch`.
- **MAVLink port**: pymavlink `mavutil.mavlink_connection("udp:host:port")` defaults to
  `input=True`, i.e. it **binds the local port** as a listener. So `"udp:127.0.0.1:15001"`
  binds local 15001 and receives what SITL sends via its second `--out`. Port 15000 must
  not be used (lidar node has it).
- **TF**: ground must be intersected at `z = -camera_height`, never at a fixed camera
  height, because `map→base_link` from this SLAM chain is yaw-only 2D (z always 0).
  Get drone x/y/yaw from `map → base_link` TF; get camera height above ground from
  `relative_alt`.
- **Detection timestamping**: image stamps are **sim time** (`/clock`), so every node in
  this plan must run with `use_sim_time=true` or TF lookups at those stamps fail.
- Existing package style: `ament_python`, nodes declared as `console_scripts` in
  `src/perception/setup.py`; `data_files` there was reduced to just `package.xml` and must
  get the launch entry back.

---

## 3. Things to build (5 files)

| File | Action |
|---|---|
| `src/perception/perception/survivor_localizer_node.py` | **create** |
| `src/perception/perception/survivor_evidence_node.py` | **create** |
| `src/perception/launch/nidar_pipeline.launch.py` | **create** |
| `src/perception/setup.py` | **edit**: 2 entry points + launch install |
| `src/perception/package.xml` | **edit**: add `geometry_msgs`, `tf2_ros` exec deps |

Nothing else in the repo changes.

### Data flow (target architecture)

```
/person_detections ─┐
/camera/camera_info ─┤ (fx, cx, cy — cache first message)
MAVLink :15001 ──────┤ ATTITUDE (roll/pitch), GLOBAL_POSITION_INT (rel_alt)   [daemon thread]
TF map→base_link ────┤ x, y, yaw at detection stamp (fallback: latest)
                     ▼
          survivor_localizer_node
                     │ /localized_person_observation  (PoseStamped, frame=map, z=0,
                     │   stamp = detection stamp)
                     ▼
          survivor_evidence_node
                     ├── /survivor_grid          (PoseArray, debug, 1 Hz)
                     └── /confirmed_survivors     (PoseArray, 1 Hz)   ◀── ANSWER
```

---

## 4. File 1 — `survivor_localizer_node.py`

### 4.1 Hardcoded constants (top of file)

```python
# --- camera extrinsics (model.sdf) ---
CAM_T = (0.12, 0.0, -0.05)     # camera origin in base_link (x fwd, y left, z up)
CAM_PITCH = 0.7854             # rad, +45° down

# --- filtering ---
MIN_GROUND_DIST = 0.5          # m — rejects drone-on-ground / upward rays
MAX_GROUND_DIST = 20.0         # m — rejects near-horizon rays
MAVLINK_STALE_S = 2.0          # s — drop detections when MAVLink data too old
TF_FALLBACK_WARN_S = 0.5       # throttle for TF warnings

# --- fallback intrinsics (bag-verified; CameraInfo always preferred) ---
FALLBACK_FX, FALLBACK_CX, FALLBACK_CY = 381.36, 320.0, 240.0

# --- wiring ---
MAVLINK_URL = "udp:127.0.0.1:15001"   # NOT 15000 (lidar node owns 15000)
PERSON_TOPIC   = "/person_detections"
CAM_INFO_TOPIC = "/camera/camera_info"
OUT_TOPIC      = "/localized_person_observation"
MAP_FRAME, BASE_FRAME = "map", "base_link"
```

### 4.2 Node structure

- Subscribes: `PersonDetectionArray` on `PERSON_TOPIC` (default QoS — publisher is default
  reliable), `CameraInfo` on `CAM_INFO_TOPIC` (cache `k[0]=fx, k[4]=fy, k[2]=cx, k[5]=cy`
  on first message; before that, drop detections with a throttled log).
- TF: `tf2_ros.Buffer` + `TransformListener`.
- MAVLink: **separate daemon thread**, pattern copied from
  `src/lidar_processing/lidar_processing/lidar_tilt_stabilizer_node.py`:
  ```python
  conn = mavutil.mavlink_connection(MAVLINK_URL)   # binds local 15001
  while not conn.wait_heartbeat(timeout=15):       # retry loop — SITL may start later
      log("waiting for heartbeat..."); continue
  conn.mav.request_data_stream_send(conn.target_system, conn.target_component,
      mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)  # 10 Hz
  # then loop: recv_match(type=['ATTITUDE','GLOBAL_POSITION_INT'], blocking=True, timeout=1.0)
  #   ATTITUDE        -> roll = msg.roll, pitch = -msg.pitch        (FLU)
  #   GLOBAL_POSITION_INT -> rel_alt = msg.relative_alt / 1000.0    (mm -> m)
  # store under a threading.Lock together with time.monotonic() receipt time
  ```
  Never crash on missing SITL: retry heartbeat forever, warn throttled.
- On each detection: if MAVLink data older than `MAVLINK_STALE_S` (or never received) →
  drop, throttled warning. No CameraInfo yet → drop. Any TF exception → drop, throttled.

### 4.3 Geometry — exact per-detection pipeline

Detection bbox gives `xmin,xmax,ymin,ymax`. Use **bottom-center**:
`u = (xmin + xmax) / 2`, `v = ymax` (the bbox bottom edge ≈ the object's ground contact).

```python
# 1. pixel -> optical ray (image u right, v down, ray z forward; D=0 so no undistort)
ray = ((u - cx) / fx, (v - cy) / fy, 1.0)

# 2. optical -> camera link frame (x fwd, y left, z up):
#    x_l = z_o, y_l = -x_o, z_l = -y_o
r = (ray[2], -ray[0], -ray[1])

# 3. camera link -> base_link: mount pitch +45° down about +Y
#    Ry(a): x' = c*x + s*z,  y' = y,  z' = -s*x + c*z
c, s = cos(CAM_PITCH), sin(CAM_PITCH)
r = (c*r[0] + s*r[2], r[1], -s*r[0] + c*r[2])

# 4. level the ray: body attitude = Rz(yaw)·Ry(pitch)·Rx(roll); inverse applies
#    Ry(-pitch) FIRST, then Rx(-roll).  theta/phi are FLU angles from MAVLink.
theta, phi = -att_pitch, att_roll
c, s = cos(theta), sin(theta)
r = (c*r[0] + s*r[2], r[1], -s*r[0] + c*r[2])      # Ry(-pitch)
c, s = cos(phi), sin(phi)
r = (r[0], c*r[1] - s*r[2], s*r[1] + c*r[2])       # Rx(-roll)

# 5. camera height above ground: apply the SAME leveling to the mount translation
cam = same_Ry_Rx_as_step4(CAM_T)
cam_z = rel_alt + cam[2]        # rel_alt = EKF3 height of base_link above ground (flat arena)

# 6. intersect ground plane z = 0 in the gravity-aligned frame
if r[2] >= -1e-6:  drop                      # ray not pointing downward
t = -cam_z / r[2]
px, py = t * r[0], t * r[1]                  # point relative to drone, yaw not yet applied
if not (MIN_GROUND_DIST <= math.hypot(px, py) <= MAX_GROUND_DIST): drop
if not (isfinite(px) and isfinite(py)): drop

# 7. level frame -> map: yaw + translation from TF  map->base_link @ detection stamp
#    (try det stamp; on LookupException/ExtrapolationException retry with Time(0),
#     throttled warning — SLAM publishes ~3 Hz so stamp may be slightly ahead)
map_x = tx + cos(yaw) * px - sin(yaw) * py
map_y = ty + sin(yaw) * px + cos(yaw) * py

publish PoseStamped(frame_id="map", stamp=det_stamp, position=(map_x, map_y, 0.0))
```

**Mental-model sanity anchors** (used in verification):
- Level drone, image-center pixel → ray 45° below horizon → ground hit ≈ `cam_z` metres
  straight ahead.
- Bottom-of-frame pixel (`v=480`) → ≈77° below horizon → hit ≈ `0.23 * cam_z` ahead.

Dependencies: `rclpy`, `math`, `tf2_ros`, `nidar_msgs`, `geometry_msgs`, `sensor_msgs`,
`pymavlink`. **No numpy / cv2 / image_geometry** (vision_opencv is vendored but unbuilt).

---

## 5. File 2 — `survivor_evidence_node.py`

### Constants

```python
GRID_SIZE = 1.0          # m, cell = floor(coord / GRID_SIZE)
CONFIRM_N = 3            # votes to confirm
DEBOUNCE_S = 0.5         # min seconds between two counted votes in one cell
SUBSCRIBE   = "/localized_person_observation"
GRID_TOPIC  = "/survivor_grid"           # PoseArray debug
CONFIRM_TOPIC = "/confirmed_survivors"   # PoseArray final
PUBLISH_PERIOD = 1.0     # s
```

### State & algorithm

```
cells     : {(gx, gy): [vote_count, last_vote_wall_time]}
confirmed : set of (gx, gy)
suppressed: set of (gx, gy)          # neighbors absorbed into a confirmed cell

on observation (x, y):
    key = (floor(x / GRID_SIZE), floor(y / GRID_SIZE))
    if key in suppressed: return
    now = time.time()                # ARRIVAL wall time (immune to bag loops / sim-time jumps)
    if key in cells and now - cells[key].last < DEBOUNCE_S: return
    update cells[key]: count += 1, last = now
    if first vote in cell: log "SUSPECTED cell (gx,gy)"
    if cells[key].count >= CONFIRM_N and key not in confirmed:
        confirmed.add(key)
        log "CONFIRMED survivor (gx,gy) at (map_x, map_y)"
        for dx, dy in {-1,0,1}², excluding (0,0):
            nb = (gx+dx, gy+dy)
            cells.pop(nb, None)      # drop partial border-split votes
            suppressed.add(nb)       # future votes here can never confirm a duplicate

timer 1 Hz (node clock → sim time stamps):
    publish PoseArray(frame_id="map") of confirmed cell centers:
        center = ((gx + 0.5) * GRID_SIZE, (gy + 0.5) * GRID_SIZE, 0.0)
    publish PoseArray of all non-suppressed cells (debug; counts shown in logs)
```

**Why these two mechanisms are mandatory (do not "simplify" them away):**
- *Debounce*: without it, 30 fps detections confirm a cell in 0.1 s from a single noisy
  glance. With it, 3 votes require ≥ ~1 s of independent sightings.
- *Neighbor absorb/suppress*: a ground point straddling a cell border splits votes across
  two cells → one survivor would confirm two adjacent cells (violates "exactly 6").
  Absorbing ±1 collapses the split; suppressing stops the phantom from ever confirming.

---

## 6. File 3 — `src/perception/launch/nidar_pipeline.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),   # REQUIRED: image stamps are sim time
        Node(package="perception", executable="camera_yolo_node",
             name="camera_yolo_node", output="screen"),
        Node(package="perception", executable="survivor_localizer_node",
             name="survivor_localizer_node", output="screen"),
        Node(package="perception", executable="survivor_evidence_node",
             name="survivor_evidence_node", output="screen"),
    ])
```

No per-node `parameters=` — all tunables are in-file constants.

## 7. Files 4–5 — packaging edits

**`src/perception/setup.py`**
- Add to `entry_points["console_scripts"]`:
  ```
  survivor_localizer_node = perception.survivor_localizer_node:main
  survivor_evidence_node  = perception.survivor_evidence_node:main
  ```
- Re-add launch install to `data_files` (currently only `package.xml`):
  `('share/' + package_name + '/launch', ['launch/nidar_pipeline.launch.py']),`

**`src/perception/package.xml`** — add:
```xml
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
```
(pymavlink is used untracked by `lidar_processing`; matching that convention is fine.)

## 8. Build & run

```bash
cd /workspace                      # repo root — YOLO node loads "yolo11m.pt" relative to cwd
colcon build --packages-select perception
source install/setup.bash
# existing stack must already be up: gz sim, gz_ros_bridge, SLAM (see utils/Notes/)
# SITL command must keep BOTH --out lines (15000 for lidar node, 15001 for localizer)
ros2 launch perception nidar_pipeline.launch.py
```

If YOLO can't find the model, either run from the repo root or make the path absolute
(`/workspace/yolo11m.pt`) — one line, not a refactor.

---

## 9. Verification checklist (run in this order; each step isolates a failure layer)

1. **Boots cleanly.** Localizer log shows `CameraInfo cached: fx≈381.36 cx=320 cy=240` and
   `MAVLink heartbeat ... rel_alt=...`; YOLO log shows model loaded.
2. **Stamps sane.** `ros2 topic echo /localized_person_observation --once` → coordinates
   finite, stamp ≈ `/clock` (sim time).
3. **Static geometry.** GUIDED takeoff to ~5 m, hover level, dummy near bottom-center of
   image. Get drone pose (`ros2 run tf2_ros tf2_echo map base_link`). Published point must
   be ≈ drone position + `rel_alt` metres along the drone's yaw heading (bottom-center
   geometry ≈ 0.23·cam_z ahead; image center ≈ 1.0·cam_z ahead). Tolerance ~0.5 m.
4. **Tilt sign test.** Fly forward (vehicle pitches): points on stationary ground features
   must stay fixed in `map`. If they slide systematically → flip the pitch sign in step 4
   (`theta = att_pitch` instead of `-att_pitch`).
5. **Mirror test.** Dummy on the RIGHT half of the image must land on the RIGHT of the
   drone's forward axis. If left/right swaps → sign error in the optical→link step
   (`r = (ray[2], -ray[0], -ray[1])`).
6. **Full mission.** Fly the survey over all 6 dummies → exactly 6 `CONFIRMED` log lines,
   `ros2 topic echo /confirmed_survivors` → exactly 6 poses, no adjacent duplicates.
   Cross-check the **relative layout** of the 6 confirmed cells against dummy poses in
   `world_dummies.sdf` (relative pattern must match; absolute offset is expected because
   `map` origin ≠ Gazebo world origin).
7. **Revisit adds nothing.** Second pass over the field → count stays 6.

## 10. Gotchas (will bite if ignored)

- `use_sim_time` must be true on all three nodes (set once in the launch file); otherwise
  TF lookups at image stamps fail and the localizer silently publishes nothing.
- Localizer binds **15001**; 15000 belongs to `lidar_tilt_stabilizer_node`.
  Both `--out` lines must be present in the `sim_vehicle.py` command.
- `GLOBAL_POSITION_INT.relative_alt` is in **millimetres**.
- FRD→FLU: `pitch = -msg.pitch`, `roll = msg.roll`.
- Inverse leveling order is `Ry(-pitch)` **then** `Rx(-roll)` (inverse of `Rz·Ry·Rx`).
- `map→base_link` is yaw-only 2D (z≡0) — ground is always intersected at
  `z = -(rel_alt + cam_offset_z)` in the gravity frame, never at a fixed camera height.
- Drone must be airborne: on the ground `cam_z ≈ 0`, rays hit immediately, and the
  `MIN_GROUND_DIST` check rejects them (this is the desired behavior).
- Guard NaN/Inf before publishing.

Thought: 6.4s
Done. Summarize briefly.
Written to /home/jeswin/.opencode/plan/plan2.md.
This version is restructured as a self-contained implementation brief for another AI — it doesn't assume anything from our conversation:
Goal + output contract — exact topics/msg types, and the explicit "do not do" list (don't touch the YOLO node, no YAML, no new messages).
Existing architecture — what already runs (bridge, YOLO node, SLAM TF chain, SITL ports), with every verified fact (camera pose, intrinsics, zero distortion, relative_alt in mm, pymavlink bind behavior, sim-time stamps) stated as things it can rely on.
Things to build — a 5-file table, then a target data-flow diagram.
Per-file specs — constants blocks ready to paste, exact MAVLink thread pattern, the full 7-step geometry pipeline as literal code with the leveling order spelled out, the evidence algorithm as pseudocode with why each mechanism is mandatory (so it doesn't get "simplified" away), and the launch/setup/package.xml edits.
Build & run, then an ordered 7-step verification checklist where each step isolates one failure layer (stamps → static geometry → tilt sign → mirroring → full mission → revisit), plus a gotchas list.
Both plan.md (narrative version) and plan2.md (implementation brief) are in the plan directory. When you switch out of Plan mode, another agent can pick up plan2.md and build directly from it.