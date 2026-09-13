# ros2_ardupilot_bridge

Streams `nav_msgs/Odometry` (e.g. `/odom_rf2o` from `rf2o_laser_odometry`) into
ArduPilot as a MAVLink `ODOMETRY` message, for non-GPS ("ExternalNav") position
estimation in EKF3. `ODOMETRY` is ArduPilot's currently-preferred message for this
(over `VISION_POSITION_ESTIMATE`/`VISION_SPEED_ESTIMATE`) because it carries pose
and body-frame velocity in a single packet — a good match for rf2o's message shape.

## 1. Build

```bash
cd ~/ros2_ws/src
# copy/clone this package here
cd ~/ros2_ws
colcon build --packages-select ros2_ardupilot_bridge
source install/setup.bash
```

`pymavlink` must be installed in the environment ROS 2 uses:
```bash
pip install pymavlink --break-system-packages
```

## 2. Open a companion-computer link on SITL

SITL's MAVProxy already uses the usual ports (5760 internally, 14550 for a GCS).
Give this node its own UDP tap by adding an extra `--out` to `sim_vehicle.py`:

```bash
sim_vehicle.py -v ArduCopter --console --map \
  --out=udp:127.0.0.1:14551
```

MAVProxy will *send* packets to 127.0.0.1:14551. The node listens for that with
`udpin:127.0.0.1:14551` (the default `connection_string` parameter) and replies
on the same socket — this is the standard pattern used for companion-computer
links into SITL.

## 3. Set ArduPilot's EKF3 source parameters

In the MAVProxy console (or Mission Planner/QGroundControl connected on 14550):

```
param set AHRS_EKF_TYPE 3
param set EK3_ENABLE 1
param set VISO_TYPE 1          # 1 = MAVLink generic vision odometry source
param set EK3_SRC1_POSXY 6     # 6 = ExternalNav
param set EK3_SRC1_VELXY 6     # ExternalNav (or 0 = None if you don't trust rf2o's velocity)
param set EK3_SRC1_POSZ 1      # Baro — rf2o is 2D, it has no reliable Z
param set EK3_SRC1_VELZ 0
param set EK3_SRC1_YAW 6       # ExternalNav
```

For a first test, leave SITL's simulated GPS enabled (`GPS_TYPE` default) so the
EKF origin gets set automatically — you're only overriding *which* source EK3
listens to for XY position/velocity/yaw, not removing the origin. Once the
pipeline is verified you can disable GPS (`GPS_TYPE 0`) and set the EKF origin
manually (Mission Planner "Set Home Here → Set EKF Origin Here", or the
`ahrs-set-origin.lua` applet) for a true non-GPS test.

## 4. Run

```bash
ros2 launch ros2_ardupilot_bridge odom_to_mavlink.launch.py
```

Watch `EKF3` status and `ODOMETRY`/`LOCAL_POSITION_NED` in MAVProxy or Mission
Planner's MAVLink Inspector to confirm ArduPilot is receiving and fusing it, and
watch that `EKF3` doesn't report "IMU0 is using external nav" as failed/red.

## Notes / things to check for your setup

- **Rate**: rf2o's `freq` is 20 Hz in your launch file, comfortably above
  ArduPilot's documented ≥4 Hz minimum for ODOMETRY.
- **Yaw drift**: rf2o odometry (like all dead-reckoning) drifts in yaw over
  time with no absolute reference. If you have a compass, keeping
  `EK3_SRC1_YAW` on `ExternalNav` will let bad yaw drift affect the EKF; you
  may prefer `1 (Compass)` and only use ExternalNav for XY position/velocity.
- **Position covariance**: the node currently sends `NaN` for `pos_covariance`,
  which tells ArduPilot to fall back on `VISO_POS_M_NSE`/`VISO_YAW_M_NSE`
  defaults. If rf2o's confidence varies a lot, populate the real covariance
  from `msg.pose.covariance` instead for better EKF weighting.
- **`reset_counter`**: currently always `0`. If you ever reset rf2o's internal
  odometry (e.g. on relocalization), increment this so ArduPilot knows not to
  treat the jump as real motion.
