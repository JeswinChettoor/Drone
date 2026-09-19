#!/usr/bin/env python3

import time
from pymavlink import mavutil


# ------------------------------------------------------------
# Connect to ArduPilot
# ------------------------------------------------------------

CONNECTION = "udp:127.0.0.1:15001"

print(f"Connecting to {CONNECTION}...")

mav = mavutil.mavlink_connection(CONNECTION)

print("Waiting for heartbeat...")
mav.wait_heartbeat()

print(
    f"Heartbeat received: "
    f"system={mav.target_system}, "
    f"component={mav.target_component}"
)


counter = 0

while True:

    # Position: fixed at origin
    x = 0.0
    y = 0.0
    z = 0.0

    # Identity quaternion
    #
    # MAVLink quaternion order:
    # [w, x, y, z]
    #
    # This means no rotation.
    q = [1.0, 0.0, 0.0, 0.0]

    # Fixed zero velocity
    vx = 0.0
    vy = 0.0
    vz = 0.0

    # Fixed zero angular velocity
    rollspeed = 0.0
    pitchspeed = 0.0
    yawspeed = 0.0

    # No covariance information
    #
    # For this first test, use zeros rather than NaN.
    pose_covariance = [0.0] * 21
    velocity_covariance = [0.0] * 21

    # Send ODOMETRY
    mav.mav.odometry_send(

        # time_usec
        int(time.time() * 1_000_000),

        # frame_id
        mavutil.mavlink.MAV_FRAME_LOCAL_FRD,

        # child_frame_id
        mavutil.mavlink.MAV_FRAME_BODY_FRD,

        # position
        x,
        y,
        z,

        # quaternion
        q,

        # velocity
        vx,
        vy,
        vz,

        # angular velocity
        rollspeed,
        pitchspeed,
        yawspeed,

        # pose covariance
        pose_covariance,

        # velocity covariance
        velocity_covariance,

        # reset_counter
        0,

        # estimator_type
        mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,

        # quality
        0
    )

    counter += 1

    print(f"Sent ODOMETRY #{counter}")

    time.sleep(0.1)   # 10 Hz
