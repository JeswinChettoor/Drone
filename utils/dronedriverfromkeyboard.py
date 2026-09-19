#!/usr/bin/env python3

from pymavlink import mavutil
import sys
import termios
import tty
import time


# --------------------------------------------------
# MAVLink connection
# --------------------------------------------------

master = mavutil.mavlink_connection(
    "udp:127.0.0.1:14551"
)

print("Waiting for heartbeat...")
master.wait_heartbeat()

print("Connected!")
print(f"System: {master.target_system}")
print(f"Component: {master.target_component}")


# --------------------------------------------------
# Put vehicle into GUIDED mode
# --------------------------------------------------

master.set_mode_apm("GUIDED")

time.sleep(1)

print("GUIDED mode requested")


# --------------------------------------------------
# Velocity settings
# --------------------------------------------------

SPEED = 0.5       # m/s
VERTICAL_SPEED = 0.3


# Current commanded velocity
vx = 0.0   # North
vy = 0.0   # East
vz = 0.0   # Down


# --------------------------------------------------
# Send LOCAL_NED velocity command
# --------------------------------------------------

def send_velocity(vx, vy, vz):

    # Type mask:
    #
    # Ignore position
    # Ignore acceleration
    # Ignore yaw
    # Ignore yaw rate
    #
    # Only use vx, vy, vz

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,

        master.target_system,
        master.target_component,

        mavutil.mavlink.MAV_FRAME_LOCAL_NED,

        type_mask,

        0, 0, 0,       # position ignored

        vx, vy, vz,    # velocity

        0, 0, 0,       # acceleration ignored

        0,             # yaw ignored
        0              # yaw rate ignored
    )


# --------------------------------------------------
# Keyboard
# --------------------------------------------------

def get_key():

    fd = sys.stdin.fileno()

    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )

    return key


print("""
=====================================
       LOCAL NED KEYBOARD CONTROL
=====================================

W : North
S : South

A : West
D : East

R : Up
F : Down

SPACE : Stop horizontal movement
X     : Stop all movement

ESC   : Exit

Speed = 0.5 m/s
=====================================
""")


# --------------------------------------------------
# Main loop
# --------------------------------------------------

try:

    while True:

        key = get_key().lower()

        # Reset velocities
        vx = 0.0
        vy = 0.0
        vz = 0.0

        # -----------------------------
        # Horizontal movement
        # -----------------------------

        if key == "w":
            vx = SPEED

        elif key == "s":
            vx = -SPEED

        elif key == "a":
            vy = -SPEED

        elif key == "d":
            vy = SPEED

        # -----------------------------
        # Vertical movement
        # -----------------------------

        elif key == "r":
            vz = -VERTICAL_SPEED

        elif key == "f":
            vz = VERTICAL_SPEED

        # -----------------------------
        # Stop
        # -----------------------------

        elif key == " ":
            vx = 0
            vy = 0
            vz = 0

        elif key == "x":
            vx = 0
            vy = 0
            vz = 0

        # -----------------------------
        # Exit
        # -----------------------------

        elif ord(key) == 27:
            break

        # -----------------------------
        # Send command
        # -----------------------------

        send_velocity(vx, vy, vz)

        print(
            f"\rN={vx:+.2f} "
            f"E={vy:+.2f} "
            f"D={vz:+.2f}     ",
            end="",
            flush=True
        )


finally:

    # Stop vehicle movement
    for _ in range(10):
        send_velocity(0, 0, 0)
        time.sleep(0.05)

    print("\nStopped.")