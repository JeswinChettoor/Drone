import sys
import subprocess
import select
import termios
import tty
import math

HELP_MSG = """
=============================================
Gazebo Direct Kinematic Pose Teleop
Model: iris | World: arena_world
=============================================
Movement (Step: 0.25 m):
        W (+X)
   A (+Y)    D (-Y)
        S (-X)

Altitude:
   R : Up (+Z)
   F : Down (-Z)

Yaw:
   Q : Rotate Left (+15 deg)
   E : Rotate Right (-15 deg)

Reset:
   Space : Reset to start (6.0, -6.5, 1.0)
   Ctrl+C: Quit
=============================================
"""

WORLD_NAME = "arena_world"
MODEL_NAME = "iris"
STEP_DIST = 0.05
STEP_YAW = math.radians(15.0)

class DirectPoseController:
    def __init__(self):
        # Starting coordinates from your SDF file
        self.x = 6.0
        self.y = -6.5
        self.z = 0.2
        self.yaw = 1.5708

    def update_pose(self, dx=0.0, dy=0.0, dz=0.0, dyaw=0.0):
        # Move relative to the current drone heading (body frame)
        if dx != 0 or dy != 0:
            self.x += dx * math.cos(self.yaw) - dy * math.sin(self.yaw)
            self.y += dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
        self.z += dz
        self.yaw = (self.yaw + dyaw) % (2 * math.pi)

        # Quaternion from yaw (roll=0, pitch=0)
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # Dispatch set_pose service call directly to Gazebo Sim
        cmd = [
            "gz", "service", "-s", f"/world/{WORLD_NAME}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "300",
            "--req",
            f'name: "{MODEL_NAME}", '
            f'position: {{x: {self.x:.3f}, y: {self.y:.3f}, z: {self.z:.3f}}}, '
            f'orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}'
        ]
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"\rPose -> X: {self.x:6.2f} | Y: {self.y:6.2f} | Z: {self.z:5.2f} | Yaw: {math.degrees(self.yaw):5.1f}°  ", end="", flush=True)

    def reset(self):
        self.x = 6.0
        self.y = -6.5
        self.z = 0.1
        self.yaw = 1.5708
        self.update_pose()

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key.lower()

def main():
    settings = termios.tcgetattr(sys.stdin)
    controller = DirectPoseController()
    print(HELP_MSG)
    controller.update_pose()

    try:
        while True:
            k = get_key(settings)
            if k == 'w':
                controller.update_pose(dx=STEP_DIST)
            elif k == 's':
                controller.update_pose(dx=-STEP_DIST)
            elif k == 'a':
                controller.update_pose(dy=STEP_DIST)
            elif k == 'd':
                controller.update_pose(dy=-STEP_DIST)
            elif k == 'r':
                controller.update_pose(dz=STEP_DIST)
            elif k == 'f':
                controller.update_pose(dz=-STEP_DIST)
            elif k == 'q':
                controller.update_pose(dyaw=STEP_YAW)
            elif k == 'e':
                controller.update_pose(dyaw=-STEP_YAW)
            elif k == ' ':
                controller.reset()
            elif k == '\x03':  # Ctrl+C
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nExited.")

if __name__ == '__main__':
    main()
