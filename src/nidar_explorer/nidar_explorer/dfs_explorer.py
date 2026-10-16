#!/usr/bin/env python3
"""
Minimal grid-DFS maze explorer: ArduPilot Copter + SLAM (/map + TF map->base_link) + fixed front camera.
Assumes the drone is ALREADY IN THE AIR hovering at the centre of the first room.
Flip to GUIDED mode to start exploration.
"""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

CONN = "udp:127.0.0.1:14550"     # or "/dev/ttyAMA0"
MAP_FRAME, BASE_FRAME = "map", "base_link"
PITCH = 3.0                      # room centre to room centre (2 m room + 1 m corridor)
N = 5                            # max grid span
OPEN = 1.6                       # free distance above this = opening (wall is ~1 m away)
STOP = 0.45                      # real occupied wall closer than this ahead = stop
TOL = 0.25                       # arrival tolerance (m)
SPIN_STEPS, DWELL = 6, 0.3       # 6 x 60 deg, 0.3 s pause each
VMAX, KP = 0.35, 0.8             # max speed (m/s), position gain (1/s)
WMAX, KW = 1.2, 1.8              # faster yaw rate (~69 deg/s), snappier heading gain
VZMAX, KZ = 0.3, 1.0             # altitude hold
GUIDED, LAND = 4, 9              # ArduCopter mode numbers
DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]    # 0 forward, 1 right, 2 back, 3 left
MASK_VEL_YAWRATE = 0b0000010111000111        # use vx,vy,vz + yaw_rate (= 1479)
DT = 0.05                        # 20 Hz control loop period
TF_TIMEOUT = 2.0                 # max seconds without a new TF stamp before failsafe


class Stop(Exception):
    """Pilot left GUIDED: stop commanding."""


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Maze(Node):
    def __init__(self):
        super().__init__("maze_dfs")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.mav = mavutil.mavlink_connection(CONN)
        self.mav.wait_heartbeat()
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1,
        )
        self.map = None
        self.x = self.y = self.yaw = self.z = self.z0 = self.mode = None
        self.last_tf_stamp = None
        self.last_tf_wall = 0.0
        self.armed = self.flying = False

    # ---------------------------------------------------------------- I/O
    def now(self):
        return time.monotonic()

    def on_map(self, m):
        i = m.info
        self.map = (
            np.array(m.data, np.int8).reshape(i.height, i.width),
            i.resolution,
            i.origin.position.x,
            i.origin.position.y,
        )

    def tick(self):
        t_start = self.now()
        while rclpy.ok() and (self.now() - t_start) < DT:
            rclpy.spin_once(self, timeout_sec=0.005)

        try:
            t = self.tf.lookup_transform(MAP_FRAME, BASE_FRAME, Time())
            q = t.transform.rotation
            self.x, self.y = t.transform.translation.x, t.transform.translation.y
            self.yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            stamp = (t.header.stamp.sec, t.header.stamp.nanosec)
            if stamp != self.last_tf_stamp:
                self.last_tf_stamp = stamp
                self.last_tf_wall = self.now()
        except TransformException:
            pass

        ok = (self.now() - self.last_tf_wall) < TF_TIMEOUT

        while (m := self.mav.recv_match(blocking=False)) is not None:
            k = m.get_type()
            if k == "LOCAL_POSITION_NED":
                self.z = m.z
            elif k == "HEARTBEAT" and m.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                self.mode = m.custom_mode
                self.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        if self.flying:
            if self.mode != GUIDED:
                raise Stop()
            if not ok:
                raise RuntimeError("SLAM pose lost (no new TF for > 2.0s)")

    def cmd(self, fwd, right, down, yaw_rate_cw):
        """Body-frame velocity (m/s: forward, right, down) + yaw rate (rad/s, clockwise +)."""
        self.mav.mav.set_position_target_local_ned_send(
            0,
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            MASK_VEL_YAWRATE,
            0, 0, 0,
            float(fwd), float(right), float(down),
            0, 0, 0,
            0, float(yaw_rate_cw),
        )

    # ---------------------------------------------------------------- map
    def look(self, h, half=6.0, rmin=0.35, rmax=5.0, unknown_is_wall=True):
        """
        Free distance from the drone along map-frame heading h (rad, CCW).
        - rmin=0.35 skips the drone's own footprint/sensor blindspot.
        - unknown_is_wall=True for door detection; False for emergency collision checks.
        """
        g, res, ox, oy = self.map
        d = np.arange(rmin, rmax, res / 2)
        best = rmax
        for a in h + np.radians(np.linspace(-half, half, 5)):
            ix = np.floor((self.x + d * math.cos(a) - ox) / res).astype(int)
            iy = np.floor((self.y + d * math.sin(a) - oy) / res).astype(int)
            inside = (ix >= 0) & (ix < g.shape[1]) & (iy >= 0) & (iy < g.shape[0])
            v = np.full(d.shape, -1, np.int8)
            v[inside] = g[iy[inside], ix[inside]]
            blocked = (v < 0) | (v >= 65) if unknown_is_wall else (v >= 65)
            hit = np.nonzero(blocked)[0]
            if hit.size:
                best = min(best, float(d[hit[0]]))
        return best

    def hd(self, k):
        return self.yaw0 - k * math.pi / 2           # k=1 is a RIGHT turn = clockwise

    def openings(self):
        dists = [self.look(self.hd(k), half=6.0, unknown_is_wall=True) for k in range(4)]
        o = [d > OPEN for d in dists]
        print(
            f"open (fwd,right,back,left): {[int(x) for x in o]} "
            f"dists: {[round(d, 2) for d in dists]}"
        )
        return o

    # ---------------------------------------------------------------- motion
    def track(self, tx, ty, h):
        """One control step: head for map point (tx, ty), hold map heading h and initial hover altitude z0."""
        ex, ey = tx - self.x, ty - self.y
        d = math.hypot(ex, ey)
        v = min(VMAX, KP * d)
        vx, vy = (v * ex / d, v * ey / d) if d > 1e-3 else (0.0, 0.0)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        fwd, right = vx * c + vy * s, vx * s - vy * c
        down = float(np.clip(KZ * (self.z0 - self.z), -VZMAX, VZMAX))
        w = float(np.clip(-KW * wrap(h - self.yaw), -WMAX, WMAX))
        self.cmd(fwd, right, down, w)

    def hold(self, s=0.5):
        p, y, t = (self.x, self.y), self.yaw, self.now()
        while self.now() - t < s:
            self.track(p[0], p[1], y)
            self.tick()

    def face(self, h, p):
        t0 = self.now()
        while abs(wrap(self.yaw - h)) > 0.15:
            self.track(p[0], p[1], h)
            self.tick()
            if self.now() - t0 > 12:
                raise RuntimeError("cannot turn to the commanded heading (yaw sign / SLAM yaw wrong?)")

    def spin(self):
        p, y0 = (self.x, self.y), self.yaw
        for i in range(1, SPIN_STEPS + 1):           # start turning immediately to +60 deg
            h = wrap(y0 + i * 2 * math.pi / SPIN_STEPS)
            self.face(h, p)
            t = self.now()
            while self.now() - t < DWELL:
                self.track(p[0], p[1], h)
                self.tick()

    def go(self, tgt):
        """Fly to tgt (map x, y), camera facing the way we go. False = stopped by an obstacle."""
        d = tgt - np.array([self.x, self.y])
        if np.linalg.norm(d) < TOL:
            return True
        h = math.atan2(d[1], d[0])
        self.face(h, (self.x, self.y))
        t0 = self.now()
        while self.now() - t0 < 60:
            rem = tgt - np.array([self.x, self.y])
            if np.linalg.norm(rem) < TOL:
                return True
            # Check narrow cone (5 deg) for REAL occupied walls (ignore unknown cells in sensor blindspot)
            obs_dist = self.look(h, half=5.0, rmin=0.25, unknown_is_wall=False)
            if obs_dist < STOP:
                print(f"obstacle ahead at {obs_dist:.2f}m -> stop")
                self.hold()
                return False
            self.track(tgt[0], tgt[1], h)
            self.tick()
        self.hold()
        return False

    def back(self, tgt):
        """Return to a previously visited room without false doorway stops."""
        d = tgt - np.array([self.x, self.y])
        if np.linalg.norm(d) < TOL:
            return
        h = math.atan2(d[1], d[0])
        self.face(h, (self.x, self.y))
        t0 = self.now()
        while self.now() - t0 < 60:
            if np.linalg.norm(tgt - np.array([self.x, self.y])) < TOL:
                self.hold(0.3)
                return
            self.track(tgt[0], tgt[1], h)
            self.tick()
        raise RuntimeError("cannot get back to a known room")

    # ---------------------------------------------------------------- mission
    def at(self, node):
        """Map position of a room centre."""
        u = lambda k: np.array([math.cos(self.hd(k)), math.sin(self.hd(k))])
        return self.home + PITCH * (node[0] * u(0) + node[1] * u(1))

    def explore(self):
        cur, k = (0, 0), 0
        visited, blocked, stack = {cur}, set(), []
        self.spin()                                  # spin first so SLAM map updates around room
        opens = {cur: self.openings()}
        opens[cur][2] = False                        # never leave through the entrance
        while True:
            nxt = None
            for d in (k, (k - 1) % 4, (k + 1) % 4, (k + 2) % 4):  # straight, left, right, back
                n = (cur[0] + DIRS[d][0], cur[1] + DIRS[d][1])
                if (
                    opens[cur][d]
                    and n not in visited
                    and (cur, d) not in blocked
                    and 0 <= n[0] < N
                    and abs(n[1]) < N
                ):
                    nxt = (n, d)
                    break
            if nxt:
                n, d = nxt
                print(f"Moving from {cur} -> {n} (dir={d})")
                if self.go(self.at(n)):
                    stack.append((cur, d))
                    cur, k = n, d
                    visited.add(cur)
                    self.spin()                      # scan room and refresh /map before reading doors
                    opens[cur] = self.openings()
                else:                                # obstacle: block this edge, go back
                    blocked |= {(cur, d), (n, (d + 2) % 4)}
                    self.back(self.at(cur))
            elif stack:
                print(f"dead end / done at {cur}; backtracking ({len(visited)} rooms seen)")
                cur, d = stack.pop()
                self.back(self.at(cur))
                k = (d + 2) % 4
            else:
                break
        self.face(self.yaw0, (self.x, self.y))

    def run(self):
        print("Waiting for fresh SLAM TF, /map, airborne altitude (z < -0.3m), armed, and GUIDED...")
        while (
            None in (self.x, self.z)
            or self.map is None
            or not self.armed
            or self.mode != GUIDED
            or self.z > -0.3
            or (self.now() - self.last_tf_wall) >= TF_TIMEOUT
        ):
            self.tick()

        # Lock current mid-air state as the home reference and altitude target
        self.z0 = self.z
        self.yaw0 = self.yaw
        self.home = np.array([self.x, self.y])
        self.flying = True
        print(
            f"Mid-air handover locked at ({self.x:.2f}, {self.y:.2f}), "
            f"alt={-self.z0:.2f}m, yaw0={math.degrees(self.yaw0):.1f} deg"
        )

        self.hold(0.5)
        self.explore()
        self.flying = False
        self.mav.set_mode(LAND)
        print("home reached, landing")


def main():
    rclpy.init()
    node = Maze()
    try:
        node.run()
    except Stop:
        print("pilot took over: no more commands")
    except (Exception, KeyboardInterrupt) as e:
        print("FAILSAFE:", repr(e), "-> LAND")
        node.mav.set_mode(LAND)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()