#!/usr/bin/env python3
"""
Ariadne's Spine - compact location-node version (ROS 2 Humble)

* Nodes are (x, y) points in the `map` frame. Node 0 is the takeoff point (home).
* Graph = every leg flown + every loop link. Dijkstra over it gives all routing:
    - next target = nearest node (by path length) that still has an untried exit
    - when nothing is left, fly the shortest path back to node 0
* A known node only counts as "near" (merge) if the straight line to it is clear on the map
  (seg_free). Routes follow the path node by node (the nodes we flew through), no skipping ahead.
* Wall facing: cardinal frame is trimmed to the lidar wall directions. Coverage: per-node bearing
  bins; rooms rotate only until every bin was seen, every leg first checks the sector ahead.
* Debug: MarkerArray /ariadne/debug (frame `map`).
Output: /cmd_vel (ROS FLU). Convert to MAVLink FRD after this node.
"""
import heapq
import math
from enum import Enum
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

TAU = 2.0 * math.pi
HALF_PI = math.pi / 2.0


def wrap_angle(a):
    return (a + math.pi) % TAU - math.pi


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class State(Enum):
    INIT = 0
    CENTER = 1
    SWEEP = 2
    WAIT_MAP = 3
    DECIDE = 4
    ALIGN = 5
    GOTO = 6
    COMPLETE = 7


class TopoNode:
    def __init__(self, nid, x, y, room, nb):
        self.id, self.x, self.y, self.is_room = nid, x, y, room
        self.exits, self.links, self.entry_card = [], set(), None
        self.cover = np.zeros(nb, np.int32)          # lidar hits per bearing bin (map frame)
        self.box, self.viewed = None, False          # room footprint (cx, cy, yaw, half_len, half_wid)

    def contains(self, x, y, m=0.15):
        if self.box is None:
            return False
        cx, cy, a, hl, hw = self.box
        dx, dy = x - cx, y - cy
        return abs(math.cos(a) * dx + math.sin(a) * dy) < hl - m and abs(-math.sin(a) * dx + math.cos(a) * dy) < hw - m

    def open_exits(self):
        return [e for e in self.exits if not e['tried']]


class NodeRegistry:
    """Location-based node store + graph routing. Identity = distance in the map frame."""

    def __init__(self, merge, nb):
        self.merge, self.nb, self.nodes = merge, nb, {}

    def create(self, x, y, room):
        n = TopoNode(len(self.nodes), x, y, room, self.nb)
        self.nodes[n.id] = n
        return n

    def link(self, a, b):
        a.links.add(b.id)
        b.links.add(a.id)

    def find(self, x, y, visible):
        """Known node (not home) if the point is inside its room footprint, or within merge
        distance with a clear line to it."""
        for d, i in sorted((math.hypot(n.x - x, n.y - y), n.id) for n in self.nodes.values() if n.id):
            n = self.nodes[i]
            if n.contains(x, y) or (d < self.merge and visible(n)):
                return n, d
        return None, 0.0

    def path(self, a, b):
        """Dijkstra over links, edge cost = distance. Returns (ids a..b, cost) or (None, inf)."""
        pq, seen = [(0.0, a, [a])], set()
        while pq:
            c, i, p = heapq.heappop(pq)
            if i == b:
                return p, c
            if i in seen:
                continue
            seen.add(i)
            n = self.nodes[i]
            for j in n.links - seen:
                m = self.nodes[j]
                heapq.heappush(pq, (c + math.hypot(n.x - m.x, n.y - m.y), j, p + [j]))
        return None, float('inf')


class AriadneMapNav(Node):
    def __init__(self):
        super().__init__('ariadne_map_nav')
        self.declare_parameter('node_merge_dist', 1.2)     # m, must stay < LEG_MIN_NODE
        self.declare_parameter('node_max_spacing', 4.0)    # m, longest leg before a waypoint node
        self.MAX_SPACING = float(self.get_parameter('node_max_spacing').value)
        self.BASE, self.MAP = 'base_link', 'map'

                # flight & safety
        self.CORRIDOR_SPEED, self.DOORWAY_SPEED = 0.40, 0.22
        self.HARD_STOP, self.REPULSE, self.ROBOT_RADIUS = 0.45, 0.30, 0.30   # REPULSE 0.36 -> 0.30 so a 0.7 m door isn't "repulsed"
        self.DECEL, self.LATENCY, self.SERVO_MAX = 0.5, 0.3, 1.4
        # map topology
        self.ROOM_MIN_SPAN, self.LAT_WIN, self.GAP_DEPTH = 3.0, 2.5, 0.25    # fewer false "rooms"; look further along walls; shallower openings count
        self.MIN_GAP = 2.0 * self.ROBOT_RADIUS + 0.1                         # 0.7 m doors (was 0.8)
        self.MAX_GAP, self.CORRIDOR_WINDOW = 6.0, 3.0                        # wide openings; corridors up to 3 m wide
        self.EXIT_MIN_DEPTH, self.PROBE_AHEAD, self.LEG_MAX, self.LEG_MIN_NODE = 0.8, 1.0, 12.0, 1.2
        self.CENTER_TOL, self.ARRIVE_TOL, self.PASS_TOL = 0.10, 0.15, 0.35
        # wall facing
        self.WALL_CONF, self.WALL_MIN_LEN, self.WALL_MAX_DELTA = 0.55, 1.0, 0.35
        # coverage
        self.NB = 36
        self.bin_c = -math.pi + (np.arange(self.NB) + 0.5) * TAU / self.NB
        self.MIN_HITS, self.FEED_R, self.SETTLE, self.UNSEEN_FRAC = 2, 0.6, 0.5, 0.12
        self.FRONT_HALF, self.SWEEP_RATE = math.radians(40.0), 0.55
        # timeouts
        self.SCAN_TIMEOUT, self.TF_TIMEOUT, self.CENTER_TIMEOUT = 0.5, 1.0, 10.0
        self.ALIGN_TIMEOUT, self.SPIN_TIMEOUT, self.MAP_WAIT = 15.0, 40.0, 7.0
        self.STUCK_TIMEOUT, self.OBSTRUCT_TIMEOUT = 8.0, 2.5                 # don't give up on an exit as quickly

        self.registry = NodeRegistry(float(self.get_parameter('node_merge_dist').value), self.NB)
        self.nodes = self.registry.nodes

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg_pub = self.create_publisher(MarkerArray, '/ariadne/debug', 1)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state, self.state_t0 = State.INIT, self.now_s()
        self.scan, self.scan_t, self._arr, self._arr_msg, self._R, self._fed = None, 0.0, None, None, {}, None
        self.blocked, self.unk, self.m_res, self.m_ox, self.m_oy, self.m_w, self.m_h, self.map_t = None, None, 0.05, 0.0, 0.0, 0, 0, 0.0
        self.frame_offset, self.target_odom_yaw, self._map_yaw, self._odom_yaw = 0.0, 0.0, 0.0, 0.0
        self._init = [0.0, 0.0, 0]
        self.cur, self.active_cardinal, self.active_exit = None, 0, None
        self.mode = 'explore'            # explore | route | home
        self.wps, self.route, self.route_goal = [], [], 0
        self.fails, self.loops = 0, []
        self.refresh, self.front_checked, self.after_view = False, False, False
        self.sweep_region, self.sweep_dir, self.last_yaw, self.spin = 'all', 0.0, None, 0.0
        self.hold_xy, self.prog, self.prog_t, self.block_t = (0.0, 0.0), (0.0, 0.0), 0.0, None
        self._pose, self._odom_xy, self.trail = None, (0.0, 0.0), []
        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.25, self.debug_cb)

    # ------------------------------------------------------------------ helpers
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s):
        self.state, self.state_t0 = s, self.now_s()

    def card_yaw(self, c):
        return wrap_angle(self.frame_offset + c * HALF_PI)

    def snap_card(self, yaw):
        return int(round(wrap_angle(yaw - self.frame_offset) / HALF_PI)) % 4

    def face(self, yaw_map):
        """Latch a map-frame heading in the continuous odom frame."""
        self.target_odom_yaw = wrap_angle(self._odom_yaw + wrap_angle(yaw_map - self._map_yaw))

    def set_cardinal(self, c):
        self.active_cardinal = c
        self.face(self.card_yaw(c))

    @staticmethod
    def to_base(dx, dy, yaw):
        return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy

    def stop(self):
        self.publish_safe_cmd(0.0, 0.0, 0.0)

    def reset_progress(self):
        self.prog, self.prog_t, self.block_t = self._odom_xy, self.now_s(), None

    def scan_cb(self, msg):
        self.scan, self.scan_t = msg, self.now_s()

    def map_cb(self, msg):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        d = np.asarray(msg.data, np.int16).reshape(h, w)
        occ, dil = d >= 50, d >= 50
        for dy in (-1, 0, 1):                                   # 3x3 dilation (edges wrap; edges are unknown anyway)
            for dx in (-1, 0, 1):
                dil = dil | np.roll(np.roll(occ, dy, 0), dx, 1)
        self.unk = d < 0
        self.blocked = dil | self.unk                           # unknown blocks
        self.m_res, self.m_w, self.m_h = msg.info.resolution, w, h
        self.m_ox, self.m_oy = msg.info.origin.position.x, msg.info.origin.position.y
        self.map_t = self.now_s()

    def get_poses(self):
        out = []
        for parent in (self.MAP, 'odom'):
            try:
                t = self.tf_buffer.lookup_transform(parent, self.BASE, Time())
            except Exception as e:
                self.get_logger().warn(f"TF {parent}->{self.BASE}: {e}", throttle_duration_sec=2.0)
                return None
            s = t.header.stamp
            if (s.sec or s.nanosec) and abs(self.now_s() - (s.sec + s.nanosec * 1e-9)) > self.TF_TIMEOUT:
                self.get_logger().warn(f"TF {parent} stale; use_sim_time?", throttle_duration_sec=2.0)
                return None
            out.append((t.transform.translation.x, t.transform.translation.y, quat_yaw(t.transform.rotation)))
        return out

    # ------------------------------------------------------------------ map ray casting
    def ray_dist(self, px, py, dx, dy, maxd, step=None):
        """Distance from each start point along (dx, dy) to the first blocked cell (obstacle, unknown,
        off-map). Start inside a blocked cell -> 0. Returns maxd if nothing is hit."""
        px, py = np.asarray(px, np.float64), np.asarray(py, np.float64)
        if self.blocked is None:
            return np.zeros(px.shape)
        t = np.arange(0.0, maxd, step or self.m_res)
        ix = np.floor((px[:, None] + dx * t - self.m_ox) / self.m_res).astype(np.int64)
        iy = np.floor((py[:, None] + dy * t - self.m_oy) / self.m_res).astype(np.int64)
        inb = (ix >= 0) & (ix < self.m_w) & (iy >= 0) & (iy < self.m_h)
        blk = np.ones(ix.shape, bool)
        blk[inb] = self.blocked[iy[inb], ix[inb]]
        return np.where(blk.any(1), t[blk.argmax(1)], maxd)

    def rd(self, x, y, ang, maxd, step=None):
        return float(self.ray_dist([x], [y], math.cos(ang), math.sin(ang), maxd, step)[0])

    def seg_free(self, ax, ay, bx, by):
        """Is the straight segment a->b clear on the map (centre line plus +-0.2 m)?"""
        L = math.hypot(bx - ax, by - ay)
        if L < 0.05:
            return True
        dx, dy = (bx - ax) / L, (by - ay) / L
        off = np.array([-0.2, 0.0, 0.2])
        return bool(np.all(self.ray_dist(ax - dy * off, ay + dx * off, dx, dy, L) >= L))

    def card_dists(self, x, y):
        return {c: self.rd(x, y, self.card_yaw(c), 6.0) for c in range(4)}

    def is_room(self, cd):
        return cd[0] + cd[2] > self.ROOM_MIN_SPAN and cd[1] + cd[3] > self.ROOM_MIN_SPAN and min(cd.values()) > 0.75

    def wall_mode(self, D):
        """Nearest dense cluster of distances = the wall line."""
        edges = np.arange(0.0, self.LEG_MAX + 0.2, 0.1)
        sm = np.convolve(np.histogram(np.clip(D, 0, self.LEG_MAX), bins=edges)[0], np.ones(3), 'same')
        if sm.max() <= 0:
            return float(np.median(D))
        k = int(np.argmax(sm >= 0.6 * sm.max()))
        sel = (D >= edges[k] - 0.1) & (D < edges[min(k + 1, len(edges) - 1)] + 0.1)
        return float(np.median(D[sel] if sel.any() else D))

    # ------------------------------------------------------------------ exits & legs
    def compute_exits(self, x, y):
        """Wall openings per cardinal side (parallel ray fan) + straight-on corridors."""
        exits = []
        for c in range(4):
            a = self.card_yaw(c)
            dx, dy = math.cos(a), math.sin(a)
            px, py = -dy, dx
            Lp = self.rd(x, y, a + HALF_PI, self.LAT_WIN, self.m_res)
            Ln = self.rd(x, y, a - HALF_PI, self.LAT_WIN, self.m_res)
            v = np.arange(-Ln + 0.1, Lp - 0.1 + 1e-6, 0.1)
            if v.size >= 5:
                D = self.ray_dist(x + v * px, y + v * py, dx, dy, self.LEG_MAX)
                ok = D > 0.3
                if ok.sum() >= 5:
                    W = self.wall_mode(D[ok])
                    thr = ok & (D > W + self.GAP_DEPTH)
                    i = 0
                    while i < len(v):
                        if not thr[i]:
                            i += 1
                            continue
                        j = i
                        while j + 1 < len(v) and thr[j + 1]:
                            j += 1
                        width = (j - i + 1) * 0.1 + 0.1
                        if self.MIN_GAP <= width <= self.MAX_GAP:
                            vc = 0.5 * (v[i] + v[j])
                            exits.append({'card': c, 'goal': (x + W * dx + vc * px, y + W * dy + vc * py),
                                          'lat': float(vc), 'tried': False, 'blocked': False})
                        i = j + 1
            # straight-on corridor; side clearance also probed PROBE_AHEAD down the line so a
            # T-junction centre still sees the corridor that continues past the branch
            D0, side = self.rd(x, y, a, self.LEG_MAX), Lp + Ln
            if side > self.CORRIDOR_WINDOW and D0 >= self.PROBE_AHEAD + self.EXIT_MIN_DEPTH:
                qx, qy = x + self.PROBE_AHEAD * dx, y + self.PROBE_AHEAD * dy
                side = self.rd(qx, qy, a + HALF_PI, self.LAT_WIN, self.m_res) + \
                    self.rd(qx, qy, a - HALF_PI, self.LAT_WIN, self.m_res)
            if side <= self.CORRIDOR_WINDOW and D0 >= self.EXIT_MIN_DEPTH:
                exits.append({'card': c, 'goal': None, 'lat': 0.0, 'tried': False, 'blocked': False})
        return exits

    def add_exits(self, node, new):
        for e in new:
            dup = next((o for o in node.exits if o['card'] == e['card'] and abs(o['lat'] - e['lat']) < 0.6), None)
            if dup:
                if dup['goal'] is None and e['goal'] is not None:
                    dup.update(goal=e['goal'], lat=e['lat'])
                continue
            if node.entry_card is not None and e['card'] == (node.entry_card + 2) % 4 and abs(e['lat']) < 0.6:
                e['tried'] = True                      # the way we came in
            node.exits.append(e)
        node.exits.sort(key=lambda e: (-e['card'], e['lat']))

    def plan_leg(self, x, y, c):
        """Distance to fly along cardinal c: first side opening beyond LEG_MIN_NODE, else just before
        the end wall, never more than MAX_SPACING."""
        a = self.card_yaw(c)
        dx, dy = math.cos(a), math.sin(a)
        Df = self.rd(x, y, a, self.LEG_MAX)
        if Df <= 0.6:
            return 0.0
        S = np.arange(0.0, Df, 0.1)
        cands = []
        for sgn in (1.0, -1.0):
            L = self.ray_dist(x + S * dx, y + S * dy, -dy * sgn, dx * sgn, 2.5, self.m_res)
            op = L > max(float(np.median(L[:8])) + 0.5, 1.2)
            i = 0
            while i < len(S):
                if not op[i]:
                    i += 1
                    continue
                j = i
                while j + 1 < len(S) and op[j + 1]:
                    j += 1
                if (j - i + 1) * 0.1 + 0.1 >= self.MIN_GAP and 0.5 * (S[i] + S[j]) >= self.LEG_MIN_NODE:
                    cands.append(float(0.5 * (S[i] + S[j])))
                i = j + 1
        return float(min(min(cands, default=1e9), max(0.0, Df - 0.5), self.MAX_SPACING))

    def center_goal(self, x, y, room):
        a = self.card_yaw(self.active_cardinal)
        dx, dy = math.cos(a), math.sin(a)
        Lp, Ln = self.rd(x, y, a + HALF_PI, 3.0, self.m_res), self.rd(x, y, a - HALF_PI, 3.0, self.m_res)
        v = 0.5 * (Lp - Ln) if (Lp < 2.5 and Ln < 2.5) else 0.0
        Df, Db = self.rd(x, y, a, 6.0), self.rd(x, y, a + math.pi, 6.0)
        u = 0.5 * (Df - Db) if (room and Df < 5.0 and Db < 5.0) else 0.0
        return x + u * dx - v * dy, y + u * dy + v * dx

    def room_box(self, x, y):
        """Room footprint: per side, the 25th-percentile wall distance over a fan of parallel rays across the room,
        so a doorway in the middle of a wall does not stretch the box into the corridor behind it."""
        cd, ext = self.card_dists(x, y), []
        for c in range(4):
            a = self.card_yaw(c)
            dx, dy = math.cos(a), math.sin(a)
            v = np.linspace(-0.9 * cd[(c + 3) % 4], 0.9 * cd[(c + 1) % 4], 13)   # fan across the room's full width
            D = self.ray_dist(x - v * dy, y + v * dx, dx, dy, 6.0)
            D = D[D > 0.2]                                    # rays starting inside blocked/unknown cells say nothing
            ext.append(float(np.percentile(D, 25)) if D.size else cd[c])
        a, ou, ov = self.card_yaw(0), (ext[0] - ext[2]) / 2.0, (ext[1] - ext[3]) / 2.0
        return (x + ou * math.cos(a) - ov * math.sin(a), y + ou * math.sin(a) + ov * math.cos(a),
                a, (ext[0] + ext[2]) / 2.0, (ext[1] + ext[3]) / 2.0)

    def room_viewpoint(self, n):
        """Once per room: if rays from its centre end in UNMAPPED space (not a wall), pick a look-from
        point part-way toward that gap. Not frontier exploration: one extra view, then the room is done."""
        if n.viewed:
            return None
        ang = np.arange(self.NB) * TAU / self.NB
        D = np.array([self.rd(n.x, n.y, a, 6.0) for a in ang])
        hx, hy = n.x + (D + self.m_res) * np.cos(ang), n.y + (D + self.m_res) * np.sin(ang)
        ix, iy = np.floor((hx - self.m_ox) / self.m_res).astype(int), np.floor((hy - self.m_oy) / self.m_res).astype(int)
        inb = (ix >= 0) & (ix < self.m_w) & (iy >= 0) & (iy < self.m_h)
        gap = (D < 6.0) & np.where(inb, self.unk[np.clip(iy, 0, self.m_h - 1), np.clip(ix, 0, self.m_w - 1)], True)
        if gap.mean() < self.UNSEEN_FRAC:
            return None
        n.viewed = True
        a = math.atan2(np.sin(ang[gap]).mean(), np.cos(ang[gap]).mean())
        for f in (0.6, 0.45, 0.3):
            vx, vy = n.x + f * np.median(D[gap]) * math.cos(a), n.y + f * np.median(D[gap]) * math.sin(a)
            if math.hypot(vx - n.x, vy - n.y) > 0.8 and self.seg_free(n.x, n.y, vx, vy):
                self.get_logger().info(f"[#{n.id}] {gap.mean():.0%} of rays end in unmapped space -> one extra viewpoint ({vx:.2f}, {vy:.2f})")
                return float(vx), float(vy)
        return None

    # ------------------------------------------------------------------ lidar (reactive layer)
    def scan_arrays(self):
        s = self.scan
        if s is None or len(s.ranges) == 0:
            return None
        if self._arr_msg is s:
            return self._arr
        fid = s.header.frame_id
        if fid not in self._R:
            if fid == self.BASE:
                self._R[fid] = np.eye(2)
            else:
                try:
                    q = self.tf_buffer.lookup_transform(self.BASE, fid, Time()).transform.rotation
                except Exception:
                    return None
                self._R[fid] = np.array([[1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y - q.z * q.w)],
                                         [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x ** 2 + q.z ** 2)]])
        R = self._R[fid]
        a = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
        ca, sa = np.cos(a), np.sin(a)
        ang = np.arctan2(R[1, 0] * ca + R[1, 1] * sa, R[0, 0] * ca + R[0, 1] * sa)
        r = np.asarray(s.ranges, np.float64)
        r = np.minimum(np.where(np.isposinf(r), s.range_max, r), s.range_max)
        bad = np.isnan(r) | np.isneginf(r) | (r < s.range_min)
        self._arr, self._arr_msg = (ang[~bad], r[~bad], ang, np.where(bad, np.nan, r)), s
        return self._arr

    def sector(self, rel, pct, half_deg=8.0):
        ang, rng = self.scan_arrays()[:2]
        sel = np.abs(wrap_angle(ang - rel)) <= math.radians(half_deg)
        return float(np.percentile(rng[sel], pct)) if sel.any() else self.scan.range_max

    # ------------------------------------------------------------------ wall facing
    def wall_axis_sums(self, yaw_add):
        """4-theta sums over short straight scan segments -> (sum w sin4t, sum w cos4t, sum w)."""
        _, _, ang, rng = self.scan_arrays()
        rmax = self.scan.range_max
        x, y = rng * np.cos(ang), rng * np.sin(ang)
        dx, dy = np.diff(x), np.diff(y)
        L = np.hypot(dx, dy)
        with np.errstate(invalid='ignore'):
            ok = np.isfinite(L) & (L > 0.01) & (L < 0.2) & (rng[:-1] < 0.95 * rmax) & (rng[1:] < 0.95 * rmax) & (rng[:-1] < 6.0)
        if not ok.any():
            return 0.0, 0.0, 0.0
        th, w = np.arctan2(dy[ok], dx[ok]) + yaw_add, L[ok]
        return float(np.sum(w * np.sin(4 * th))), float(np.sum(w * np.cos(4 * th))), float(w.sum())

    def trim_frame(self, yaw, gain):
        """Pull the cardinal frame onto the real walls (only valid while facing the active cardinal)."""
        s, c, w = self.wall_axis_sums(0.0)
        if w < self.WALL_MIN_LEN or math.hypot(s, c) / w < self.WALL_CONF:
            return
        delta = math.atan2(s, c) / 4.0                       # walls appear rotated by delta in the body frame
        diff = wrap_angle(yaw + delta - self.active_cardinal * HALF_PI - self.frame_offset)
        if abs(delta) < self.WALL_MAX_DELTA and abs(diff) < self.WALL_MAX_DELTA:
            self.frame_offset = wrap_angle(self.frame_offset + gain * diff)
            self.set_cardinal(self.active_cardinal)

    # ------------------------------------------------------------------ coverage
    def feed_coverage(self, x, y, yaw):
        if self.cur is None or self.scan is self._fed or self.scan_arrays() is None:
            return
        n = self.nodes[self.cur]
        fx, fy = self.hold_xy if self.state == State.SWEEP else (n.x, n.y)
        if math.hypot(x - fx, y - fy) > self.FEED_R:
            return
        self._fed = self.scan
        idx = np.floor((wrap_angle(self.scan_arrays()[0] + yaw) + math.pi) / TAU * self.NB).astype(np.int64) % self.NB
        n.cover += (np.bincount(idx, minlength=self.NB) >= 2).astype(np.int32)

    def uncovered(self, node, region):
        if region == 'all':
            idx = np.arange(self.NB)
        else:
            idx = np.where(np.abs(wrap_angle(self.bin_c - self.card_yaw(self.active_cardinal))) <= self.FRONT_HALF)[0]
        return idx[node.cover[idx] < self.MIN_HITS]

    def start_sweep(self, region, x, y):
        self.sweep_region, self.hold_xy, self.last_yaw, self.sweep_dir = region, (x, y), None, 0.0
        self.set_state(State.SWEEP)

    # ------------------------------------------------------------------ command output
    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def publish_safe_cmd(self, vx, vy, wz):
        """Direction-aware shield on the raw scan (base frame, FLU)."""
        arr = self.scan_arrays()
        if arr is not None and arr[1].size:
            ang, rng = arr[0], arr[1]
            px, py = rng * np.cos(ang), rng * np.sin(ang)
            s = math.hypot(vx, vy)
            if s > 1e-3:
                ux, uy = vx / s, vy / s
                ahead, lat = px * ux + py * uy, np.abs(-px * uy + py * ux)
                m = (ahead > 0.0) & (lat < self.ROBOT_RADIUS)
                if m.any():
                    c = float(ahead[m].min()) - self.HARD_STOP
                    v_ok = 0.0 if c <= 0 else self.DECEL * (-self.LATENCY + math.sqrt(self.LATENCY ** 2 + 2 * c / self.DECEL))
                    if v_ok < s:
                        vx, vy = ux * v_ok, uy * v_ok
            i = int(np.argmin(rng))
            if rng[i] < self.REPULSE:
                nx, ny = -px[i] / rng[i], -py[i] / rng[i]
                toward = -(vx * nx + vy * ny)
                if toward > 0.0:
                    vx, vy = vx + toward * nx, vy + toward * ny
                vx, vy = vx + 0.15 * nx, vy + 0.15 * ny
                self.get_logger().warn(f"SHIELD repulse {rng[i]:.2f} m", throttle_duration_sec=1.0)
        msg = Twist()
        msg.linear.x, msg.linear.y = float(np.clip(vx, -0.45, 0.45)), float(np.clip(vy, -0.35, 0.35))
        msg.angular.z = float(np.clip(wz, -0.8, 0.8))
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------ main loop
    def control_loop(self):
        if self.state == State.COMPLETE:
            return self.publish_zero()
        if self.scan is None or self.now_s() - self.scan_t > self.SCAN_TIMEOUT or self.scan_arrays() is None \
                or self.blocked is None:
            self.get_logger().warn("Waiting for fresh /scan and /map.", throttle_duration_sec=2.0)
            return self.publish_zero()
        poses = self.get_poses()
        if poses is None:
            return self.publish_zero()
        (x, y, yaw), (ox, oy, oyaw) = poses
        self._map_yaw, self._odom_yaw, self._pose, self._odom_xy = yaw, oyaw, (x, y, yaw), (ox, oy)
        if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 0.1:
            self.trail = (self.trail + [(x, y)])[-3000:]
        self.feed_coverage(x, y, yaw)
        now, st, t = self.now_s(), self.state, self.now_s() - self.state_t0
        e_yaw = wrap_angle(self.target_odom_yaw - oyaw)

        if st == State.INIT:                      # building axes from walls, then register home
            self.publish_zero()
            s, c, _ = self.wall_axis_sums(yaw)
            self._init[0], self._init[1], self._init[2] = self._init[0] + s, self._init[1] + c, self._init[2] + 1
            if self._init[2] >= 20:
                self.frame_offset = math.atan2(self._init[0], self._init[1]) / 4.0 \
                    if math.hypot(self._init[0], self._init[1]) > 1.0 else 0.0
                self.set_cardinal(self.snap_card(yaw))
                self.cur = self.registry.create(x, y, False).id
                self.get_logger().info(f"HOME node #0 at map ({x:.2f}, {y:.2f}); axis {math.degrees(self.frame_offset):.1f} deg")
                self.set_state(State.CENTER)

        elif st == State.CENTER:                  # drive to the cross-section centre, face the walls squarely
            self.trim_frame(yaw, 0.1)
            e_yaw = wrap_angle(self.target_odom_yaw - oyaw)
            gx, gy = self.center_goal(x, y, self.is_room(self.card_dists(x, y)))
            ex, ey = self.to_base(gx - x, gy - y, yaw)
            if (abs(ex) < self.CENTER_TOL and abs(ey) < self.CENTER_TOL and abs(e_yaw) < 0.08) or t > self.CENTER_TIMEOUT:
                self.stop()
                self.set_state(State.WAIT_MAP if self.mode == 'explore' else State.DECIDE)
            else:
                self.publish_safe_cmd(float(np.clip(0.8 * ex, -0.25, 0.25)), float(np.clip(0.8 * ey, -0.25, 0.25)), 1.2 * e_yaw)

        elif st == State.WAIT_MAP:                # let slam_toolbox publish a map that includes what we saw
            self.stop()
            if self.map_t > self.state_t0 or t > self.MAP_WAIT:
                if self.refresh:
                    n = self.nodes[self.cur]
                    self.add_exits(n, self.compute_exits(n.x, n.y))
                    self.refresh = False
                    vp = None
                    if n.is_room:
                        n.box = self.room_box(n.x, n.y)           # footprint with the post-sweep map
                        vp = self.room_viewpoint(n)
                    if vp:
                        self.mode, self.active_exit, self.wps = 'explore', None, [vp]
                        self.after_view, self.front_checked = True, True
                        self.set_cardinal(self.snap_card(yaw))
                        self.set_state(State.ALIGN)
                    else:
                        self.set_state(State.DECIDE)
                else:
                    self.register(x, y)

        elif st == State.SWEEP:                   # rotate in place only until the needed bearings were seen
            n = self.nodes[self.cur]
            if self.last_yaw is None:
                self.last_yaw, self.spin, self.sweep_dir = oyaw, 0.0, 0.0
            self.spin += abs(wrap_angle(oyaw - self.last_yaw))
            self.last_yaw = oyaw
            need = self.uncovered(n, self.sweep_region)
            gave_up = self.spin >= (TAU + 0.1 if self.sweep_region == 'all' else math.pi) or t > self.SPIN_TIMEOUT
            if (t > self.SETTLE and need.size == 0) or gave_up:
                if gave_up and need.size:
                    self.get_logger().warn(f"[#{n.id}] sweep '{self.sweep_region}' gave up, {need.size} bins unseen (blind spot?)")
                self.stop()
                self.last_yaw = None
                if self.sweep_region == 'all':
                    self.set_cardinal(self.snap_card(yaw))
                    self.refresh = True
                    self.set_state(State.WAIT_MAP)
                else:
                    self.set_cardinal(self.active_cardinal)
                    self.set_state(State.ALIGN)
                return
            wz = 0.0
            if t > self.SETTLE and need.size:
                if self.sweep_dir == 0.0:
                    rel = wrap_angle(self.bin_c[need] - yaw)
                    self.sweep_dir = 1.0 if rel[int(np.argmin(np.abs(rel)))] >= 0 else -1.0
                wz = self.SWEEP_RATE * self.sweep_dir
            ex, ey = self.to_base(self.hold_xy[0] - x, self.hold_xy[1] - y, yaw)
            self.publish_safe_cmd(float(np.clip(0.8 * ex, -0.2, 0.2)), float(np.clip(0.8 * ey, -0.2, 0.2)), wz)

        elif st == State.DECIDE:
            self.decide(x, y)

        elif st == State.ALIGN:                   # in-place pivot, then verify the area ahead has been seen
            if abs(e_yaw) < 0.05 or t > self.ALIGN_TIMEOUT:
                self.stop()
                if self.mode == 'explore' and not self.front_checked:
                    self.front_checked = True
                    return self.start_sweep('front', x, y)
                self.reset_progress()
                self.set_state(State.GOTO)
            else:
                self.publish_safe_cmd(0.0, 0.0, 1.5 * e_yaw)

        elif st == State.GOTO:                    # one mover for legs, backtracking and the way home
            if self.mode == 'explore':
                self.trim_frame(yaw, 0.03)
            ex, ey = self.to_base(self.wps[0][0] - x, self.wps[0][1] - y, yaw)
            dist = math.hypot(ex, ey)
            d_f, d_l, d_r = self.sector(0.0, 20), self.sector(HALF_PI, 25), self.sector(-HALF_PI, 25)
            if d_l < self.SERVO_MAX and d_r < self.SERVO_MAX:
                vy = 0.85 * (d_l - d_r) / 2.0                 # corridor centring
            elif d_l < 0.55:
                vy = -0.2
            elif d_r < 0.55:
                vy = 0.2
            else:
                vy = float(np.clip(0.8 * ey, -0.25, 0.25))
            speed = self.DOORWAY_SPEED if d_l + d_r < 0.95 else self.CORRIDOR_SPEED
            vx = min(speed, max(0.0, 0.8 * ex), max(0.0, 0.65 * (d_f - self.HARD_STOP)))
            if math.hypot(ox - self.prog[0], oy - self.prog[1]) > 0.1:
                self.prog, self.prog_t = (ox, oy), now
            close = d_f <= self.HARD_STOP + 0.12
            self.block_t = (now if self.block_t is None else self.block_t) if close else None
            last = self.mode == 'explore' and len(self.wps) == 1 or self.mode != 'explore' and len(self.route) == 1
            if dist <= (self.ARRIVE_TOL if last else self.PASS_TOL) or (close and dist < 0.8):
                self.arrive(x, y)
            elif now - self.prog_t > self.STUCK_TIMEOUT:
                self.abort(x, y, "no progress")
            elif self.block_t is not None and now - self.block_t > self.OBSTRUCT_TIMEOUT:
                self.abort(x, y, f"obstructed at {d_f:.2f} m")
            else:
                self.publish_safe_cmd(vx, vy, 1.6 * e_yaw)

    # ------------------------------------------------------------------ decisions & routing
    def decide(self, x, y):
        """Untried exit here -> take it. Else go to the nearest node (by path) with an untried exit.
        Nothing left -> shortest path to node 0."""
        node = self.nodes[self.cur]
        self.front_checked, self.block_t = False, None
        for ex in node.open_exits():
            ex['tried'], a = True, self.card_yaw(ex['card'])
            gx, gy = ex['goal'] or (x, y)                          # plan from the door, not from the room centre
            s = self.plan_leg(gx, gy, ex['card'])
            tgt = (gx + s * math.cos(a), gy + s * math.sin(a))
            known = self.registry.find(tgt[0], tgt[1], lambda n: True)[0] if s > 0.5 else None   # s~0: nothing mapped beyond the door yet
            if known and known.id == node.id and not node.contains(*tgt):
                known = None                                       # own radius is not "known": only its room footprint counts
            if known or (not ex['goal'] and s < self.registry.merge):
                ex['known'] = True                                 # lands in a known room/node, or too short to ever make a node
                self.get_logger().info(f"[#{node.id}] exit card={ex['card']} skipped ("
                                       + (f"leads into known #{known.id}" if known else f"leg only {s:.2f} m") + ").")
                continue
            self.active_exit, self.mode = ex, 'explore'
            self.set_cardinal(ex['card'])
            self.wps = ([ex['goal']] if ex['goal'] else []) + [tgt]
            self.get_logger().info(f"[#{node.id}] exit card={ex['card']} lat={ex['lat']:.2f} leg={s:.2f} m")
            return self.set_state(State.ALIGN)
        best = None
        for n in self.nodes.values():
            if n.open_exits():
                p, c = self.registry.path(self.cur, n.id)
                if p and (best is None or c < best[1]):
                    best = (p, c)
        goal = best[0] if best else self.registry.path(self.cur, 0)[0]
        if not goal or len(goal) < 2:
            self.get_logger().info(f"DONE: {len(self.nodes) - 1} nodes explored, at home.")
            return self.set_state(State.COMPLETE)
        self.get_logger().info(f"[#{node.id}] {'-> node #%d' % goal[-1] if best else 'exploration complete, going HOME'}: path {goal}")
        self.start_route(goal[1:], 'route' if best else 'home', x, y)

    def start_route(self, ids, mode, x, y):
        self.mode, self.route, self.route_goal = mode, ids, ids[-1]
        self.fails, self.active_exit = 0, None
        self.pick_wp(x, y)

    def pick_wp(self, x, y):
        """Always aim at the NEXT node on the path (the nodes we flew through), never skip ahead."""
        n = self.nodes[self.route[0]]
        self.wps = [(n.x, n.y)]
        self.face(math.atan2(n.y - y, n.x - x))
        self.set_state(State.ALIGN)

    def arrive(self, x, y):
        self.stop()
        self.reset_progress()
        if self.mode == 'explore':
            self.wps.pop(0)
            if not self.wps:
                if self.after_view:                  # reached the extra viewpoint: look again from here
                    self.after_view = False
                    self.nodes[self.cur].cover[:] = 0
                    return self.start_sweep('all', x, y)
                self.set_state(State.CENTER)
            return
        self.cur, self.route = self.route[0], self.route[1:]
        self.fails = 0
        if self.route:
            self.pick_wp(x, y)
        else:
            self.end_route()

    def end_route(self):
        self.set_cardinal(self.snap_card(self._map_yaw))
        self.set_state(State.COMPLETE if self.mode == 'home' else State.CENTER)

    def abort(self, x, y, why):
        self.stop()
        self.get_logger().warn(f"Leg aborted ({why}).")
        if self.mode == 'explore':                # exit is blocked: go back to the node we left
            if self.active_exit:
                self.active_exit['blocked'] = True
            return self.start_route([self.cur], 'route', x, y)
        self.fails += 1                           # retry the same hop a few times, then give up
        if self.fails > 3:
            self.get_logger().warn("Route keeps failing; stopping here.")
            self.cur, self.route = self.route_goal, []
            return self.end_route()
        self.pick_wp(x, y)

    def register(self, x, y):
        """Arrival on a forward leg: merge into a VISIBLE known node, else create a new one."""
        src = self.nodes[self.cur]
        other, d = self.registry.find(x, y, lambda n: self.seg_free(x, y, n.x, n.y))
        if other:
            if other.id == src.id:
                if self.active_exit:
                    self.active_exit['blocked'] = True
                self.get_logger().info(f"Leg too short ({d:.2f} m); exit blocked.")
            else:
                self.registry.link(src, other)
                self.loops.append((src.id, other.id))
                for e in other.exits:
                    if e['card'] == (self.active_cardinal + 2) % 4 and abs(e['lat']) < 0.8:
                        e['tried'] = True
                self.get_logger().info(f"REVISIT #{other.id} (d={d:.2f} m, line clear): loop link #{src.id}<->#{other.id}")
            self.cur = other.id
            return self.set_state(State.DECIDE)
        near = min(((math.hypot(n.x - x, n.y - y), n.id) for n in self.nodes.values()), default=(0.0, -1))
        node = self.registry.create(x, y, self.is_room(self.card_dists(x, y)))
        self.registry.link(src, node)
        node.entry_card = self.active_cardinal if src.id else None
        node.box = self.room_box(x, y) if node.is_room else None
        self.add_exits(node, self.compute_exits(x, y))
        self.cur = node.id
        self.get_logger().info(
            f"NODE CREATED #{node.id} {'ROOM' if node.is_room else 'corridor'} at map ({x:.2f}, {y:.2f}) | "
            f"nearest #{near[1]} at {near[0]:.2f} m | from #{src.id} | exits "
            + (", ".join(f"c{e['card']}@{e['lat']:.2f}" for e in node.exits) or "none"))
        self.start_sweep('all', x, y) if node.is_room else self.set_state(State.DECIDE)

    # ------------------------------------------------------------------ RViz debug
    def debug_cb(self):
        if self._pose is None:
            return
        x, y, yaw = self._pose
        ms, stamp = [], self.get_clock().now().to_msg()
        clear = Marker()
        clear.header.frame_id, clear.action = self.MAP, Marker.DELETEALL
        ms.append(clear)

        def add(ns, kind, col, sc, pos=None, pts=None, cols=None, txt=None):
            m = Marker()
            m.header.frame_id, m.header.stamp, m.ns, m.id, m.type, m.action = self.MAP, stamp, ns, len(ms), kind, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = float(sc[0]), float(sc[1]), float(sc[2])
            m.color = ColorRGBA(r=float(col[0]), g=float(col[1]), b=float(col[2]), a=float(col[3]))
            if pos:
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
            if pts:
                m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2] if len(p) > 2 else 0.1)) for p in pts]
            if cols:
                m.colors = [ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=float(c[3])) for c in cols]
            if txt is not None:
                m.text = txt
            ms.append(m)

        circle = np.linspace(0.0, TAU, 33)
        loops = {frozenset(p) for p in self.loops}
        for n in self.nodes.values():
            col = (0.1, 1, 0.2, 1) if n.id == self.cur else (1, 1, 1, 1) if n.id == 0 else \
                (1, 0.55, 0, 1) if n.is_room else (0, 0.8, 1, 1)
            add('nodes', Marker.SPHERE, col, (0.3, 0.3, 0.3), pos=(n.x, n.y, 0.1))
            add('merge', Marker.LINE_STRIP, col[:3] + (0.35,), (0.02, 0, 0),
                pts=[(n.x + self.registry.merge * math.cos(a), n.y + self.registry.merge * math.sin(a)) for a in circle])
            if n.box:
                cx, cy, a, hl, hw = n.box
                add('room_box', Marker.LINE_STRIP, (1, 0.55, 0, 0.6), (0.03, 0, 0), pts=[
                    (cx + su * hl * math.cos(a) - sv * hw * math.sin(a), cy + su * hl * math.sin(a) + sv * hw * math.cos(a))
                    for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1), (1, 1))])
            add('label', Marker.TEXT_VIEW_FACING, (1, 1, 1, 1), (0, 0, 0.22), pos=(n.x, n.y, 0.55),
                txt=f"#{n.id}{'H' if n.id == 0 else 'R' if n.is_room else 'C'} open={len(n.open_exits())}")
            for j in n.links:
                if j > n.id:
                    m2 = self.nodes[j]
                    add('links', Marker.LINE_LIST, (1, 0.1, 0.1, 0.9) if frozenset((n.id, j)) in loops else (1, 1, 1, 0.8),
                        (0.04, 0, 0), pts=[(n.x, n.y), (m2.x, m2.y)])
            for e in n.exits:
                col = (1, 0.1, 0.1, 0.9) if e['blocked'] else (1, 0, 1, 1) if e is self.active_exit else \
                    (1, 0.55, 0, 0.8) if e.get('known') else (0.5, 0.5, 0.5, 0.6) if e['tried'] else (1, 1, 0, 0.9)
                a = self.card_yaw(e['card'])
                end = e['goal'] or (n.x + math.cos(a), n.y + math.sin(a))
                add('exits', Marker.ARROW, col, (0.04, 0.09, 0.12), pts=[(n.x, n.y), end])
        if len(self.trail) > 1:
            add('trail', Marker.LINE_STRIP, (0.4, 0.6, 1, 0.6), (0.02, 0, 0), pts=[(p[0], p[1], 0.03) for p in self.trail])
        path = self.wps if self.mode == 'explore' else [(self.nodes[i].x, self.nodes[i].y) for i in self.route]
        if path and self.state != State.COMPLETE:
            add('route', Marker.LINE_STRIP, (1, 0.6, 0, 0.9), (0.05, 0, 0), pts=[(x, y)] + list(path))
        add('drone', Marker.ARROW, (1, 1, 1, 1), (0.05, 0.12, 0.15),
            pts=[(x, y, 0.2), (x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw), 0.2)])
        ca = self.card_yaw(self.active_cardinal)
        add('cardinal', Marker.ARROW, (1, 0, 1, 0.9), (0.03, 0.08, 0.12),
            pts=[(x, y, 0.25), (x + math.cos(ca), y + math.sin(ca), 0.25)])
        add('state', Marker.TEXT_VIEW_FACING, (1, 1, 0.3, 1), (0, 0, 0.22), pos=(x, y, 0.8),
            txt=f"{self.state.name} [{self.mode}]")
        if self.cur is not None:                  # coverage ring of the current node
            n, pts, cols = self.nodes[self.cur], [], []
            for i in range(self.NB):
                col = (0.1, 1, 0.1, 0.9) if n.cover[i] >= self.MIN_HITS else (1, 0.1, 0.1, 0.9)
                for a in (self.bin_c[i] - 0.4 * TAU / self.NB, self.bin_c[i] + 0.4 * TAU / self.NB):
                    pts.append((n.x + 0.75 * math.cos(a), n.y + 0.75 * math.sin(a)))
                    cols.append(col)
            add('coverage', Marker.LINE_LIST, (1, 1, 1, 1), (0.06, 0, 0), pts=pts, cols=cols)
        self.dbg_pub.publish(MarkerArray(markers=ms))


def main(args=None):
    rclpy.init(args=args)
    node = AriadneMapNav()
    try:
        rclpy.spin(node)
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()