#!/usr/bin/env python3
"""
Room-Constrained Next-Best-View (NBV) + Frontier Explorer for ROS 2 / Nav2.
Replaces fixed-step Tremaux DFS with:
  1. EDT Doorway Gate Segmentation (locks the active room until 100% cleared)
  2. Live Directional Camera Frustum Raycasting (published to explorer/coverage)
  3. Time-Normalized Next-Best-View (rotates in place to sweep walls, then clears corners)
  4. Geodesic BFS Frontier Selection (never plans through walls, minimizes backtracking)
"""
import math
from collections import deque

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# --- Clearance & Geometry Parameters ---
CLEAR_M = 0.28            # Minimum safe wall clearance for any goal pose (m)
PATH_CLEAR_M = 0.18       # Minimum clearance along straight line-of-sight rays (m)
DOOR_HALF_WIDTH_M = 0.58  # EDT choke threshold for doorway Virtual Gates (~1.15m doors)
ROOM_OPEN_M = 0.75        # EDT clearance that indicates an open room interior (m)

# --- Camera / Visual Coverage Parameters ---
CAM_FOV_DEG = 75.0        # Horizontal FOV of forward camera (degrees)
CAM_RANGE_M = 3.50        # Effective visual inspection range (m)
STANDOFF_OPT_M = 1.60     # Ideal standoff distance from unseen wall targets (m)
STANDOFF_TIGHT_M = 0.85   # Fallback standoff for narrow rooms/alcoves (m)

# --- Motion Cost Constants (for NBV Gain/Time scoring) ---
V_MAX = 0.65              # Approximate linear speed (m/s)
W_MAX = 1.10              # Approximate angular speed (rad/s)
MIN_FRONTIER_CELLS = 6    # Minimum contiguous frontier pixels to form a valid cluster


def wrap_angle(angle):
    """Wrap angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class Explorer(Node):
    def __init__(self):
        super().__init__("dfs_explorer")
        self.g = None
        self.clear = None
        self.vis = None        # Visual coverage grid: 0=Unseen, 1=SeenFloor, 2=InspectedWall
        self.blacklist = None  # Unreachable/stuck target mask
        self.map_header = None

        self.res = 0.05
        self.ox = 0.0
        self.oy = 0.0
        self.home = None

        # State tracking
        self.busy = False
        self.cur_goal = None   # (gx, gy, gyaw, mode, target_rc)
        self.last_pose = None
        self.last_pose_time = None
        self.omega_z = 0.0
        self.sweep_dir = 1.0   # +1 CCW, -1 CW (keeps room rotation monotonic)
        self.visit_counts = {} # {(r_bin, c_bin): attempts} to prevent infinite loops
        self.visited_history = []

        # Viz caches
        self.viz_room_center = None
        self.viz_candidates = []
        self.viz_frontiers = []

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cov_pub = self.create_publisher(OccupancyGrid, "explorer/coverage", qos)
        self.mark_pub = self.create_publisher(MarkerArray, "explorer/stack", 1)

        # Fast timer updates camera frustum coverage; main timer drives planner
        self.create_timer(0.15, self.update_visual_coverage)
        self.create_timer(0.40, self.tick)
        self.create_timer(1.00, self.viz)
        self.get_logger().info("Room-Constrained NBV + Frontier Explorer started.")

    # =========================================================================
    # 1. MAP & VISUAL COVERAGE UPDATES
    # =========================================================================
    def on_map(self, msg):
        i = msg.info
        if i.height == 0 or i.width == 0:
            return
        new_g = np.array(msg.data, dtype=np.int16).reshape(i.height, i.width)
        new_res = float(i.resolution)
        new_ox = float(i.origin.position.x)
        new_oy = float(i.origin.position.y)

        # Preserve visual coverage across SLAM map resizes
        if self.vis is None or self.vis.shape != new_g.shape or abs(new_ox - self.ox) > 1e-3 or abs(new_oy - self.oy) > 1e-3:
            new_vis = np.zeros_like(new_g, dtype=np.uint8)
            new_bl = np.zeros_like(new_g, dtype=bool)
            if self.vis is not None and abs(new_res - self.res) < 1e-4:
                dr = int(round((self.oy - new_oy) / new_res))
                dc = int(round((self.ox - new_ox) / new_res))
                H_old, W_old = self.vis.shape
                H_new, W_new = new_g.shape
                r0_old = max(0, -dr)
                c0_old = max(0, -dc)
                r1_old = min(H_old, H_new - dr)
                c1_old = min(W_old, W_new - dc)
                if r1_old > r0_old and c1_old > c0_old:
                    new_vis[r0_old + dr : r1_old + dr, c0_old + dc : c1_old + dc] = self.vis[r0_old:r1_old, c0_old:c1_old]
                    new_bl[r0_old + dr : r1_old + dr, c0_old + dc : c1_old + dc] = self.blacklist[r0_old:r1_old, c0_old:c1_old]
            self.vis = new_vis
            self.blacklist = new_bl

        self.g = new_g
        self.res = new_res
        self.ox = new_ox
        self.oy = new_oy
        self.map_header = msg.header

        # Treat both walls (>= 50) and unknown (< 0) as obstacles for safe clearance
        free_mask = ((self.g >= 0) & (self.g < 50)).astype(np.uint8)
        self.clear = cv2.distanceTransform(free_mask, cv2.DIST_L2, 3) * self.res

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        now = self.get_clock().now().nanoseconds * 1e-9
        x, y = float(t.transform.translation.x), float(t.transform.translation.y)

        if self.last_pose is not None and self.last_pose_time is not None:
            dt = now - self.last_pose_time
            if 0.02 < dt < 1.0:
                self.omega_z = abs(wrap_angle(yaw - self.last_pose[2])) / dt
        self.last_pose = (x, y, float(yaw))
        self.last_pose_time = now
        return (x, y, float(yaw))

    def world_to_rc(self, x, y):
        c = int(math.floor((x - self.ox) / self.res))
        r = int(math.floor((y - self.oy) / self.res))
        return r, c

    def rc_to_world(self, r, c):
        x = float(self.ox + (c + 0.5) * self.res)
        y = float(self.oy + (r + 0.5) * self.res)
        return x, y

    def update_visual_coverage(self):
        """Raycasts the camera frustum into self.vis when rotation is stable."""
        if self.g is None or self.vis is None:
            return
        p = self.pose()
        if p is None:
            return
        # Avoid smearing visual coverage during fast yaw spins
        if self.omega_z > 0.85:
            return

        x, y, yaw = p
        H, W = self.g.shape
        r0, c0 = self.world_to_rc(x, y)
        if not (0 <= r0 < H and 0 <= c0 < W):
            return

        fov_rad = math.radians(CAM_FOV_DEG)
        max_steps = int(CAM_RANGE_M / self.res)
        num_rays = max(25, int(CAM_FOV_DEG * 0.8))
        angles = np.linspace(yaw - fov_rad * 0.5, yaw + fov_rad * 0.5, num_rays)

        steps = np.arange(1, max_steps + 1, dtype=np.float32)[:, None]
        rs = np.floor(r0 + steps * np.sin(angles)[None, :]).astype(int)
        cs = np.floor(c0 + steps * np.cos(angles)[None, :]).astype(int)

        in_bounds = (rs >= 0) & (rs < H) & (cs >= 0) & (cs < W)
        rs_safe = np.clip(rs, 0, H - 1)
        cs_safe = np.clip(cs, 0, W - 1)
        vals = np.where(in_bounds, self.g[rs_safe, cs_safe], 100)

        blocked = (vals < 0) | (vals >= 50)
        first_hit = np.argmax(blocked, axis=0)
        has_hit = blocked[first_hit, np.arange(num_rays)]
        stop_idx = np.where(has_hit, first_hit, max_steps)

        # Mark free floor cells along unblocked ray segments as Seen (1)
        step_grid = np.arange(max_steps)[:, None]
        free_vis = (step_grid < stop_idx[None, :]) & in_bounds
        fr, fc = rs_safe[free_vis], cs_safe[free_vis]
        self.vis[fr, fc] = np.maximum(self.vis[fr, fc], 1)

        # Mark the wall cell where each ray terminated as Inspected Wall (2)
        for k in range(num_rays):
            if has_hit[k]:
                idx = int(first_hit[k])
                hr, hc = int(rs_safe[idx, k]), int(cs_safe[idx, k])
                if in_bounds[idx, k] and self.g[hr, hc] >= 50:
                    self.vis[hr - 1 : hr + 2, hc - 1 : hc + 2] = 2

    # =========================================================================
    # 2. LINE-OF-SIGHT & ROOM SEGMENTATION (THE GATEKEEPER)
    # =========================================================================
    def line_of_sight_free(self, x0, y0, x1, y1, min_clear=PATH_CLEAR_M):
        """Pixel-exact raycast from (x0, y0) to (x1, y1) ensuring zero wall crossings."""
        H, W = self.g.shape
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(4, int(math.ceil(dist / (self.res * 0.5))))
        xs = np.linspace(x0, x1, steps)
        ys = np.linspace(y0, y1, steps)
        mc = np.floor((xs - self.ox) / self.res).astype(int)
        mr = np.floor((ys - self.oy) / self.res).astype(int)

        if bool(((mr < 0) | (mr >= H) | (mc < 0) | (mc >= W)).any()):
            return False

        vals = self.g[mr, mc]
        if bool((vals < 0).any() or (vals >= 50).any()):
            return False

        start_idx = min(steps - 1, max(1, int(0.20 / (self.res * 0.5))))
        if bool((self.clear[mr[start_idx:], mc[start_idx:]] < min_clear).any()):
            return False
        return True

    def segment_current_room(self, r0, c0):
        """
        Flood-fills the current room from (r0, c0) using EDT.
        Halts expansion whenever clearance drops into a doorway choke point (Virtual Gate)
        AFTER the flood-fill has already experienced open room space.
        """
        H, W = self.g.shape
        free_mask = (self.g >= 0) & (self.g < 50)
        gate_zone = free_mask & (self.clear <= DOOR_HALF_WIDTH_M)

        room_mask = np.zeros((H, W), dtype=bool)
        if not (0 <= r0 < H and 0 <= c0 < W):
            return room_mask, gate_zone

        max_rad = int(8.5 / self.res)
        q = deque([(r0, c0, float(self.clear[r0, c0]))])
        room_mask[r0, c0] = True

        while q:
            r, c, peak_cl = q.popleft()
            if abs(r - r0) > max_rad or abs(c - c0) > max_rad:
                continue
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < H and 0 <= nc < W and not room_mask[nr, nc] and free_mask[nr, nc]:
                    cl = float(self.clear[nr, nc])
                    # Stop at doorway choke point if transitioning out of an open room
                    if peak_cl >= ROOM_OPEN_M and cl <= DOOR_HALF_WIDTH_M:
                        continue
                    room_mask[nr, nc] = True
                    q.append((nr, nc, max(peak_cl, cl)))

        return room_mask, gate_zone

    def compute_bfs_distance_map(self, r0, c0):
        """Computes geodesic (wall-respecting) distance in meters from (r0, c0) across the map."""
        H, W = self.g.shape
        dist_map = np.full((H, W), np.inf, dtype=np.float32)
        navigable = (self.g >= 0) & (self.g < 50) & (self.clear >= PATH_CLEAR_M)

        # Allow seed neighborhood in case robot is momentarily close to a wall
        r_min, r_max = max(0, r0 - 3), min(H, r0 + 4)
        c_min, c_max = max(0, c0 - 3), min(W, c0 + 4)
        navigable[r_min:r_max, c_min:c_max] = (self.g[r_min:r_max, c_min:c_max] >= 0) & (
            self.g[r_min:r_max, c_min:c_max] < 50
        )

        q = deque([(r0, c0)])
        dist_map[r0, c0] = 0.0
        step_m = self.res

        while q:
            r, c = q.popleft()
            d_next = dist_map[r, c] + step_m
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < H and 0 <= nc < W and navigable[nr, nc]:
                    if d_next < dist_map[nr, nc]:
                        dist_map[nr, nc] = d_next
                        q.append((nr, nc))
        return dist_map

    # =========================================================================
    # 3. ROOM-CONSTRAINED NEXT-BEST-VIEW (INNER LOOP)
    # =========================================================================
    def find_in_room_targets(self, room_mask):
        """Finds uninspected wall faces and unknown structural frontiers inside room_mask."""
        kernel = np.ones((3, 3), dtype=np.uint8)
        dilated_room = cv2.dilate(room_mask.astype(np.uint8), kernel, iterations=2).astype(bool)

        # 1. Uninspected walls bordering the current room
        unseen_walls = (
            (self.g >= 50)
            & (self.vis < 2)
            & dilated_room
            & (~self.blacklist)
        )
        # 2. Unknown structural frontiers inside the current room
        unk_adj = cv2.dilate((self.g < 0).astype(np.uint8), kernel, iterations=1).astype(bool)
        in_room_frontiers = (
            room_mask
            & (self.g >= 0)
            & (self.g < 50)
            & unk_adj
            & (~self.blacklist)
        )
        return unseen_walls, in_room_frontiers

    def score_frustum_gain(self, vr, vc, vyaw, unseen_walls, in_room_frontiers):
        """Simulates camera frustum raycast from (vr, vc, vyaw) and returns weighted InfoGain."""
        H, W = self.g.shape
        fov_rad = math.radians(CAM_FOV_DEG)
        max_steps = int(CAM_RANGE_M / self.res)
        num_rays = 15
        angles = np.linspace(vyaw - fov_rad * 0.45, vyaw + fov_rad * 0.45, num_rays)

        steps = np.arange(1, max_steps + 1, dtype=np.float32)[:, None]
        rs = np.floor(vr + steps * np.sin(angles)[None, :]).astype(int)
        cs = np.floor(vc + steps * np.cos(angles)[None, :]).astype(int)

        in_b = (rs >= 0) & (rs < H) & (cs >= 0) & (cs < W)
        rs_s = np.clip(rs, 0, H - 1)
        cs_s = np.clip(cs, 0, W - 1)
        vals = np.where(in_b, self.g[rs_s, cs_s], 100)

        blocked = (vals < 0) | (vals >= 50)
        first_hit = np.argmax(blocked, axis=0)
        has_hit = blocked[first_hit, np.arange(num_rays)]
        stop_idx = np.where(has_hit, first_hit, max_steps)

        # Structural frontier gain along open rays
        step_grid = np.arange(max_steps)[:, None]
        valid_ray = (step_grid <= stop_idx[None, :]) & in_b
        struct_hits = in_room_frontiers[rs_s[valid_ray], cs_s[valid_ray]]
        struct_gain = int(np.count_nonzero(struct_hits))

        # Unseen wall inspection gain at ray terminations
        wall_gain = 0
        for k in range(num_rays):
            if has_hit[k]:
                idx = int(first_hit[k])
                hr, hc = int(rs_s[idx, k]), int(cs_s[idx, k])
                if in_b[idx, k] and unseen_walls[hr, hc]:
                    wall_gain += 4  # Weight wall inspection highly so rooms finish cleanly

        return float(wall_gain + struct_gain)

    def select_room_nbv(self, p, r0, c0, room_mask, bfs_dist):
        """Samples viewpoints inside room_mask and picks max InfoGain / ExecutionTime."""
        x0, y0, yaw0 = p
        unseen_walls, in_room_frontiers = self.find_in_room_targets(room_mask)

        total_unseen_walls = int(np.count_nonzero(unseen_walls))
        total_room_frontiers = int(np.count_nonzero(in_room_frontiers))
        if total_unseen_walls < 5 and total_room_frontiers < MIN_FRONTIER_CELLS:
            return None  # Room is visually and structurally complete!

        candidates = []

        # 1. Room EDT Center (Vantage Point): Step past doorway occlusion and spin
        room_edt = np.where(room_mask, self.clear, 0.0)
        cr, cc = np.unravel_index(np.argmax(room_edt), room_edt.shape)
        self.viz_room_center = self.rc_to_world(int(cr), int(cc))

        # 2. In-place 6-stop yaw sweep at current pose (if safe clearance)
        if self.clear[r0, c0] >= CLEAR_M:
            for k in range(1, 7):
                cyaw = wrap_angle(yaw0 + k * self.sweep_dir * (math.pi / 3.0))
                candidates.append((r0, c0, cyaw))

        # 3. 6-stop yaw sweep at Room EDT Center (steps into center of room first)
        if room_edt[cr, cc] >= CLEAR_M and np.isfinite(bfs_dist[cr, cc]):
            for k in range(6):
                cyaw = wrap_angle(yaw0 + k * self.sweep_dir * (math.pi / 3.0))
                candidates.append((int(cr), int(cc), cyaw))

        # 4. Standoff viewpoints facing clusters of unseen walls or room pockets
        target_mask = (unseen_walls | in_room_frontiers).astype(np.uint8)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(target_mask, connectivity=8)

        H, W = self.g.shape
        for lbl in range(1, num_labels):
            if stats[lbl, cv2.CC_STAT_AREA] < 3:
                continue
            tc, tr = int(round(centroids[lbl][0])), int(round(centroids[lbl][1]))
            for standoff_m in (STANDOFF_OPT_M, STANDOFF_TIGHT_M):
                rad_cells = int(standoff_m / self.res)
                for ang in np.linspace(-math.pi, math.pi, 8, endpoint=False):
                    vr = int(round(tr + rad_cells * math.sin(ang)))
                    vc = int(round(tc + rad_cells * math.cos(ang)))
                    if 0 <= vr < H and 0 <= vc < W:
                        if room_mask[vr, vc] and self.clear[vr, vc] >= CLEAR_M and np.isfinite(bfs_dist[vr, vc]):
                            tx, ty = self.rc_to_world(tr, tc)
                            vx, vy = self.rc_to_world(vr, vc)
                            # Require clear sightline from candidate viewpoint to the target cluster
                            if self.line_of_sight_free(vx, vy, tx, ty, min_clear=0.05):
                                look_yaw = math.atan2(ty - vy, tx - vx)
                                candidates.append((vr, vc, look_yaw))

        best_score = -1.0
        best_view = None
        scored_viz = []

        for vr, vc, vyaw in candidates:
            gain = self.score_frustum_gain(vr, vc, vyaw, unseen_walls, in_room_frontiers)
            if gain < 4.0:
                continue

            d_m = float(bfs_dist[vr, vc])
            if not np.isfinite(d_m):
                continue
            dyaw = wrap_angle(vyaw - yaw0)

            # Avoid micro-adjustments that don't meaningfully move or rotate
            if d_m < 0.20 and abs(dyaw) < 0.30:
                continue

            # Divide gain by total execution time so in-place rotation naturally beats driving
            t_trans = d_m / V_MAX
            t_rot = abs(dyaw) / W_MAX
            total_time = max(0.30, 1.35 * t_trans + t_rot)

            # Monotonic sweep bonus: keep spinning in the same direction when rotating in place
            monotonic_bonus = 1.30 if (d_m < 0.25 and (dyaw * self.sweep_dir) > 0) else 1.0
            score = (gain / total_time) * monotonic_bonus

            vx, vy = self.rc_to_world(vr, vc)
            scored_viz.append((vx, vy, vyaw, score))

            if score > best_score:
                best_score = score
                best_view = (vx, vy, vyaw, (vr, vc))

        self.viz_candidates = scored_viz[:15]
        return best_view

    # =========================================================================
    # 4. GLOBAL FRONTIER & PORTAL SELECTION (OUTER LOOP)
    # =========================================================================
    def select_global_frontier(self, p, bfs_dist):
        """
        Once the current room is 100% cleared, finds all reachable structural frontiers
        across the map and scores them using Geodesic BFS Distance + Line-of-Sight + Heading.
        """
        x0, y0, yaw0 = p
        H, W = self.g.shape
        kernel = np.ones((3, 3), dtype=np.uint8)
        unk_adj = cv2.dilate((self.g < 0).astype(np.uint8), kernel, iterations=1).astype(bool)

        frontier_mask = (
            (self.g >= 0)
            & (self.g < 50)
            & unk_adj
            & (~self.blacklist)
        ).astype(np.uint8)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier_mask, connectivity=8)

        best_cost = float("inf")
        best_goal = None
        frontier_viz = []

        for lbl in range(1, num_labels):
            size = int(stats[lbl, cv2.CC_STAT_AREA])
            if size < MIN_FRONTIER_CELLS:
                continue

            pts_r, pts_c = np.nonzero(labels == lbl)
            cr, cc = float(np.mean(pts_r)), float(np.mean(pts_c))

            # Snap cluster centroid to nearest safe, reachable cell with clearance >= CLEAR_M
            idx_order = np.argsort((pts_r - cr) ** 2 + (pts_c - cc) ** 2)
            goal_rc = None

            for idx in idx_order[:15]:
                r_cand, c_cand = int(pts_r[idx]), int(pts_c[idx])
                r_lo, r_hi = max(0, r_cand - 5), min(H, r_cand + 6)
                c_lo, c_hi = max(0, c_cand - 5), min(W, c_cand + 6)
                sub_cl = self.clear[r_lo:r_hi, c_lo:c_hi]
                sub_d = bfs_dist[r_lo:r_hi, c_lo:c_hi]
                valid = (sub_cl >= CLEAR_M) & np.isfinite(sub_d)
                if bool(valid.any()):
                    vr, vc = np.nonzero(valid)
                    # Pick safe point with highest clearance near the frontier
                    best_local = int(np.argmax(sub_cl[vr, vc]))
                    goal_rc = (r_lo + int(vr[best_local]), c_lo + int(vc[best_local]))
                    break

            if goal_rc is None:
                continue

            gr, gc = goal_rc
            d_geo = float(bfs_dist[gr, gc])
            if not np.isfinite(d_geo) or d_geo < 0.25:
                continue

            gx, gy = self.rc_to_world(gr, gc)
            frontier_viz.append((gx, gy))

            # Check if frontier has direct line-of-sight (same corridor/adjacent portal)
            has_los = self.line_of_sight_free(x0, y0, gx, gy, min_clear=0.10)
            los_penalty = 0.0 if has_los else 2.5

            # Heading continuity bonus + small-pocket cleanup bonus
            heading_angle = math.atan2(gy - y0, gx - x0)
            heading_Align = math.cos(wrap_angle(heading_angle - yaw0))
            pocket_bonus = 1.2 if (size < 25 and d_geo < 3.5) else 0.0

            cost = d_geo + los_penalty - (0.65 * heading_Align) - pocket_bonus

            if cost < best_cost:
                best_cost = cost
                # Orient robot toward the unknown centroid upon arrival
                fx, fy = self.rc_to_world(int(round(cr)), int(round(cc)))
                gyaw = math.atan2(fy - gy, fx - gx) if math.hypot(fx - gx, fy - gy) > 0.2 else heading_angle
                best_goal = (gx, gy, gyaw, (gr, gc))

        self.viz_frontiers = frontier_viz
        return best_goal

    # =========================================================================
    # 5. MAIN PLANNER TICK & NAV2 ACTION DISPATCH
    # =========================================================================
    def tick(self):
        if self.g is None or self.clear is None or self.vis is None or self.busy:
            return
        if not self.nav.server_is_ready():
            return
        p = self.pose()
        if p is None:
            return

        if self.home is None:
            self.home = p
            self.get_logger().info(f"Home pose recorded at ({p[0]:.2f}, {p[1]:.2f})")

        r0, c0 = self.world_to_rc(p[0], p[1])
        H, W = self.g.shape
        if not (0 <= r0 < H and 0 <= c0 < W):
            return

        # 1. Segment active room and compute wall-respecting BFS distances
        room_mask, _ = self.segment_current_room(r0, c0)
        bfs_dist = self.compute_bfs_distance_map(r0, c0)

        # 2. Inner Loop: Clear current room via Next-Best-View (rotations + corner hops)
        nbv = self.select_room_nbv(p, r0, c0, room_mask, bfs_dist)
        if nbv is not None:
            gx, gy, gyaw, target_rc = nbv
            dyaw = wrap_angle(gyaw - p[2])
            if math.hypot(gx - p[0], gy - p[1]) < 0.25 and abs(dyaw) > 0.15:
                self.sweep_dir = 1.0 if dyaw >= 0 else -1.0
                self.get_logger().info(f"[Room NBV Spin] Yaw -> {math.degrees(gyaw):.0f} deg")
            else:
                self.get_logger().info(f"[Room NBV Vantage] -> ({gx:.2f}, {gy:.2f}), Yaw {math.degrees(gyaw):.0f} deg")
            self.send((gx, gy), gyaw, "room_nbv", target_rc)
            return

        # 3. Outer Loop: Current room is 100% cleared; unlock portals & go to best frontier
        nxt_frontier = self.select_global_frontier(p, bfs_dist)
        if nxt_frontier is not None:
            gx, gy, gyaw, target_rc = nxt_frontier
            self.get_logger().info(f"[Room Cleared -> Next Portal/Frontier] ({gx:.2f}, {gy:.2f})")
            self.send((gx, gy), gyaw, "frontier", target_rc)
            return

        # 4. No remaining frontiers or unseen walls -> Return Home
        dist_home = math.hypot(p[0] - self.home[0], p[1] - self.home[1])
        if dist_home > 0.45:
            self.get_logger().info("Exploration 100% complete! Returning to Home pose.")
            self.send((self.home[0], self.home[1]), self.home[2], "return_home", (r0, c0))
        else:
            self.get_logger().info("Maze & Room Visual Inspection Complete! Back at start. DONE.")

    def send(self, xy, yaw, mode, target_rc):
        self.busy = True
        self.cur_goal = (xy, yaw, mode, target_rc)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.z = float(math.sin(yaw / 2.0))
        goal.pose.pose.orientation.w = float(math.cos(yaw / 2.0))
        self.nav.send_goal_async(goal).add_done_callback(self.on_accept)

    def on_accept(self, fut):
        h = fut.result()
        if h is None or not h.accepted:
            return self.finish(False)
        h.get_result_async().add_done_callback(
            lambda f: self.finish(f.result().status == GoalStatus.STATUS_SUCCEEDED)
        )

    def finish(self, ok):
        xy, yaw, mode, target_rc = self.cur_goal
        p = self.pose() or (xy[0], xy[1], yaw)

        # Force an immediate frustum update once the robot settles at the goal yaw
        self.omega_z = 0.0
        self.update_visual_coverage()
        self.visited_history.append((float(p[0]), float(p[1])))

        # Track repeated visits to the same ~0.25m cell; blacklist if stuck or unreachable
        tr, tc = target_rc
        key = (tr // 5, tc // 5)
        self.visit_counts[key] = self.visit_counts.get(key, 0) + (1 if ok else 2)

        if not ok or self.visit_counts[key] >= 3:
            if self.blacklist is not None:
                H, W = self.blacklist.shape
                r_lo, r_hi = max(0, tr - 8), min(H, tr + 9)
                c_lo, c_hi = max(0, tc - 8), min(W, tc + 9)
                self.blacklist[r_lo:r_hi, c_lo:c_hi] = True
                self.get_logger().warn(f"Marked stubborn region around ({xy[0]:.2f}, {xy[1]:.2f}) as cleared/ignored.")

        self.busy = False

    # =========================================================================
    # 6. RVIZ VISUALIZATION (explorer/coverage & explorer/stack)
    # =========================================================================
    def viz(self):
        if self.g is None or self.vis is None or self.map_header is None:
            return

        # 1. Publish visual coverage OccupancyGrid (0=Unseen, 50=Seen Floor, 100=Inspected Wall)
        cov_msg = OccupancyGrid()
        cov_msg.header = self.map_header
        cov_msg.header.stamp = self.get_clock().now().to_msg()
        cov_msg.info.resolution = self.res
        cov_msg.info.width = self.g.shape[1]
        cov_msg.info.height = self.g.shape[0]
        cov_msg.info.origin.position.x = self.ox
        cov_msg.info.origin.position.y = self.oy
        cov_msg.info.origin.orientation.w = 1.0

        cov_data = np.full(self.g.shape, -1, dtype=np.int8)
        cov_data[self.vis == 1] = 0    # Seen floor -> Free (white/light gray in RViz)
        cov_data[self.vis == 2] = 100  # Inspected wall -> Occupied (black/emerald in RViz)
        cov_msg.data = cov_data.ravel().tolist()
        self.cov_pub.publish(cov_msg)

        # 2. Publish MarkerArray for visited vantage points, room center, and frontiers
        arr = MarkerArray()
        d = Marker()
        d.action = Marker.DELETEALL
        arr.markers.append(d)
        stamp = self.get_clock().now().to_msg()
        mid = 0

        # Visited trajectory nodes (Green spheres)
        for x, y in self.visited_history[-80:]:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "visited"
            m.id = mid
            mid += 1
            m.action = Marker.ADD
            m.type = Marker.SPHERE
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.22
            m.color.a, m.color.r, m.color.g, m.color.b = 0.85, 0.0, 0.9, 0.3
            arr.markers.append(m)

        # Candidate NBV viewpoints (Purple arrows)
        for vx, vy, vyaw, _ in self.viz_candidates:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "nbv_candidates"
            m.id = mid
            mid += 1
            m.action = Marker.ADD
            m.type = Marker.ARROW
            m.pose.position.x = float(vx)
            m.pose.position.y = float(vy)
            m.pose.orientation.z = float(math.sin(vyaw / 2.0))
            m.pose.orientation.w = float(math.cos(vyaw / 2.0))
            m.scale.x, m.scale.y, m.scale.z = 0.35, 0.06, 0.06
            m.color.a, m.color.r, m.color.g, m.color.b = 0.75, 0.7, 0.2, 1.0
            arr.markers.append(m)

        # Global Frontiers (Cyan cubes)
        for fx, fy in self.viz_frontiers:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "frontiers"
            m.id = mid
            mid += 1
            m.action = Marker.ADD
            m.type = Marker.CUBE
            m.pose.position.x = float(fx)
            m.pose.position.y = float(fy)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.26
            m.color.a, m.color.r, m.color.g, m.color.b = 0.9, 0.0, 0.85, 1.0
            arr.markers.append(m)

        self.mark_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()