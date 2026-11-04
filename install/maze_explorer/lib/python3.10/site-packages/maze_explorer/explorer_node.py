#!/usr/bin/env python3
"""
frontier_explorer_node.py  (v5: DFS bias, door pull, dual camera, CPU cuts, scoreboard)
=====================================================================
Depth-first exploration for a differential-drive robot with a 360-degree LiDAR
(SLAM + Nav2) and a FIXED forward camera (human detection).

Key idea
--------
"Explored" must mean "the CAMERA has looked here", not "the LiDAR mapped it".
The node therefore keeps its own *camera-coverage grid* (cells the camera wedge
has actually seen, with wall occlusion) and generates two kinds of targets:

  1. FRONTIER targets  - free cells touching unknown space (map still growing).
                         Goal = the frontier cell, yaw = frontier normal.
  2. UNSEEN targets    - LiDAR-mapped free space the camera has NOT yet seen
                         (e.g. a room the LiDAR mapped through the doorway).
                         Goal = viewpoint `inspect_depth_m` INSIDE the region,
                         along the entrance->centroid direction, yaw = that
                         direction. Because the room is already mapped free,
                         Nav2 plans into it natively (no allow_unknown needed).

After reaching an UNSEEN viewpoint the robot SWEEPS (Nav2 `spin`): it rotates
toward the largest remaining unseen mass until the corners are covered.

Reachability: before dispatching, candidates are checked with Nav2's ComputePathToPose;
unreachable ones are skipped and the real PATH length (not straight-line) is used for scoring.

No commit lock (v5.1): while driving, the robot re-scores every tick and may switch to a
better-scoring target at any time. The only things that end a goal are a better target,
the goal ceasing to be valid (after `invalid_grace_s`), a Nav2 abort/reject, or the watchdog.

States: INITIALIZING, FINDING_FRONTIER, NAVIGATING_TO_GOAL, GOAL_REACHED,
        EXPLORATION_COMPLETE

v5.1 changes (on top of v5)
---------------------------
- Turn cost is now ONE term with a dead zone: exactly 0 for any target within
  +/- `turn_deadzone_deg` (135) of the robot's heading, left or right. Only targets
  further behind than that pay, ramping quadratically up to `turn_penalty_m` at 180.
  `w_angle` and `reverse_penalty_m` are gone (they penalised 90-degree door turns).
- No commit lock: `commit_radius_*`, `preempt_confirm_s`, `min_goal_interval_s`,
  `preempt_margin_m/ratio/room_margin_m` are gone. While driving, the robot re-scores
  every tick and switches to a better target immediately, if its SCORE (distance, turn,
  door bonus) beats the current goal's by `switch_margin_m` (anti-jitter tie-break).

v5 changes (on top of v4)
-------------------------
DFS / anti-backtracking
- `w_angle` 0.5 -> 3.0 (moderate: 90 deg ~ 0.5 m-equivalent, 180 deg ~ 0.9 m).
- `reverse_penalty_m` 1.0 -> 2.0 and now QUADRATIC in how directly opposite the
  candidate is (cos^2), so a real U-turn is expensive but a mild turn-back is not.
Doors
- `unseen_score_penalty_m` 1.2 -> 0.0 (it was repelling the robot from rooms).
- NEW `passing_door_bonus_m` / `passing_door_radius_m`: UNSEEN targets get a
  distance discount that ramps up smoothly as the robot gets close to them, so a
  doorway beats the turn penalty when you are right next to it (no on/off flicker).
CPU
- `eval_rate_hz` 4 -> 2, `coverage_rate_hz` 10 -> 5, `complete_confirmations`
  20 -> 10 (keeps the ~5 s "nothing left" window at the lower rate).
- `is_room_spot` (180 rays x 6 m) is cached per goal and cleared on every new map,
  so the preemption check no longer re-casts rays every tick.
- Unseen targets are cached and recomputed at most every `unseen_refresh_s`
  (or immediately on a new map).
Stability
- `commit_radius_frontier_m` 1.0 -> 0.8, `preempt_confirm_s` 1.5 -> 2.0,
  `frontier_standoff_m` 0.5 -> 0.0 (stop on the frontier; raise it if goals start
  failing to plan because the costmap lags /map).
New features
- `use_dual_cameras`: also paints a coverage wedge 180 deg behind the robot.
  ONLY correct if the robot really has a rear camera. Set False otherwise.
- Scoreboard log: top-5 targets every `scoreboard_period_s` with true distance,
  turn angle and every penalty / bonus that produced the final score.

v4 changes
----------
- `unseen_score_penalty_m`: a fixed cost added to UNSEEN candidates during
  scoring, so a very-close unseen patch no longer automatically outranks a
  farther-but-real frontier (was causing to-and-fro at the start of a run).
- Non-room visited spots now get the SAME exclusion radius as room spots
  (was half), so an almost-identical patch can't immediately reappear as a
  "new" target right next to one just finished.
- `reverse_penalty_m`: candidates that require reversing the direction the
  robot just committed to traveling now get a proportional score penalty,
  to stop marginal-score 180-degree flip-flopping.
- `compute_unseen_targets` no longer clamps the inspect viewpoint depth to
  the doorway->centroid distance; `project_along`'s own wall/clearance walk
  is now the only thing limiting how far into a room the robot goes.
- `visible_cells` / `is_room_spot` / `project_along` shared ray-casting setup
  (angles -> world samples -> grid cells -> in-bounds mask) pulled into one
  `_ray_grid_coords` helper. No behavior change, just less duplicated math.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

INITIALIZING = "INITIALIZING"
FINDING_FRONTIER = "FINDING_FRONTIER"
NAVIGATING_TO_GOAL = "NAVIGATING_TO_GOAL"
GOAL_REACHED = "GOAL_REACHED"
EXPLORATION_COMPLETE = "EXPLORATION_COMPLETE"

KIND_FRONTIER = "frontier"
KIND_UNSEEN = "unseen"
KIND_SWEEP = "sweep"


@dataclass
class Target:
    kind: str                        # KIND_FRONTIER | KIND_UNSEEN
    label: int                       # label id inside that kind's label image
    centroid: Tuple[float, float]
    goal: Tuple[float, float]        # position sent to Nav2 (map frame, m)
    yaw: float                       # goal heading (rad); NaN -> fall back to bearing
    size_cells: int
    extent: float                    # frontier length (m) or unseen area (m^2)
    dist: float = 0.0                # path length once Nav2 has confirmed it, else straight-line
    score: float = 0.0
    checked: bool = False            # True = Nav2's planner confirmed it is reachable
    raw_goal: Optional[Tuple[float, float]] = None   # goal before it was nudged off the walls
    parts: Dict[str, float] = field(default_factory=dict)   # score breakdown (for the scoreboard)


@dataclass
class ActiveGoal:
    kind: str
    xy: Tuple[float, float]
    sent_t: float
    committed: bool = False
    room: bool = False               # True = arrive facing in + camera sweeps allowed


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("explorer_node")
        P = self.declare_parameter
        # -- frames / interfaces
        P("map_topic", "/map")
        P("map_frame", "map")
        P("robot_base_frame", "base_link")
        P("camera_frame", "")                     # "" -> use robot_base_frame heading
        P("nav_action_name", "navigate_to_pose")
        P("spin_action_name", "spin")
        # -- loop rates
        P("eval_rate_hz", 2.0)                    # >= 2
        P("coverage_rate_hz", 5.0)
        # -- map denoising / frontier extraction
        P("denoise_kernel_size", 3)               # free-space opening (cells)
        P("denoise_unknown_kernel_size", 3)       # unknown-space opening: fills thin stripes/gaps up to this width
        P("occupied_threshold", 65)
        P("min_frontier_length_meters", 0.4)
        P("robot_clearance_m", 0.25)
        P("cluster_merge_cells", 3)
        # -- scoring (distance-dominant DFS)
        P("w_dist", 10.0)
        P("w_size", 0.0)
        # Turn cost: ZERO for any target within +/- turn_deadzone_deg of the robot's heading
        # (left or right). Beyond that it ramps quadratically to turn_penalty_m (metres-equivalent,
        # scaled by w_dist) at a full 180-degree reversal.
        P("turn_deadzone_deg", 135.0)
        P("turn_penalty_m", 3.0)
        # Fixed cost (metres-equivalent) added to UNSEEN candidates. 0 = off.
        P("unseen_score_penalty_m", 0.0)
        # Door pull: UNSEEN targets get up to this many metres knocked off their distance when
        # the robot is right next to them, fading linearly to 0 at passing_door_radius_m.
        # Smooth ramp (no on/off switch), so it can't flip-flop at the edge of the radius.
        P("passing_door_bonus_m", 2.0)
        P("passing_door_radius_m", 3.0)
        # -- camera model
        P("use_dual_cameras", True)               # True = also mark a wedge 180 deg BEHIND as seen (needs a rear camera!)
        P("camera_hfov_deg", 70.0)
        P("camera_range_m", 3.5)                  # reliable human-detection range
        # -- unseen (coverage) targets
        P("min_unseen_area_m2", 0.6)
        P("unseen_open_m", 0.3)                   # kills thin unseen slivers
        P("unseen_refresh_s", 1.0)                # recompute unseen targets at most this often (CPU)
        P("inspect_depth_m", 1.0)                 # viewpoint depth into the region
        P("visited_radius_m", 1.0)
        # -- sweep (in-place rotation after reaching an inspect viewpoint)
        P("max_sweeps", 3)
        P("sweep_yaw_step_deg", 20.0)
        P("min_sweep_area_m2", 0.15)
        # -- switching (no commit lock: the robot may change target at any time)
        # A different target takes over as soon as its score beats the current goal's by this
        # many metres-equivalent. Tie-break only; set to 0 for a pure "always best" robot.
        P("switch_margin_m", 0.5)
        P("invalid_grace_s", 1.5)                 # goal must stay "gone" this long before we drop it
        P("goal_validity_radius_m", 1.0)
        P("goal_timeout_s", 90.0)
        # -- debug scoreboard
        P("scoreboard_enabled", True)
        P("scoreboard_period_s", 2.0)
        # -- blacklist / termination
        P("blacklist_duration_s", 45.0)
        P("blacklist_radius_m", 0.6)
        P("complete_confirmations", 10)          # ~5 s of "nothing left" at 2 Hz
        P("unreachable_retry_rounds", 2)         # re-try "unreachable" targets this many times before finishing
        # -- planner-wide failure guard: many DIFFERENT targets failing at once is not the targets' fault
        P("global_fail_targets", 3)
        P("global_fail_window_s", 4.0)
        P("global_fail_hold_s", 12.0)
        P("frontier_standoff_m", 0.0)            # stop this far BEFORE the unknown edge (0 = on it)
        P("room_ray_range_m", 6.0)               # room test: every direction hits a wall within this
        P("room_max_open_rays", 3)               # ...allowing this many of 180 rays to miss (holes in a wall)
        # -- Nav2 costmap awareness: never aim at a spot the robot body cannot actually occupy
        P("costmap_topic", "/global_costmap/costmap")
        P("costmap_goal_max_cost", 50)           # 0..100; >= 99 means inscribed/lethal (no-go)
        P("costmap_snap_max_m", 0.55)            # how far a goal may be moved (stays under wall thickness + inflation)
        P("costmap_goal_clearance_m", 0.45)      # goal must be this far from any wall = robot radius + goal tolerance
        # -- self-rescue: if the robot ends up inside the no-go zone, creep out (planner cannot start there)
        P("rescue_enabled", True)
        P("cmd_vel_topic", "/cmd_vel")
        P("rescue_speed", 0.12)
        P("rescue_target_clearance_m", 0.40)     # costmap mode: aim for a cell this far from lethal
        P("rescue_exit_clearance_m", 0.36)       # costmap mode: stop once this far from lethal
        P("rescue_open_clearance_m", 0.55)       # SLAM-map mode: aim for a spot this far from any wall
        # -- reachability check: ask Nav2's planner before committing to a goal
        P("path_check_enabled", True)
        P("path_check_top_k", 3)              # how many best candidates to check per tick
        P("path_cache_ttl_s", 8.0)            # reuse a "reachable" answer this long
        P("unreachable_ttl_s", 8.0)           # first "unreachable" answers are treated as maybe-temporary
        P("unreachable_max_fails", 3)         # after this many in a row it is really unreachable
        P("unreachable_hard_ttl_s", 120.0)
        P("compute_path_action_name", "compute_path_to_pose")

        self.map_frame = self.p("map_frame")
        self.base_frame = self.p("robot_base_frame")
        self.camera_frame = self.p("camera_frame") or self.base_frame
        rate = max(2.0, float(self.p("eval_rate_hz")))

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.p("map_topic"), self.map_cb, map_qos)
        self.create_subscription(OccupancyGrid, self.p("costmap_topic"), self.costmap_cb, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, self.p("nav_action_name"))
        self.spin_client = ActionClient(self, Spin, self.p("spin_action_name"))
        self.path_client = ActionClient(self, ComputePathToPose, self.p("compute_path_action_name"))

        self.frontier_pub = self.create_publisher(MarkerArray, "/exploration/frontiers", 1)
        self.goal_pub = self.create_publisher(PointStamped, "/exploration/active_goal", 1)
        self.coverage_pub = self.create_publisher(OccupancyGrid, "/exploration/camera_coverage", 1)
        self.cmd_pub = self.create_publisher(Twist, self.p("cmd_vel_topic"), 1)

        # map-derived cache
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_counter = 0
        self.processed_counter = -1
        self.map_meta = None                      # (res, ox, oy, h, w)
        self.free_clean: Optional[np.ndarray] = None
        self.clear_mask: Optional[np.ndarray] = None
        self.frontier_labels: Optional[np.ndarray] = None
        self.frontier_targets: List[Target] = []
        self.unseen_labels: Optional[np.ndarray] = None
        # camera coverage
        self.seen: Optional[np.ndarray] = None
        self.seen_meta = None

        # state
        self.state = INITIALIZING
        self.goal_seq = 0
        self.goal_handle = None
        self.active: Optional[ActiveGoal] = None
        self.last_kind: Optional[str] = None      # kind of the goal that just finished
        self.sweep_count = 0
        self.empty_ticks = 0
        self.blacklist: List[Tuple[float, float, float]] = []
        self.visited: List[Tuple[float, float, float]] = []   # (x, y, radius)
        self.path_info: Dict[Tuple[int, int], Tuple[Optional[float], float]] = {}  # key -> (length|None, time)
        self.path_pending: Set[Tuple[int, int]] = set()
        self.path_fails: Dict[Tuple[int, int], int] = {}
        self.soft_skipped = 0                     # targets Nav2 could not plan to YET
        self.all_targets: List[Target] = []       # every live target, before the reachability filter
        self.last_room = False
        self.occ_mask: Optional[np.ndarray] = None
        self.costmap_arr: Optional[np.ndarray] = None
        self.costmap_info = None
        self.costmap_edt: Optional[np.ndarray] = None   # metres to the nearest LETHAL cell
        self.robot_blocked = False
        self.wall_dist: Optional[np.ndarray] = None     # metres to nearest occupied SLAM cell
        self.occ_near: Optional[np.ndarray] = None
        self.hard_skipped = 0
        self.retry_round = 0
        self.recent_fails: List[Tuple[float, Tuple[int, int]]] = []
        self.global_fail_until = 0.0
        self.global_fail_episodes = 0
        self.check_disabled_until = 0.0
        self.invalid_since: Optional[float] = None
        self.preempt_key = None
        self.preempt_since = 0.0
        self.rescue_mode: Optional[str] = None
        self.rescue_started = 0.0
        self.rescue_cooldown_until = 0.0
        self.blocked_since = 0.0
        self.rescuing = False
        self.last_nav_check = 0.0
        # CPU caches
        self.room_cache: Dict[Tuple[int, int], bool] = {}
        self.room_cache_counter = -1
        self.unseen_cache: List[Target] = []
        self.unseen_cache_t = -1e9
        self.unseen_cache_counter = -1
        # scoreboard
        self.last_scoreboard_t = 0.0
        self.last_use_check = False

        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(1.0 / max(1.0, float(self.p("coverage_rate_hz"))), self.coverage_tick)
        self.create_timer(1.0, self.publish_coverage)
        self.get_logger().info(f"Coverage explorer started ({rate:.1f} Hz eval, "
                               f"dual_cameras={bool(self.p('use_dual_cameras'))}).")

    # ------------------------------------------------------------------ utils
    def p(self, name):
        return self.get_parameter(name).value

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s: str):
        if s != self.state:
            self.get_logger().info(f"State: {self.state} -> {s}")
            self.state = s

    def get_pose(self, frame: str, timeout: float = 0.1):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, frame, Time(),
                                                 timeout=Duration(seconds=timeout))
        except TransformException as ex:
            self.get_logger().warn(f"TF {self.map_frame}->{frame} unavailable: {ex}",
                                   throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        return t.x, t.y, quat_to_yaw(tf.transform.rotation)

    def world_to_cell(self, x, y):
        res, ox, oy, _, _ = self.map_meta
        return int(math.floor((y - oy) / res)), int(math.floor((x - ox) / res))

    def cell_to_world(self, r, c):
        res, ox, oy, _, _ = self.map_meta
        return (ox + (c + 0.5) * res, oy + (r + 0.5) * res)

    def _ray_grid_coords(self, x, y, angles, dists):
        """Shared ray-sampling math used by visible_cells / is_room_spot / project_along:
        cast rays from (x, y) at each angle in `angles`, sample at each distance in
        `dists`, and convert every sample to a grid (row, col). Returns rows and cols
        CLIPPED to stay in-bounds (safe to index with), plus an `inb` mask of which
        samples were actually inside the map -- callers must check `inb` before
        trusting a clipped cell."""
        res, ox, oy, h, w = self.map_meta
        xs = x + np.outer(np.cos(angles), dists)
        ys = y + np.outer(np.sin(angles), dists)
        cols = np.floor((xs - ox) / res).astype(np.int64)
        rows = np.floor((ys - oy) / res).astype(np.int64)
        inb = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        return np.clip(rows, 0, h - 1), np.clip(cols, 0, w - 1), inb

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_counter += 1

    # ------------------------------------------------- Nav2 costmap awareness
    def costmap_cb(self, msg: OccupancyGrid):
        self.costmap_info = msg.info
        arr = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.costmap_arr = arr
        # distance (m) from every cell to the nearest LETHAL cell (cost 100)
        self.costmap_edt = cv2.distanceTransform((arr != 100).astype(np.uint8), cv2.DIST_L2, 3) \
            * msg.info.resolution

    def costmap_cell(self, x, y):
        info = self.costmap_info
        res = info.resolution
        return (int(math.floor((y - info.origin.position.y) / res)),
                int(math.floor((x - info.origin.position.x) / res)))

    def snap_to_costmap(self, xy):
        """Nav2's costmap knows what the SLAM map doesn't (live LiDAR hits, robot size, inflation).
        A goal must be somewhere the robot can STOP: robot radius + goal tolerance away from walls.
        Otherwise it is moved to the nearest cell that is (relaxing the margin if the corridor is tight)."""
        cm, edt = self.costmap_arr, self.costmap_edt
        if cm is None:
            return xy
        h, w = cm.shape
        r, c = self.costmap_cell(*xy)
        thr = int(self.p("costmap_goal_max_cost"))
        full = float(self.p("costmap_goal_clearance_m"))
        res = self.costmap_info.resolution
        inside = 0 <= r < h and 0 <= c < w
        if inside and 0 <= cm[r, c] <= thr and edt[r, c] >= full:
            return xy                                     # already a good spot
        R = max(1, int(float(self.p("costmap_snap_max_m")) / res))
        r0, r1, c0, c1 = max(0, r - R), min(h, r + R + 1), max(0, c - R), min(w, c + R + 1)
        if r0 >= r1 or c0 >= c1:
            return xy
        win, ewin = cm[r0:r1, c0:c1], edt[r0:r1, c0:c1]
        for need in (full, 0.8 * full, 0.0):              # tight corridor? accept a smaller margin
            rr, cc = np.nonzero((win >= 0) & (win <= thr) & (ewin >= need))
            if rr.size:
                rr, cc = rr + r0, cc + c0
                j = np.argmin((rr - r) ** 2 + (cc - c) ** 2)
                o = self.costmap_info.origin.position
                return (o.x + (cc[j] + 0.5) * res, o.y + (rr[j] + 0.5) * res)
        return xy                                         # nothing nearby; let the planner check judge

    def update_robot_blocked(self, pose):
        """Is the robot itself inside the no-go zone? Then NavFn cannot start ANY path from here."""
        cm = self.costmap_arr
        blocked = False
        if cm is not None:
            r, c = self.costmap_cell(pose[0], pose[1])
            blocked = bool(0 <= r < cm.shape[0] and 0 <= c < cm.shape[1] and cm[r, c] >= 99)
        if blocked and not self.robot_blocked:
            self.blocked_since = self.now_s()
        if self.robot_blocked and not blocked:
            # freed: every "unreachable" verdict was probably caused by this, so forget them
            self.path_info = {k: v for k, v in self.path_info.items() if v[0] is not None}
            self.path_fails.clear()
        self.robot_blocked = blocked
        if blocked:
            self.get_logger().warn(
                "Robot is inside Nav2's no-go zone next to a wall, so no path can start from here. "
                "Planning checks are paused while it backs out.", throttle_duration_sec=5.0)

    # ---- self-rescue: creep out toward the nearest spot with plenty of room around it ----
    def rescue_target(self, pose, mode):
        """mode 'costmap': Nav2 says we are inside the no-go zone -> nearest clearly free costmap cell.
        mode 'slam'    : planner fails everywhere though the costmap looks fine -> nearest spot with
                         lots of wall clearance in the SLAM map."""
        rx, ry, _ = pose
        if mode == "costmap":
            cm, edt = self.costmap_arr, self.costmap_edt
            if cm is None:
                return None
            valid = (cm >= 0) & (cm <= 50) & (edt >= float(self.p("rescue_target_clearance_m")))
            res, o = self.costmap_info.resolution, self.costmap_info.origin.position
            ox, oy = o.x, o.y
        else:
            if self.free_clean is None or self.wall_dist is None:
                return None
            valid = (self.free_clean == 1) & (self.wall_dist >= float(self.p("rescue_open_clearance_m")))
            res, ox, oy, _, _ = self.map_meta
        h, w = valid.shape
        r, c = int(math.floor((ry - oy) / res)), int(math.floor((rx - ox) / res))
        R = int(1.5 / res)
        r0, r1, c0, c1 = max(0, r - R), min(h, r + R + 1), max(0, c - R), min(w, c + R + 1)
        if r0 >= r1 or c0 >= c1:
            return None
        rr, cc = np.nonzero(valid[r0:r1, c0:c1])
        if rr.size == 0:
            return None
        rr, cc = rr + r0, cc + c0
        j = np.argmin((rr - r) ** 2 + (cc - c) ** 2)
        return ox + (cc[j] + 0.5) * res, oy + (rr[j] + 0.5) * res

    def rescue_command(self, pose, mode="costmap"):
        tgt = self.rescue_target(pose, mode)
        if tgt is None:
            return None
        rx, ry, ryaw = pose
        err = wrap_angle(math.atan2(tgt[1] - ry, tgt[0] - rx) - ryaw)
        if abs(err) > 0.5:                                # face the way out first (slow tank turn)
            return 0.0, float(np.clip(1.0 * err, -0.6, 0.6))
        return float(self.p("rescue_speed")), float(np.clip(1.5 * err, -0.5, 0.5))

    def rescue_done(self, pose, mode) -> bool:
        if mode == "costmap":
            r, c = self.costmap_cell(pose[0], pose[1])
            if not (0 <= r < self.costmap_arr.shape[0] and 0 <= c < self.costmap_arr.shape[1]):
                return True
            return bool(self.costmap_arr[r, c] < 99
                        and self.costmap_edt[r, c] >= float(self.p("rescue_exit_clearance_m")))
        res, ox, oy, h, w = self.map_meta
        r, c = int(math.floor((pose[1] - oy) / res)), int(math.floor((pose[0] - ox) / res))
        if not (0 <= r < h and 0 <= c < w):
            return True
        return bool(self.wall_dist[r, c] >= float(self.p("rescue_open_clearance_m")) - 0.05)

    def stop_robot(self):
        if self.rescuing:
            self.cmd_pub.publish(Twist())
            self.rescuing = False
            self.rescue_mode = None

    def rescue_tick(self):
        if not bool(self.p("rescue_enabled")):
            return self.stop_robot()
        now = self.now_s()
        idle = self.active is None and self.state in (FINDING_FRONTIER, GOAL_REACHED)
        pose = self.get_pose(self.base_frame, timeout=0.05) if idle else None
        if pose is None:
            return self.stop_robot()
        mode = self.rescue_mode if self.rescuing else None
        if mode is None:
            if now < self.rescue_cooldown_until or self.free_clean is None:
                return
            if self.robot_blocked and now - self.blocked_since >= 1.0:
                mode = "costmap"
            elif now < self.global_fail_until:
                mode = "slam"
            else:
                return
        if self.rescue_done(pose, mode):
            return self.stop_robot()
        if not self.rescuing:
            self.rescue_started = now
            self.get_logger().info(f"Rescue ({mode}): moving away from the wall.")
        if now - self.rescue_started > 15.0:
            self.get_logger().warn("Rescue gave up after 15 s.")
            self.rescue_cooldown_until = now + 20.0
            return self.stop_robot()
        cmd = self.rescue_command(pose, mode)
        if cmd is None:
            self.rescue_cooldown_until = now + 20.0
            return self.stop_robot()
        msg = Twist()
        msg.linear.x, msg.angular.z = cmd
        self.rescuing, self.rescue_mode = True, mode
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------ map pipeline
    def refresh_map_cache(self):
        if self.map_msg is None or self.map_counter == self.processed_counter:
            return
        try:
            self.process_map(self.map_msg)
            self.processed_counter = self.map_counter
        except Exception as ex:
            self.get_logger().error(f"Map processing failed: {ex}")

    def process_map(self, msg: OccupancyGrid):
        h, w = msg.info.height, msg.info.width
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y  # origin yaw assumed 0
        grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)

        k = max(1, int(self.p("denoise_kernel_size")))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        ku = max(1, int(self.p("denoise_unknown_kernel_size")))
        kernel_u = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ku, ku))
        occ_thr = int(self.p("occupied_threshold"))
        unknown = (grid < 0).astype(np.uint8)
        occ = (grid >= occ_thr).astype(np.uint8)
        free = ((grid >= 0) & (grid < occ_thr)).astype(np.uint8)

        # Denoise: open FREE (drop free specks in the unknown) and open UNKNOWN
        # (fill unknown specks inside explored space). Walls are not opened.
        free_clean = cv2.morphologyEx(free, cv2.MORPH_OPEN, kernel)
        unknown_clean = cv2.morphologyEx(unknown, cv2.MORPH_OPEN, kernel_u)
        # Thin unknown stripes (gaps between sparse long-range rays, holes in dotted walls) are
        # filled by whichever is more common nearby: wall -> becomes wall, open space -> becomes free.
        specks = (unknown == 1) & (unknown_clean == 0) & (occ == 0)
        if specks.any():
            occ_density = cv2.blur(occ.astype(np.float32), (7, 7))
            free_density = cv2.blur(free_clean.astype(np.float32), (7, 7))
            to_wall = specks & (occ_density > free_density)
            free_clean[specks & ~to_wall] = 1
            occ[to_wall] = 1

        # Wall clearance (metres to nearest occupied cell)
        dist_wall = cv2.distanceTransform((occ == 0).astype(np.uint8), cv2.DIST_L2, 3) * res
        self.clear_mask = dist_wall >= float(self.p("robot_clearance_m"))
        self.wall_dist = dist_wall
        self.occ_near = cv2.dilate(occ, np.ones((3, 3), np.uint8)).astype(bool)   # wall within 1 cell
        self.free_clean = free_clean
        self.occ_mask = occ.astype(bool)
        self.map_meta = (res, ox, oy, h, w)
        self.sync_seen(h, w, res, ox, oy)

        # ---- frontiers ----
        self.frontier_labels = np.zeros((h, w), np.int32)
        self.frontier_targets = []
        unknown_dil = cv2.dilate(unknown_clean, np.ones((3, 3), np.uint8))
        frontier = ((free_clean == 1) & (unknown_dil == 1)).astype(np.uint8)
        if not frontier.any():
            return

        # direction from each frontier cell toward its adjacent unknown cells
        unk_f = unknown_clean.astype(np.float32)
        kx = np.array([[-1, 0, 1]] * 3, np.float32)
        gx = cv2.filter2D(unk_f, -1, kx, borderType=cv2.BORDER_CONSTANT)
        gy = cv2.filter2D(unk_f, -1, kx.T.copy(), borderType=cv2.BORDER_CONSTANT)

        m = max(1, int(self.p("cluster_merge_cells")))
        n, comp, stats, _ = cv2.connectedComponentsWithStats(
            cv2.dilate(frontier, np.ones((m, m), np.uint8)), connectivity=8)
        labels = comp.astype(np.int32) * frontier
        sizes = np.bincount(labels.ravel(), minlength=n)
        min_cells = max(1, int(math.ceil(float(self.p("min_frontier_length_meters")) / res)))
        valid = sizes >= min_cells
        valid[0] = False
        labels = np.where(valid, np.arange(n), 0).astype(np.int32)[labels]

        for lab in np.nonzero(valid)[0]:
            x0, y0 = stats[lab, cv2.CC_STAT_LEFT], stats[lab, cv2.CC_STAT_TOP]
            bw, bh = stats[lab, cv2.CC_STAT_WIDTH], stats[lab, cv2.CC_STAT_HEIGHT]
            rr, cc = np.nonzero(labels[y0:y0 + bh, x0:x0 + bw] == lab)
            if rr.size == 0:
                continue
            rr, cc = rr + y0, cc + x0
            ok = self.clear_mask[rr, cc]
            if not ok.any():
                labels[labels == lab] = 0
                continue
            cen = self.cell_to_world(rr.mean(), cc.mean())
            rro, cco = rr[ok], cc[ok]
            j = np.argmin((rro - rr.mean()) ** 2 + (cco - cc.mean()) ** 2)
            anchor = self.cell_to_world(rro[j], cco[j])
            nx, ny = float(gx[rr, cc].sum()), float(gy[rr, cc].sum())
            if math.hypot(nx, ny) > 1e-3:
                yaw = math.atan2(ny, nx)
            else:
                yaw = float("nan")
            # stand back from the unknown edge (frontier_standoff_m; 0 = stop on it). The costmap
            # lags /map by a moment, so a goal exactly on the edge can be "not planable yet" --
            # raise the standoff if you see that. The 10 m LiDAR sees past it anyway.
            goal = anchor
            if not math.isnan(yaw):
                goal = self.project_along(anchor, yaw + math.pi, float(self.p("frontier_standoff_m")))
            self.frontier_targets.append(Target(
                KIND_FRONTIER, int(lab), cen, goal, yaw, int(rr.size), float(rr.size * res)))
        self.frontier_labels = labels

    # ------------------------------------------------------- camera coverage
    def sync_seen(self, h, w, res, ox, oy):
        """(Re)allocate the seen grid when /map grows, preserving old coverage."""
        meta = (h, w, res, ox, oy)
        if self.seen is not None and self.seen_meta == meta:
            return
        new = np.zeros((h, w), bool)
        if self.seen is not None and self.seen_meta[2] == res:
            oh, ow, _, oox, ooy = self.seen_meta
            dc, dr = int(round((oox - ox) / res)), int(round((ooy - oy) / res))
            r0, c0 = max(dr, 0), max(dc, 0)
            r1, c1 = min(dr + oh, h), min(dc + ow, w)
            if r1 > r0 and c1 > c0:
                new[r0:r1, c0:c1] = self.seen[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
        self.seen, self.seen_meta = new, meta

    def visible_cells(self, x, y, yaw):
        """Free cells inside the camera wedge, with wall/unknown occlusion (vectorised ray-cast)."""
        res = self.map_meta[0]
        hfov = math.radians(float(self.p("camera_hfov_deg")))
        rng = float(self.p("camera_range_m"))
        n_rays = int(math.ceil(hfov * rng / res)) + 1
        n_steps = max(1, int(rng / res))
        angs = yaw + np.linspace(-hfov / 2, hfov / 2, n_rays)
        ds = np.arange(1, n_steps + 1) * res
        rows, cols, inb = self._ray_grid_coords(x, y, angs, ds)
        blocked = ~inb | (self.free_clean[rows, cols] == 0)
        visible = ~np.maximum.accumulate(blocked, axis=1)   # nothing beyond the first blocker
        return rows[visible], cols[visible]

    def camera_cells(self, x, y, yaw):
        """Cells covered by the camera(s) at this pose: the forward wedge, plus (when
        use_dual_cameras is on) a second wedge 180 degrees behind it."""
        r, c = self.visible_cells(x, y, yaw)
        if not bool(self.p("use_dual_cameras")):
            return r, c
        r2, c2 = self.visible_cells(x, y, yaw + math.pi)
        return np.concatenate([r, r2]), np.concatenate([c, c2])

    def update_seen(self):
        if self.free_clean is None or self.seen is None:
            return
        cam = self.get_pose(self.camera_frame, timeout=0.05)
        if cam is None:
            return
        r, c = self.camera_cells(*cam)
        self.seen[r, c] = True

    def coverage_tick(self):
        self.refresh_map_cache()
        self.update_seen()
        self.rescue_tick()

    def is_room_spot(self, x, y) -> bool:
        """True when this spot is inside an enclosed room: looking in EVERY direction, the view
        ends at a wall within room_ray_range_m. A corridor fails (it runs on past that range or
        into unknown space), so corridors never get camera-facing turns or sweeps."""
        if self.free_clean is None:
            return False
        rng = float(self.p("room_ray_range_m"))
        res = self.map_meta[0]
        steps = max(1, int(rng / res))
        angs = np.linspace(0.0, 2 * math.pi, 180, endpoint=False)      # 2 degree spacing
        ds = np.arange(1, steps + 1) * res
        rows, cols, inb = self._ray_grid_coords(x, y, angs, ds)
        nonfree = ~inb | (self.free_clean[rows, cols] == 0)
        if not nonfree.any(axis=1).all():          # some direction never ends -> open space
            return False
        first = np.argmax(nonfree, axis=1)
        idx = np.arange(len(angs))
        hit_wall = inb[idx, first] & self.occ_near[rows[idx, first], cols[idx, first]]
        # a few holes in a wall are tolerated; a corridor's open end (many rays) is not
        return bool((~hit_wall).sum() <= int(self.p("room_max_open_rays")))

    def is_room_spot_cached(self, xy) -> bool:
        """is_room_spot() costs 180 rays x 6 m of matrix math. The answer only changes when the
        map does, so cache it per goal (20 cm buckets) and wipe the cache on every new map."""
        if self.room_cache_counter != self.processed_counter:
            self.room_cache.clear()
            self.room_cache_counter = self.processed_counter
        key = self.goal_key(xy)
        hit = self.room_cache.get(key)
        if hit is None:
            hit = self.is_room_spot(xy[0], xy[1])
            self.room_cache[key] = hit
        return hit

    def project_along(self, start, yaw, dist):
        """March from `start` along `yaw` up to `dist` m, staying on free, wall-clear cells."""
        res = self.map_meta[0]
        steps = int(dist / res)
        if steps < 1:
            return start
        ds = np.arange(1, steps + 1) * res
        xs, ys = start[0] + ds * math.cos(yaw), start[1] + ds * math.sin(yaw)
        rows, cols, inb = self._ray_grid_coords(start[0], start[1], np.array([yaw]), ds)
        rows, cols, inb = rows[0], cols[0], inb[0]
        ok = inb & (self.free_clean[rows, cols] == 1) & self.clear_mask[rows, cols]
        k = steps if ok.all() else int(np.argmin(ok))
        return start if k == 0 else (float(xs[k - 1]), float(ys[k - 1]))

    def get_unseen_targets(self, pose) -> List[Target]:
        """compute_unseen_targets() runs connected components + per-label masks, so it is the
        heaviest part of a tick. Reuse the last result until the map changes or it gets stale."""
        now = self.now_s()
        if (self.unseen_cache_counter != self.processed_counter
                or now - self.unseen_cache_t >= float(self.p("unseen_refresh_s"))):
            self.unseen_cache = self.compute_unseen_targets(pose)
            self.unseen_cache_t = now
            self.unseen_cache_counter = self.processed_counter
        return self.unseen_cache

    def compute_unseen_targets(self, pose) -> List[Target]:
        """Free space the LiDAR mapped but the camera has not seen -> inspect viewpoints."""
        res = self.map_meta[0]
        rx, ry, _ = pose
        unseen = ((self.free_clean == 1) & (~self.seen)).astype(np.uint8)
        ko = max(1, int(round(float(self.p("unseen_open_m")) / res)))
        ko += (ko % 2 == 0)
        unseen = cv2.morphologyEx(unseen, cv2.MORPH_OPEN,
                                  cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko, ko)))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(unseen, connectivity=8)
        self.unseen_labels = labels.astype(np.int32)
        min_cells = int(math.ceil(float(self.p("min_unseen_area_m2")) / (res * res)))
        depth_cfg = float(self.p("inspect_depth_m"))
        r0, c0 = self.world_to_cell(rx, ry)
        out = []
        for lab in range(1, n):
            area = int(stats[lab, cv2.CC_STAT_AREA])
            if area < min_cells:
                continue
            x0, y0 = stats[lab, cv2.CC_STAT_LEFT], stats[lab, cv2.CC_STAT_TOP]
            bw, bh = stats[lab, cv2.CC_STAT_WIDTH], stats[lab, cv2.CC_STAT_HEIGHT]
            rr, cc = np.nonzero(labels[y0:y0 + bh, x0:x0 + bw] == lab)
            rr, cc = rr + y0, cc + x0
            clr = self.clear_mask[rr, cc]
            if not clr.any():
                continue
            cen = self.cell_to_world(rr.mean(), cc.mean())
            rrc, ccc = rr[clr], cc[clr]
            # Entrance = cluster cells touching free space the camera HAS seen (the
            # doorway interface). Straight-line "nearest to robot" can land on a room
            # corner behind a wall and skew the heading.
            ya, yb = max(0, y0 - 2), min(self.map_meta[3], y0 + bh + 2)
            xa, xb = max(0, x0 - 2), min(self.map_meta[4], x0 + bw + 2)
            in_cl = (labels[ya:yb, xa:xb] == lab)
            seen_free = (self.free_clean[ya:yb, xa:xb] == 1) & self.seen[ya:yb, xa:xb] & ~in_cl
            touch = cv2.dilate(seen_free.astype(np.uint8), np.ones((3, 3), np.uint8)) == 1
            ir, ic = np.nonzero(in_cl & touch & self.clear_mask[ya:yb, xa:xb])
            if ir.size:
                rrc, ccc = ir + ya, ic + xa
            j = np.argmin((rrc - r0) ** 2 + (ccc - c0) ** 2)      # nearest interface cell to robot
            entry = self.cell_to_world(rrc[j], ccc[j])
            dx, dy = cen[0] - entry[0], cen[1] - entry[1]
            span = math.hypot(dx, dy)
            yaw = math.atan2(dy, dx) if span > 0.1 else math.atan2(entry[1] - ry, entry[0] - rx)
            # Depth into the room is capped only by project_along's own wall/clearance
            # walk -- NOT by the doorway->centroid distance (`span`), which could be
            # much shorter than inspect_depth_m for an off-center or oddly-shaped room
            # and was stopping the robot short of a real 1m penetration.
            goal = self.project_along(entry, yaw, depth_cfg)
            out.append(Target(KIND_UNSEEN, lab, cen, goal, yaw, area, area * res * res))
        return out

    # ------------------------------------------- reachability (Nav2 planner)
    @staticmethod
    def goal_key(xy) -> Tuple[int, int]:
        return int(round(xy[0] / 0.2)), int(round(xy[1] / 0.2))     # 20 cm buckets

    def is_hard_unreachable(self, xy) -> bool:
        return self.path_fails.get(self.goal_key(xy), 0) >= int(self.p("unreachable_max_fails"))

    def path_status(self, xy):
        """'unknown' | 'unreachable' | path length in metres (from Nav2's own planner)."""
        info = self.path_info.get(self.goal_key(xy))
        if info is None:
            return "unknown"
        length, stamp = info
        if length is None:
            ttl = float(self.p("unreachable_hard_ttl_s" if self.is_hard_unreachable(xy)
                               else "unreachable_ttl_s"))
        else:
            ttl = float(self.p("path_cache_ttl_s"))
        if self.now_s() - stamp > ttl:
            return "unknown"
        return "unreachable" if length is None else length

    def request_path_check(self, t: Target):
        key = self.goal_key(t.goal)
        if key in self.path_pending:
            return
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = self.map_frame
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x, goal.goal.pose.position.y = t.goal
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False                      # plan from the robot's current pose
        self.path_pending.add(key)
        fut = self.path_client.send_goal_async(goal)
        fut.add_done_callback(lambda fu, k=key: self.path_response_cb(fu, k))

    def path_response_cb(self, fut, key):
        try:
            handle = fut.result()
        except Exception:
            self.path_pending.discard(key)
            return
        if not handle.accepted:
            self.path_pending.discard(key)
            return
        handle.get_result_async().add_done_callback(lambda fu, k=key: self.path_result_cb(fu, k))

    def path_result_cb(self, fut, key):
        self.path_pending.discard(key)
        try:
            res = fut.result()
        except Exception:
            return
        if res.status == GoalStatus.STATUS_SUCCEEDED:
            pts = np.array([[q.pose.position.x, q.pose.position.y] for q in res.result.path.poses])
            length = float(np.hypot(*np.diff(pts, axis=0).T).sum()) if len(pts) > 1 else 0.0
            self.path_info[key] = (length, self.now_s())
            self.path_fails[key] = 0
            self.global_fail_until, self.global_fail_episodes = 0.0, 0     # planner works again
        elif not self.robot_blocked:                  # a failure from a blocked start says nothing about the goal
            now = self.now_s()
            win = float(self.p("global_fail_window_s"))
            self.recent_fails = [(tt, kk) for tt, kk in self.recent_fails if now - tt < win] + [(now, key)]
            if len({kk for _, kk in self.recent_fails}) >= int(self.p("global_fail_targets")):
                # several DIFFERENT targets failed at once: the planner/robot is the problem, not them
                if now >= self.global_fail_until:
                    self.report_global_failure()
                self.global_fail_until = now + float(self.p("global_fail_hold_s"))
                self.recent_fails.clear()
                return
            if now < self.global_fail_until:
                return
            self.path_info[key] = (None, now)
            self.path_fails[key] = self.path_fails.get(key, 0) + 1

    def report_global_failure(self):
        """Log exactly what the robot/costmap look like, so the cause can be read off the log."""
        pose = self.get_pose(self.base_frame, timeout=0.05)
        info = "no pose"
        if pose is not None:
            info = f"robot ({pose[0]:.2f}, {pose[1]:.2f})"
            if self.costmap_arr is not None:
                r, c = self.costmap_cell(pose[0], pose[1])
                if 0 <= r < self.costmap_arr.shape[0] and 0 <= c < self.costmap_arr.shape[1]:
                    info += (f", costmap cost {int(self.costmap_arr[r, c])}, "
                             f"{self.costmap_edt[r, c]:.2f} m from nearest lethal cell")
                else:
                    info += ", OUTSIDE the costmap"
            else:
                info += ", costmap NOT received (check costmap_topic)"
            if self.wall_dist is not None and self.map_meta is not None:
                res, ox, oy, h, w = self.map_meta
                r, c = int(math.floor((pose[1] - oy) / res)), int(math.floor((pose[0] - ox) / res))
                if 0 <= r < h and 0 <= c < w:
                    info += f", {self.wall_dist[r, c]:.2f} m from nearest wall in the SLAM map"
        self.global_fail_episodes += 1
        self.get_logger().warn(f"Planner failed for several different targets at once ({info}). "
                               "Treating it as a robot/costmap problem, not a target problem.")
        if self.global_fail_episodes >= 3:            # keep exploring instead of waiting forever
            self.check_disabled_until = self.now_s() + 30.0
            self.global_fail_episodes = 0
            self.get_logger().warn("Planner keeps failing: skipping reachability checks for 30 s.")

    def report_unreachable(self):
        mx = int(self.p("unreachable_max_fails"))
        bad = [(k[0] * 0.2, k[1] * 0.2) for k, (ln, _) in self.path_info.items()
               if ln is None and self.path_fails.get(k, 0) >= mx]
        if bad:
            spots = ", ".join(f"({x:.1f}, {y:.1f})" for x, y in bad[:8])
            self.get_logger().warn(
                f"Nav2 could not plan to {len(bad)} target(s) near: {spots}. If these are rooms, "
                "the doorway is probably blocked in the costmap (robot_radius/inflation too big).")

    # ----------------------------------------------------- scoring/blacklist
    def is_blacklisted(self, x, y) -> bool:
        t = self.now_s()
        self.blacklist = [b for b in self.blacklist if b[2] > t]
        r = float(self.p("blacklist_radius_m"))
        return any(math.hypot(x - bx, y - by) < r for bx, by, _ in self.blacklist)

    def add_blacklist(self, xy):
        dur = float(self.p("blacklist_duration_s"))
        self.blacklist.append((xy[0], xy[1], self.now_s() + dur))
        self.get_logger().warn(f"Blacklisted ({xy[0]:.2f}, {xy[1]:.2f}) for {dur:.0f}s")

    def score_targets(self, pose) -> List[Target]:
        """Candidates sorted best-first. Distance = real path length once Nav2 confirmed it.

        score = w_dist * (dist + unseen_pen + turn_pen - door_bonus) - w_size * size
        (every term is stored in Target.parts for the scoreboard log)."""
        rx, ry, ryaw = pose
        wd, ws = (float(self.p(k)) for k in ("w_dist", "w_size"))
        unseen_penalty = float(self.p("unseen_score_penalty_m"))
        dead = math.radians(float(self.p("turn_deadzone_deg")))
        turn_penalty = float(self.p("turn_penalty_m"))
        door_bonus = float(self.p("passing_door_bonus_m"))
        door_radius = max(0.1, float(self.p("passing_door_radius_m")))
        use_check = (bool(self.p("path_check_enabled")) and self.path_client.server_is_ready()
                     and self.now_s() >= self.check_disabled_until)
        self.last_use_check = use_check

        live = []
        for t in list(self.frontier_targets) + self.get_unseen_targets(pose):
            if t.raw_goal is None:
                t.raw_goal = t.goal
            t.goal = self.snap_to_costmap(t.raw_goal)    # keep the goal where the body fits
            if self.is_blacklisted(*t.goal):
                continue
            if t.kind == KIND_UNSEEN and any(
                    math.hypot(t.goal[0] - vx, t.goal[1] - vy) < vrad for vx, vy, vrad in self.visited):
                continue
            live.append(t)
        self.all_targets = live                      # used for "is my goal still valid?"

        def score(t, dist):
            tx, ty = t.goal[0] - rx, t.goal[1] - ry
            straight = math.hypot(tx, ty)
            ang = abs(wrap_angle(math.atan2(ty, tx) - ryaw))     # 0..pi, left/right symmetric
            pen_unseen = unseen_penalty if t.kind == KIND_UNSEEN else 0.0
            # turn: free inside the dead zone, quadratic ramp only for targets further behind
            pen_turn = 0.0
            if ang > dead:
                f = (ang - dead) / max(1e-6, math.pi - dead)
                pen_turn = turn_penalty * f * f
            # door pull: UNSEEN targets get cheaper the closer the robot is to them (linear ramp)
            bonus = 0.0
            if t.kind == KIND_UNSEEN and door_bonus > 0.0:
                bonus = door_bonus * max(0.0, 1.0 - straight / door_radius)
            total = wd * (dist + pen_unseen + pen_turn - bonus) - ws * t.size_cells
            t.parts = {"ang_deg": math.degrees(ang), "dist_cost": wd * dist,
                       "turn_cost": wd * pen_turn, "unseen": wd * pen_unseen,
                       "door": -wd * bonus, "size": -ws * t.size_cells}
            return total

        out, soft, hard = [], 0, 0
        for t in live:
            t.dist = math.hypot(t.goal[0] - rx, t.goal[1] - ry)
            t.checked = not use_check
            t.score = score(t, t.dist)
            if use_check:
                st = self.path_status(t.goal)
                if st == "unreachable":
                    if self.is_hard_unreachable(t.goal):
                        hard += 1
                    else:
                        soft += 1                    # maybe just a costmap-lag hiccup: retry soon
                    continue
                if st != "unknown":
                    t.dist, t.checked = float(st), True
                    t.score = score(t, t.dist)
            out.append(t)
        out.sort(key=lambda t: t.score)
        self.soft_skipped, self.hard_skipped = soft, hard
        self.log_scoreboard(out, pose)
        if soft or hard:
            self.get_logger().info(f"Nav2 can't plan to {soft} target(s) yet, {hard} given up on.",
                                   throttle_duration_sec=5.0)
        if use_check and not self.robot_blocked and self.now_s() >= self.global_fail_until:
            k = int(self.p("path_check_top_k"))
            if self.state == NAVIGATING_TO_GOAL:      # don't spam the planner the BT is also using
                k = 1 if self.now_s() - self.last_nav_check > 2.0 else 0
                if k:
                    self.last_nav_check = self.now_s()
            for t in [x for x in out if not x.checked][:k]:
                self.request_path_check(t)
        return out

    def log_scoreboard(self, cands: List[Target], pose):
        """Every scoreboard_period_s: top 5 targets with the true distance, turn angle and every
        term that went into the final score (all terms are in score units and sum to SCORE).
        Distance marker: '*' = Nav2 path length, '~' = straight-line (not planner-checked yet)."""
        if not bool(self.p("scoreboard_enabled")):
            return
        now = self.now_s()
        if now - self.last_scoreboard_t < float(self.p("scoreboard_period_s")):
            return
        self.last_scoreboard_t = now
        head = (f"SCOREBOARD [{self.state}] robot=({pose[0]:.1f},{pose[1]:.1f}) "
                f"heading={math.degrees(pose[2]):.0f}deg")
        if not cands:
            self.get_logger().info(head + " -- no candidates")
            return
        lines = [head]
        for i, t in enumerate(cands[:5]):
            pt = t.parts
            mark = "*" if (self.last_use_check and t.checked) else "~"
            lines.append(
                f"  #{i + 1} {t.kind:<8} ({t.goal[0]:6.1f},{t.goal[1]:6.1f}) "
                f"dist={t.dist:5.2f}m{mark} turn={pt.get('ang_deg', 0.0):4.0f}deg | "
                f"dist={pt.get('dist_cost', 0.0):6.1f} turn={pt.get('turn_cost', 0.0):+5.1f} "
                f"unseen={pt.get('unseen', 0.0):+5.1f} "
                f"door={pt.get('door', 0.0):+6.1f} size={pt.get('size', 0.0):+5.1f} "
                f"=> SCORE {t.score:7.1f}")
        self.get_logger().info("\n".join(lines))

    # ------------------------------------------------------------ Nav2 goals
    def _dispatch(self, client, goal_msg):
        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_handle = None
        fut = client.send_goal_async(goal_msg)
        fut.add_done_callback(lambda fu, s=seq: self.goal_response_cb(fu, s))
        self.set_state(NAVIGATING_TO_GOAL)

    def send_goal(self, t: Target, pose):
        rx, ry, ryaw = pose
        self.invalid_since, self.preempt_key = None, None
        # Heading policy: ONLY in a room do we arrive facing into it. Everywhere else we arrive
        # facing the way we travelled, so there is no extra turn at the goal.
        room = self.is_room_spot_cached(t.goal)
        if room and not math.isnan(t.yaw):
            yaw = t.yaw
        else:
            gx, gy = t.goal[0] - rx, t.goal[1] - ry
            yaw = math.atan2(gy, gx) if math.hypot(gx, gy) > 0.3 else ryaw
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = t.goal
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.active = ActiveGoal(t.kind, t.goal, self.now_s(), committed=False, room=room)
        if t.kind == KIND_UNSEEN:
            self.sweep_count = 0
        self.last_kind = None
        self._dispatch(self.nav_client, goal)

        pt = PointStamped()
        pt.header.frame_id, pt.header.stamp = self.map_frame, goal.pose.header.stamp
        pt.point.x, pt.point.y = t.goal
        self.goal_pub.publish(pt)
        self.get_logger().info(
            f"Goal[{t.kind}{'/room' if room else ''}] -> ({t.goal[0]:.2f}, {t.goal[1]:.2f}) yaw={math.degrees(yaw):.0f}deg "
            f"path={t.dist:.2f}m extent={t.extent:.2f}")

    def send_sweep(self, delta_yaw, rx, ry):
        goal = Spin.Goal()
        goal.target_yaw = float(delta_yaw)
        goal.time_allowance = Duration(seconds=15.0).to_msg()
        self.active = ActiveGoal(KIND_SWEEP, (rx, ry), self.now_s(), committed=True, room=True)
        self.get_logger().info(f"Sweep: rotating {math.degrees(delta_yaw):.0f}deg "
                               f"({self.sweep_count}/{int(self.p('max_sweeps'))})")
        self._dispatch(self.spin_client, goal)

    def cancel_active_goal(self):
        self.goal_seq += 1                        # makes late callbacks inert
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.active = None

    def handle_failure(self, act: Optional[ActiveGoal]):
        if act is None:
            self.set_state(FINDING_FRONTIER)
        elif act.kind == KIND_SWEEP:              # give up sweeping, finalise this viewpoint
            self.sweep_count = int(self.p("max_sweeps"))
            self.last_kind = KIND_SWEEP
            self.set_state(GOAL_REACHED)
        else:
            self.add_blacklist(act.xy)
            self.set_state(FINDING_FRONTIER)

    def goal_response_cb(self, fut, seq):
        try:
            handle = fut.result()
        except Exception as ex:
            self.get_logger().error(f"send_goal failed: {ex}")
            if seq == self.goal_seq:
                act, self.active = self.active, None
                self.handle_failure(act)
            return
        if seq != self.goal_seq:                  # superseded before the server answered
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().warn("Goal rejected.")
            act, self.active = self.active, None
            self.handle_failure(act)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda fu, s=seq: self.result_cb(fu, s))

    def result_cb(self, fut, seq):
        if seq != self.goal_seq:
            return
        status = fut.result().status
        act, self.active, self.goal_handle = self.active, None, None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.last_kind = act.kind if act else None
            self.last_room = bool(act and act.room)
            self.set_state(GOAL_REACHED)
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Goal aborted by server.")
            self.handle_failure(act)
        else:
            self.set_state(FINDING_FRONTIER)

    # ----------------------------------------------------------- main loop
    def tick(self):
        if self.state == EXPLORATION_COMPLETE:
            return
        if self.state == INITIALIZING:
            if self.map_msg is None:
                self.get_logger().info("Waiting for /map ...", throttle_duration_sec=3.0)
                return
            if self.get_pose(self.base_frame) is None:
                return
            nav_ok = self.nav_client.server_is_ready()
            spin_ok = self.spin_client.server_is_ready()
            if not (nav_ok and spin_ok):
                missing = [n for n, ok in (("navigate_to_pose", nav_ok), ("spin", spin_ok)) if not ok]
                self.get_logger().info(f"Waiting for Nav2 action server(s): {', '.join(missing)} "
                                       "(is Nav2 fully active? check lifecycle + ros2 action list)",
                                       throttle_duration_sec=3.0)
                return
            self.refresh_map_cache()
            self.set_state(FINDING_FRONTIER)

        pose = self.get_pose(self.base_frame)
        if pose is None:
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("navigate_to_pose server not ready.", throttle_duration_sec=3.0)
            return
        self.refresh_map_cache()
        if self.free_clean is None:
            return
        self.update_robot_blocked(pose)

        if self.state == GOAL_REACHED:
            if self.maybe_sweep(pose):
                return
            self.set_state(FINDING_FRONTIER)

        cands = self.score_targets(pose)
        self.publish_markers(cands)
        if self.state == FINDING_FRONTIER:
            self.select_and_send(cands, pose)
        elif self.state == NAVIGATING_TO_GOAL:
            self.reevaluate(cands, pose)

    def select_and_send(self, cands: List[Target], pose):
        if not cands:
            if (self.soft_skipped > 0 or self.robot_blocked
                    or self.now_s() < self.global_fail_until):    # targets exist / robot is being freed
                self.empty_ticks = 0
                return
            self.empty_ticks += 1
            if self.empty_ticks >= int(self.p("complete_confirmations")):
                if self.hard_skipped > 0 and self.retry_round < int(self.p("unreachable_retry_rounds")):
                    self.retry_round += 1
                    self.get_logger().warn(f"{self.hard_skipped} target(s) judged unreachable; "
                                           f"re-trying them (round {self.retry_round}) before finishing.")
                    self.path_info.clear()
                    self.path_fails.clear()
                    self.empty_ticks = 0
                    return
                self.cancel_active_goal()
                self.set_state(EXPLORATION_COMPLETE)
                self.report_unreachable()
                self.get_logger().info("Exploration Complete!")
            return
        ready = [t for t in cands if t.checked]
        if not ready:                             # waiting for the planner's answer (next tick)
            return
        self.empty_ticks = 0
        self.retry_round = 0
        self.send_goal(ready[0], pose)

    def maybe_sweep(self, pose) -> bool:
        """After reaching an inspect viewpoint: rotate toward the largest unseen mass."""
        if self.last_kind not in (KIND_UNSEEN, KIND_SWEEP):
            return False
        self.update_seen()
        rx, ry, ryaw = pose
        res = self.map_meta[0]
        in_room = self.last_room and self.is_room_spot(rx, ry)
        if in_room and self.sweep_count < int(self.p("max_sweeps")):
            step = math.radians(float(self.p("sweep_yaw_step_deg")))
            best_yaw, best_n = None, 0
            for yaw in ryaw + np.arange(-math.pi, math.pi, step):       # ~18 candidates
                r, c = self.camera_cells(rx, ry, yaw)
                n = int((~self.seen[r, c]).sum())
                if n > best_n:
                    best_yaw, best_n = yaw, n
            if best_yaw is not None and best_n * res * res >= float(self.p("min_sweep_area_m2")):
                delta = wrap_angle(best_yaw - ryaw)
                if abs(delta) > 0.1:
                    self.sweep_count += 1
                    self.send_sweep(delta, rx, ry)
                    return True
        vr = float(self.p("visited_radius_m"))
        # Non-room spots get the SAME exclusion radius as room spots, so a sliver the camera
        # just barely missed next to a finished spot can't reappear as a "new" target one step away.
        self.visited.append((rx, ry, vr))
        self.last_kind, self.sweep_count = None, 0
        return False

    def reevaluate(self, cands: List[Target], pose):
        act = self.active
        if act is None:
            return
        now = self.now_s()
        if now - act.sent_t > float(self.p("goal_timeout_s")):
            self.get_logger().warn("Goal watchdog timeout.")
            self.cancel_active_goal()
            self.handle_failure(act)
            return
        if act.kind == KIND_SWEEP:
            return

        # ---- NO commit lock: the goal can be dropped or swapped at any moment.
        labels = self.frontier_labels if act.kind == KIND_FRONTIER else self.unseen_labels
        res = self.map_meta[0]
        R = max(1, int(float(self.p("goal_validity_radius_m")) / res))
        gr, gc = self.world_to_cell(*act.xy)
        win = labels[max(0, gr - R):gr + R + 1, max(0, gc - R):gc + R + 1]
        ids = set(np.unique(win).tolist()) - {0}
        near = [t for t in self.all_targets if t.kind == act.kind and t.label in ids]

        if not near:
            if self.invalid_since is None:
                self.invalid_since = now
            if now - self.invalid_since < float(self.p("invalid_grace_s")):
                return                            # frontiers flicker on dotted walls: wait before dropping it
            self.invalid_since = None
            self.get_logger().info("Active goal no longer valid; re-planning.")
            self.cancel_active_goal()
            self.set_state(FINDING_FRONTIER)
            return self.select_and_send(cands, pose)
        self.invalid_since = None

        # ---- switch by SCORE, every tick: same terms (distance, turn, door bonus) that picked the goal
        ready = [t for t in cands if t.checked]
        if not ready:
            return
        best = ready[0]
        if any(best is n for n in near):
            return                                # the current goal is still the best
        cur = min(n.score for n in near)
        if best.score + float(self.p("w_dist")) * float(self.p("switch_margin_m")) >= cur:
            return
        self.get_logger().info(f"SWITCH -> {best.kind} ({best.goal[0]:.1f},{best.goal[1]:.1f}) "
                               f"score {best.score:.1f} beats current goal {cur:.1f}.")
        self.cancel_active_goal()
        self.send_goal(best, pose)

    # --------------------------------------------------------------- viz
    def publish_markers(self, cands: List[Target]):
        arr = MarkerArray()
        d = Marker()
        d.action = Marker.DELETEALL
        arr.markers.append(d)
        stamp = self.get_clock().now().to_msg()
        for i, t in enumerate(cands):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.map_frame, stamp
            m.ns, m.id = "targets", i
            m.type = Marker.SPHERE if t.kind == KIND_FRONTIER else Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = t.goal
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.4 if i == 0 else 0.25
            m.color.a = 0.9
            m.color.r, m.color.g, m.color.b = (0.0, 1.0, 0.2) if t.kind == KIND_FRONTIER else (1.0, 0.6, 0.0)
            arr.markers.append(m)
        if cands:                                   # arrow = heading of the best goal
            a = Marker()
            a.header.frame_id, a.header.stamp = self.map_frame, stamp
            a.ns, a.id, a.type, a.action = "best_heading", 0, Marker.ARROW, Marker.ADD
            a.pose.position.x, a.pose.position.y = cands[0].goal
            y = cands[0].yaw if not math.isnan(cands[0].yaw) else 0.0
            a.pose.orientation.z, a.pose.orientation.w = math.sin(y / 2), math.cos(y / 2)
            a.scale.x, a.scale.y, a.scale.z = 0.6, 0.08, 0.08
            a.color.a, a.color.r = 1.0, 1.0
            arr.markers.append(a)
        self.frontier_pub.publish(arr)

    def publish_coverage(self):
        """RViz debug: 60 = free & seen by camera, 0 = free & NOT seen, -1 = not free."""
        self.refresh_map_cache()
        if self.seen is None or self.map_msg is None or self.free_clean is None:
            return
        if self.seen.shape != self.free_clean.shape:
            return
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_msg.info
        data = np.where(self.free_clean == 1, np.where(self.seen, 60, 0), -1).astype(np.int8)
        msg.data = data.ravel().tolist()
        self.coverage_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cancel_active_goal()
            node.stop_robot()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():                            # avoids "rcl_shutdown already called" on Ctrl-C
            rclpy.shutdown()


if __name__ == "__main__":
    main()
