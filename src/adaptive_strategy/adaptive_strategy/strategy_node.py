#!/usr/bin/env python3
"""ROS2 wrapper around StrategyCore.

Drives table_sim (and, once it exposes a matching /face_tool, the real
robot) through only /cmd_vel, /odom (via TF, see below), /scan,
/camera/detections, /face_tool and /game/time_remaining,/game/match_active -
no sim-only topics (/sim/score, /opponent/odom, /sim/pick_target_us) are
touched, so this node is portable.

Navigation is split, matching StrategyCore's own split (see core.py's
docstring): coarse legs (Command.goal_pose set) are handed to Nav2 via
NavigateToPose - this is what actually makes /global_costmap, /local_costmap
and /plan show up in RViz, and gets real obstacle-aware planning/avoidance
instead of this package's own simple GridPlanner driving blind. The final,
precise approach (goal_pose=None, Command.vx/vy/wz set) is driven directly -
Nav2's own goal tolerance is too loose for the exact alignment picking/
dropping needs.

Also publishes /strategy/belief_objects - the same oriented-box MarkerArray
format sim_node.py's /custom_objects uses (custom_objects_layer's costmap
plugin doesn't care which topic, it's just configured per-costmap - see
table_sim's nav2_sim.launch.py custom_objects_topic arg), but built from our
own belief instead of ground truth, so Nav2's costmap reflects only what
this strategy actually "knows", not sim-only omniscience.

Also publishes the robot's actual footprint (base triangle + whichever
faces are currently loaded - a held box sticks out ~32cm, well past the
bare body) to /local_costmap/footprint and /global_costmap/footprint
whenever it changes (nav2_params_sim.yaml's footprint param is empty for
both costmaps specifically so they subscribe instead of using a fixed
polygon) - otherwise Nav2 plans/avoids as if the robot stayed the bare
triangle size even while carrying boxes.

Current pose comes from a map -> base_footprint TF lookup (same pattern
table_sim's goal_driver.py uses - lookup_transform(..., Time()) for "latest
available", not a literal "now" lookup), not /odom directly: /odom lives in
the (potentially drifting) odom frame, while the strategy reasons about
stack/zone positions in the map frame.
"""

import json
import math
import os
from concurrent.futures import ThreadPoolExecutor

import yaml

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, Point32, Polygon, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, ColorRGBA, Float32, Int32, String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from table_sim.field import Field
from table_sim.geometry import polygon_from_truncated_triangle, rect_poly
from table_sim_interfaces.action import FaceTool
from table_sim_interfaces.msg import CameraDetections

from .core import Observation, StrategyCore, gap_fillers

GOAL_POS_TOL = 0.08    # re-send a Nav2 goal only once it actually moves this much
GOAL_YAW_TOL = 0.15


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _convex_hull(points):
    """Andrew's monotone chain. `points`: list of (x, y). Safe (if slightly
    conservative where the true shape is concave) stand-in for the real
    union of the body triangle + held-box rectangles - good enough for a
    collision footprint, which only needs to never be smaller than the
    truth."""
    pts = sorted(set((round(x, 5), round(y, 5)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _polygon_strips(poly, h=0.02):
    """A polygon (any shape, convex or not) as horizontal strips (cx, cy, length, height) - the costmap
    obstacle layer only draws oriented rectangles. Scanline at height h; strips overlap by 2 mm."""
    pts = [(float(x), float(y)) for x, y in poly]
    ys = [p[1] for p in pts]
    out, y = [], min(ys) + h / 2
    while y < max(ys):
        xs = []
        for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
            if (y1 > y) != (y2 > y):
                xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
        xs.sort()
        for a, b in zip(xs[0::2], xs[1::2]):
            if b - a > 1e-3:
                out.append(((a + b) / 2, y, b - a, h + 0.002))
        y += h
    return out


def _quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class StrategyNode(Node):
    def __init__(self):
        super().__init__('strategy_node')
        dp = self.declare_parameter
        dp('team', 'yellow')
        dp('field_file', '')
        dp('control_rate', 20.0)
        dp('map_frame', 'map')
        dp('base_frame', 'base_footprint')
        # StrategyCore's own tuning (drop_sequence, pick_sequence, dock pose
        # overrides, blocked_timeout_s, ...) lives in strategy.yaml, loaded
        # directly here (same pattern as field_file above: a path in, the
        # file's own contents read in plain Python) rather than through ROS
        # parameter declarations - this used to be silently skipped
        # (StrategyCore got {} every time), which is why editing strategy.yaml
        # never changed anything about a live (non-headless) run.
        dp('strategy_params_file', '')
        dp('brain', 'planner')     # 'planner' (PlannerBrain, beam search) | 'rules' (StrategyCore, rule brain)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.team = g('team')
        self.field = Field(g('field_file') or None)
        self.dt = 1.0 / float(g('control_rate'))
        self.map_frame = g('map_frame')
        self.base_frame = g('base_frame')

        sp_path = g('strategy_params_file')
        strategy_params = {}
        if sp_path:
            with open(sp_path) as f:
                strategy_params = (yaml.safe_load(f).get('strategy_node') or {}).get('ros__parameters') or {}
            dpf = strategy_params.get('dock_poses_file')
            if dpf and not os.path.isabs(dpf):
                strategy_params['dock_poses_file'] = os.path.join(os.path.dirname(sp_path), dpf)
        # Carrying: the footprint grows by the held boxes (footprint_carry, see _footprint_polygon) and
        # the global costmap's inflation switches to inflation_radius_carry_m.
        self._infl_empty = float(strategy_params.get('inflation_radius_empty_m', 0.28))
        self._infl_carry = float(strategy_params.get('inflation_radius_carry_m', 0.40))
        self._infl_node = str(strategy_params.get('inflation_param_node', '/global_costmap/global_costmap'))
        self._footprint_carry = bool(strategy_params.get('footprint_carry', True))   # grow the footprint by the held boxes
        # global costmap while carrying: a circle around the held boxes (NavFn plans the centre point only
        # and keeps it out of the footprint's INSCRIBED radius - the real hull's is 0.11 m until all 3
        # faces are loaded, so 1-2 held stacks were invisible to the path; Max)
        self._footprint_global_margin = float(strategy_params.get('footprint_global_margin_m', 0.02))
        # > 0: the loaded global circle's radius, fixed (Max: 0.29 closed the castle-border corridor at P1);
        # the controller's real-hull check against the local costmap covers the rest of the box reach
        self._footprint_global_r = float(strategy_params.get('footprint_global_radius_m', 0.0))
        # evacuation (Max): robot centre in a blocked global-costmap cell -> nearest free point within this
        # radius; none there (closed pocket) -> the global footprint falls back to the bare robot this long
        self._evac_search = float(strategy_params.get('evac_search_m', 0.40))
        self._evac_inset = float(strategy_params.get('evac_inset_m', 0.02))
        self._evac_bare_s = float(strategy_params.get('evac_bare_s', 2.0))
        self._gcm, self._bare_until = None, -1.0
        # keep-out zones for the global costmap (field keepouts:, Max): PAMI zones for all, the opponent's
        # start zone always, the opponent's castle from keepout_from_s; ours stay free
        self._keepout_from = float(strategy_params.get('keepout_from_s', 20.0))
        self._keepout_always, self._keepout_castle = [], []
        for k in (self.field.cfg.get('keepouts') or []):
            if k.get('kind') == 'pami' or k.get('team') != self.team:
                self._keepout_always += _polygon_strips(k['points'])
        opp_castle = self.field.castle.get('blue' if self.team == 'yellow' else 'yellow')
        if opp_castle is not None and len(opp_castle):
            self._keepout_castle = _polygon_strips(opp_castle.tolist())
        self._infl_want, self._infl_sent, self._infl_client, self._infl_fut = None, None, None, None
        if g('brain') == 'rules':
            self.brain = StrategyCore(self.field, self.team, strategy_params)
        else:
            from .planner_brain import PlannerBrain
            self.brain = PlannerBrain(self.field, self.team, strategy_params)
        self.get_logger().info(f'strategy_node: brain = {type(self.brain).__name__}')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose = None
        self.time_left = self.field.match_duration
        self.active = False
        self._got_time = False
        self._last_scan = None
        self._scan_yaw_offset = None    # resolved lazily from TF, see _tick
        self._last_cam = None
        self._result = None
        self._action_in_flight = False
        self._pending_face = None
        self._pending_mode = None
        # FLIP's own concurrent slot - tracked independently so it can run
        # alongside a PICK/DROP on another face (see world.py's flip_action).
        self._flip_result = None
        self._flip_in_flight = False
        self._pending_flip_face = None
        self._debug_log_len = 0   # how much of brain.debug.log has been forwarded to the ROS logger

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(CameraDetections, '/camera/detections', self._on_cam, 10)
        self.create_subscription(Float32, '/game/time_remaining', self._on_time, 10)
        self.create_subscription(Bool, '/game/match_active', self._on_active, 10)
        self.create_subscription(Int32, '/sim/reset_event', self._on_reset, 10)
        # Opponent seen from everywhere (strategy.yaml opponent_beacon, Max): the real robot gets it
        # from an external beacon; in the sim that is /opponent/odom. Off -> lidar only.
        self._opp_xy = None
        if strategy_params.get('opponent_beacon'):
            self.create_subscription(Odometry, '/opponent/odom', self._on_opp_odom, 10)
            self.get_logger().info('strategy_node: opponent beacon on (/opponent/odom)')

        self.tool = ActionClient(self, FaceTool, '/face_tool')

        # --------------------------------------------------------- Nav2
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._nav_target = None         # (x, y, th) currently asked of Nav2
        self._nav_handle = None
        self._nav_sending = False       # guards against overlapping send_goal_async calls

        # ------------------------------------------------- belief costmap
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_objects = self.create_publisher(MarkerArray, '/strategy/belief_objects', latched)
        self._objects_key = None
        self._next_objects_resync = 0.0

        # ----------------------------------------------------- footprint
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._on_gcm, latched)
        self.pub_footprint_local = self.create_publisher(Polygon, '/local_costmap/footprint', latched)
        self.pub_footprint_global = self.create_publisher(Polygon, '/global_costmap/footprint', latched)
        self._footprint_key = None
        self._last_footprint_poly = None
        # Costmap2DROS's footprint_sub_ subscribes VOLATILE/BEST_EFFORT (see
        # sim_node.py's own identical note) - a late-joining Nav2 won't
        # replay this publisher's transient-local backlog, so a subscriber
        # that connects between two face/vertical changes would otherwise
        # never see a footprint at all until the next one happens to occur.
        # Resend on a slow timer regardless of the change-detection cache
        # below (which only skips redundant recompute/publish on an
        # unchanged tick, not this).
        self.create_timer(1.0, self._resend_footprint)

        # ------------------------------------------------- status/markers
        self.pub_status = self.create_publisher(String, '/strategy/status', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/strategy/markers', 10)
        # The required heading at every pick/drop only ever existed inside a
        # NavigateToPose action goal (a service under the hood, so not
        # recordable) - publish it as a plain topic so a bag replay can show
        # what angle was asked for next to what the robot actually did.
        self.pub_goal = self.create_publisher(PoseStamped, '/strategy/goal', 10)

        # The brain runs in a worker thread: a replan takes 0.1-0.5 s, which used to block this
        # node's control tick (missed control-loop rate, last velocity held during the pause).
        # The tick never waits for it: while it thinks, Nav2 keeps driving a coarse leg on its own
        # and a direct-velocity stage gets a zero command (stop rather than hold a stale one).
        self._worker = ThreadPoolExecutor(max_workers=1)
        self._step_fut = None
        self._step_inputs = None
        self._last_step_ns = None
        self._last_cmd_direct = False
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f'strategy_node ready: team={self.team}')

    # ------------------------------------------------------------ inbound
    def _on_scan(self, msg):
        self._last_scan = msg

    def _on_cam(self, msg):
        self._last_cam = dict(zip(msg.names, msg.counts))

    def _on_time(self, msg):
        self.time_left = float(msg.data)
        self._got_time = True

    def _on_active(self, msg):
        self.active = bool(msg.data)

    def _on_reset(self, msg):
        # /sim/reset doesn't restart this node - without this, StrategyCore's
        # belief (what we think we've dropped, tracked opponent position,
        # current target, ...) just carries over stale from the previous
        # match, which is how "drop zone on our side doesn't reset" happens.
        self._cancel_nav()
        self.brain.reset()
        self._action_in_flight = False
        self._result = None
        self._pending_face = self._pending_mode = None
        self._flip_in_flight = False
        self._flip_result = None
        self._pending_flip_face = None
        self._objects_key = None
        self._footprint_key = None
        self.get_logger().info('strategy_node: reset (brain state cleared)')

    def _update_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return False
        t = tf.transform.translation
        self.pose = (t.x, t.y, _yaw(tf.transform.rotation))
        return True

    def _get_scan_yaw_offset(self, scan_frame):
        """base_frame -> scan_frame yaw (static, so resolved once and
        cached) - see the long comment at the _tick call site."""
        if self._scan_yaw_offset is not None:
            return self._scan_yaw_offset
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, scan_frame, rclpy.time.Time())
        except TransformException:
            return 0.0
        self._scan_yaw_offset = _yaw(tf.transform.rotation)
        return self._scan_yaw_offset

    # -------------------------------------------------------------- action
    def _send_action(self, face, mode, count=0):
        if not self.tool.server_is_ready():
            return
        goal = FaceTool.Goal()
        goal.face, goal.mode, goal.count = face, mode, count
        self._pending_face, self._pending_mode = face, mode
        self._action_in_flight = True
        self.tool.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self._action_in_flight = False
            return
        gh.get_result_async().add_done_callback(self._on_action_result)

    def _send_flip_action(self, face):
        if not self.tool.server_is_ready():
            return
        goal = FaceTool.Goal()
        goal.face, goal.mode = face, FaceTool.Goal.FLIP
        self._pending_flip_face = face
        self._flip_in_flight = True
        self.tool.send_goal_async(goal).add_done_callback(self._on_flip_goal_response)

    def _on_flip_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self._flip_in_flight = False
            return
        gh.get_result_async().add_done_callback(self._on_flip_action_result)

    def _on_flip_action_result(self, fut):
        r = fut.result().result
        self._flip_result = (self._pending_flip_face, r.success, r.count, r.message)
        self._flip_in_flight = False

    def _on_action_result(self, fut):
        r = fut.result().result
        self._result = (self._pending_face, self._pending_mode, r.success, r.count, r.message)
        self._action_in_flight = False

    # ---------------------------------------------------------------- nav2
    def _nav_matches(self, pose):
        if self._nav_target is None:
            return False
        dx = pose[0] - self._nav_target[0]
        dy = pose[1] - self._nav_target[1]
        dth = math.atan2(math.sin(pose[2] - self._nav_target[2]), math.cos(pose[2] - self._nav_target[2]))
        return math.hypot(dx, dy) < GOAL_POS_TOL and abs(dth) < GOAL_YAW_TOL

    def _send_nav_goal(self, pose):
        if self._nav_sending or not self.nav.server_is_ready():
            return
        self._nav_sending = True
        self._nav_target = pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = TimeMsg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[0], pose[1]
        q = _quat(pose[2])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = q[2], q[3]
        prev = self._nav_handle
        self._nav_handle = None
        if prev is not None:
            prev.cancel_goal_async()
        self.nav.send_goal_async(goal).add_done_callback(self._on_nav_response)

    def _on_nav_response(self, fut):
        self._nav_sending = False
        gh = fut.result()
        if gh is not None and gh.accepted:
            self._nav_handle = gh
            gh.get_result_async().add_done_callback(lambda f, gh=gh: self._on_nav_result(gh))

    def _on_nav_result(self, gh):
        # Nothing to do with the outcome here: StrategyCore's own arrival
        # check (self.pose vs. its target, every tick) is what actually
        # drives the FSM forward - it's true regardless of how we got
        # there. If Nav2 gave up short, that just shows up as "not
        # arrived yet", and the existing blocked_t timeout takes over.
        # Only clear if this result is still for the currently-active goal
        # (an old goal's result can arrive after a newer one replaced it).
        if self._nav_handle is gh:
            self._nav_handle = None

    def _cancel_nav(self):
        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
        self._nav_target, self._nav_handle = None, None

    # ------------------------------------------------------- footprint
    def _footprint_polygon(self):
        """The robot hexagon, grown by the boxes held on each loaded face (Max: test - the carried
        stacks hit piles and the opponent because Nav2 only knew the bare robot). Convex hull of the
        hexagon + every held stack's rectangle (lying: box_length along the face, standing: the 3
        slots, 3 x box_width), placed like world.py's held_poly (r_in + box_width / 2 off the face)."""
        b = self.brain
        pts = [tuple(p) for p in polygon_from_truncated_triangle(
            b.face_long_m, b.face_short_m, b.face_angles)]
        if not self._footprint_carry or not any(b.faces):
            return pts
        bl, bw = self.field.box_length, self.field.box_width
        for f, n in enumerate(b.faces):
            if not n:
                continue
            a = b.face_angles[f]
            nx, ny = math.cos(a), math.sin(a)
            tx, ty = -ny, nx
            along = 3 * bw if b.vertical[f] else bl
            for sa in (-along / 2, along / 2):
                for d in (b.r_in, b.r_in + bw):
                    pts.append((nx * d + tx * sa, ny * d + ty * sa))
        return _convex_hull(pts)

    def _update_inflation(self):
        """inflation_radius of the global costmap: larger while at least one box is carried."""
        want = self._infl_carry if sum(self.brain.faces) > 0 else self._infl_empty
        if want == self._infl_sent or (self._infl_fut is not None and not self._infl_fut.done()):
            return
        if self._infl_client is None:
            from rclpy.parameter_client import AsyncParameterClient
            self._infl_client = AsyncParameterClient(self, self._infl_node)
        if not self._infl_client.services_are_ready():
            return
        from rclpy.parameter import Parameter
        self._infl_fut = self._infl_client.set_parameters(
            [Parameter('inflation_layer.inflation_radius', Parameter.Type.DOUBLE, want)])
        self._infl_sent = want
        self.get_logger().info(f'global costmap inflation_radius -> {want:.2f} m')

    def _publish_footprint(self):
        # vertical state matters too, not just the counts - a flip changes
        # a loaded face's footprint from a single box_length pile to a
        # 3x-box_width span with the count unchanged, which the counts
        # alone wouldn't detect (the published footprint would then keep
        # showing the pre-flip shape).
        key = (tuple(self.brain.faces), tuple(self.brain.vertical),
               self.get_clock().now().nanoseconds * 1e-9 < self._bare_until)
        if key == self._footprint_key:
            return
        if self._footprint_carry and any(self.brain.faces) and self._infl_sent != self._infl_carry:
            return          # loaded footprint only once the carry inflation is set (Nav2: inflation >= inscribed)
        self._footprint_key = key
        # nav2_costmap_2d's footprint_sub_ is a plain geometry_msgs/Polygon,
        # not PolygonStamped (costmap_2d_ros.hpp) - that type mismatch meant
        # these were never actually being received at all, silently leaving
        # Nav2 on its default circular robot_radius_ footprint.
        pts = self._footprint_polygon()
        poly = Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in pts])
        gpoly = poly
        bare = self.get_clock().now().nanoseconds * 1e-9 < self._bare_until
        if self._footprint_carry and any(self.brain.faces) and not bare:
            # inscribed radius = the held boxes' outer corner + margin (16-gon, its apothem is that radius)
            r_in = (self._footprint_global_r if self._footprint_global_r > 0
                    else max(math.hypot(x, y) for x, y in pts) + self._footprint_global_margin)
            r = r_in / math.cos(math.pi / 16)
            gpoly = Polygon(points=[Point32(x=float(r * math.cos(2 * math.pi * k / 16)),
                                            y=float(r * math.sin(2 * math.pi * k / 16)), z=0.0) for k in range(16)])
        self._last_footprint_poly = (poly, gpoly)
        self.pub_footprint_local.publish(poly)
        self.pub_footprint_global.publish(gpoly)

    def _on_gcm(self, msg):
        import numpy as np
        i = msg.info
        self._gcm = (np.asarray(msg.data, dtype=np.int16).reshape(i.height, i.width),
                     i.resolution, i.origin.position.x, i.origin.position.y)

    def _evac_point(self):
        """Nearest point the global planner can start from, when the robot centre is in a cell it treats
        as blocked (OccupancyGrid >= 99: inscribed / lethal), else None. No free cell within evac_search_m
        (a closed pocket): publish the bare footprint to the global costmap for evac_bare_s instead."""
        if self._gcm is None or self.pose is None or not any(self.brain.faces):
            return None
        import numpy as np
        g, res, ox, oy = self._gcm
        x, y = self.pose[0], self.pose[1]
        ci, cj = int((y - oy) / res), int((x - ox) / res)
        H, W = g.shape
        if not (0 <= ci < H and 0 <= cj < W) or g[ci, cj] < 99:
            return None
        k = int(self._evac_search / res)
        i0, i1, j0, j1 = max(0, ci - k), min(H, ci + k + 1), max(0, cj - k), min(W, cj + k + 1)
        win = g[i0:i1, j0:j1]
        ii, jj = np.nonzero((win < 99) & (win >= 0))
        if len(ii) == 0:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now >= self._bare_until:
                self.get_logger().info('evacuate: no free cell within %.2f m, bare footprint for %.1f s'
                                       % (self._evac_search, self._evac_bare_s))
            self._bare_until = now + self._evac_bare_s
            return None
        px, py = ox + (jj + j0 + 0.5) * res, oy + (ii + i0 + 0.5) * res
        d = np.hypot(px - x, py - y)
        n = int(np.argmin(d))
        if d[n] > self._evac_search:
            return None
        ex, ey, dn = float(px[n]), float(py[n]), float(d[n])
        if dn > 1e-6:                     # a little past the edge, into the free side
            ex, ey = ex + (ex - x) / dn * self._evac_inset, ey + (ey - y) / dn * self._evac_inset
        return (ex, ey)

    def _resend_footprint(self):
        if self._last_footprint_poly is not None:
            self.pub_footprint_local.publish(self._last_footprint_poly[0])
            self.pub_footprint_global.publish(self._last_footprint_poly[1])

    def _drain_debug_log(self):
        """Forward StrategyCore.debug.say() messages to the ROS logger -
        without this, the brain's own reasoning ("stuck heading to X,
        abandoning", "P7 needs boxes on face 0, deferring", ...) is kept in
        an in-memory list (Debug.log) that nothing else ever reads, so a
        live run gives zero visibility into WHY it did something (e.g. a
        stuck backout with "no apparent reason" - the reason was always
        being recorded, just never surfaced). Debug.log itself is trimmed
        to its last 20 entries, so track how many have ever been said
        (Debug.log_total, monotonic) rather than the list's own length."""
        log = self.brain.debug.log
        total = self.brain.debug.log_total
        missed = total - self._debug_log_len
        for msg in log[max(0, len(log) - missed):]:
            self.get_logger().info(f'brain: {msg}')
        self._debug_log_len = total

    # --------------------------------------------------- status/markers
    def _publish_status(self, t, cmd=None):
        b = self.brain
        data = {
            't': round(t, 1), 'time_left': round(self.time_left, 1),
            'state': b.debug.state, 'mode': b._mode,
            'target': getattr(b.target_obj, 'label', None), 'stage': b.stage,
            'faces': list(b.faces),
            'opponent': b.debug.opp,
            # 'belief' = decay-weighted effective count (what pick-scoring
            # uses); 'belief_raw' = last actually observed/assumed count
            # before decay - NOT ground truth, just the undiscounted value.
            'belief': {s.label: round(b.belief.believed_count(s, t), 1) for s in self.field.stacks},
            'belief_raw': {s.label: b.belief._count.get(s.name) for s in self.field.stacks},
        }
        data.update(self._goal_fields(cmd))
        self.pub_status.publish(String(data=json.dumps(data)))

    def _goal_fields(self, cmd):
        """Everything needed to check, after the fact, that the heading asked
        for at a pick/drop was the intended one and was actually reached.

        The requested yaw comes from three hops, all of which are worth
        recording because a mistake can happen at any of them:
          field.py's Spot.face_heading   - which way the working face must
                                           point (stack.approach/zone.approach_deg)
          - face_angles[face]            - the face we chose to use
          = the robot yaw that achieves it (what gets sent as the goal)
        """
        b = self.brain
        sp, face = b.spot, b.face
        out = {
            'face': face,
            # coarse Nav2 leg's goal; None while the brain drives directly
            'goal': list(cmd.goal_pose) if (cmd is not None and cmd.goal_pose is not None) else None,
            'face_heading': None, 'req_yaw': None, 'req_yaw_deg': None,
            'yaw_err_deg': None, 'dock_pose': None,
        }
        if sp is None:
            return out
        req = _wrap(sp.face_heading - b.face_angles[face])
        out['face_heading'] = round(sp.face_heading, 4)
        out['req_yaw'] = round(req, 4)
        out['req_yaw_deg'] = round(math.degrees(req), 1)
        out['yaw_err_deg'] = round(math.degrees(_wrap(req - self.pose[2])), 1)
        # The final docking pose is never sent to Nav2 (the brain drives that
        # last leg itself), so without this it appears nowhere at all.
        out['dock_pose'] = [round(sp.x, 4), round(sp.y, 4), round(req, 4)]
        return out

    def _publish_goal(self, cmd):
        """The pose currently being driven to, as a real topic for RViz/bags:
        the Nav2 goal on a coarse leg, else the final docking pose."""
        b = self.brain
        if cmd is not None and cmd.goal_pose is not None:
            x, y, th = cmd.goal_pose
        elif b.spot is not None:
            x, y = b.spot.x, b.spot.y
            th = _wrap(b.spot.face_heading - b.face_angles[b.face])
        else:
            return
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        q = _quat(float(th))
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        self.pub_goal.publish(msg)

    def _publish_opponent_marker(self):
        opp = self.brain.tracker.opp
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'opponent_belief', 0
        if not opp.seen_ever:
            m.action = Marker.DELETEALL
            arr.markers.append(m)
            self.pub_markers.publish(arr)
            return
        m.type, m.action = Marker.CYLINDER, Marker.ADD
        m.pose.position = Point(x=float(opp.x), y=float(opp.y), z=0.05)
        m.pose.orientation.w = 1.0
        d = max(0.1, min(1.0, 2.0 * opp.sigma))
        m.scale.x = m.scale.y = d
        m.scale.z = 0.1
        m.color = ColorRGBA(r=1.0, g=0.1, b=0.6, a=0.9) if opp.visible else ColorRGBA(r=0.6, g=0.1, b=0.6, a=0.35)
        arr.markers.append(m)
        self.pub_markers.publish(arr)

    # ------------------------------------------------------ belief costmap
    def _publish_belief_objects(self, t):
        occ_stacks, occ_zones = self.brain.occupied(t)
        castle_on = t >= self._keepout_from
        key = (tuple(sorted(s.name for s in occ_stacks) + sorted(z.name for z in occ_zones)),
               len(self.brain.discard_piles), castle_on)
        # CustomObjectsLayer marks itself "not current" on every message and
        # needs a fresh one periodically to ever go current again - without
        # this safety-net resync a stable belief (nothing picked yet, so the
        # occupied set never changes) means exactly one message ever goes
        # out, the costmap never reports current, and Nav2 silently stops
        # producing velocity output. Same fix sim_node.py's own
        # _publish_custom_objects already uses, just missed it here.
        now = self.get_clock().now().nanoseconds * 1e-9
        if key == self._objects_key and now < self._next_objects_resync:
            return
        self._objects_key = key
        self._next_objects_resync = now + 1.0
        f = self.field
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        mid = 0
        stamp = self.get_clock().now().to_msg()
        for s in occ_stacks:
            arr.markers.append(self._obj_marker(mid, stamp, s.x, s.y, s.th, f.box_length, f.box_width))
            mid += 1
        for z in occ_zones:
            zx, zy, zth = self.brain.zone_obstacle_pose(z)
            # Circle zones only ever hold a flipped (standing) box -
            # box_width x box_width, not lying-flat box_length x box_width.
            lx = f.box_width if z.shape == 'circle' else f.box_length
            arr.markers.append(self._obj_marker(mid, stamp, zx, zy, zth, lx, f.box_width))
            mid += 1
        castle = []
        for z in occ_zones:
            zx, zy, zth = self.brain.zone_obstacle_pose(z)
            castle.append(rect_poly(zx, zy, f.box_width if z.shape == 'circle' else f.box_length, f.box_width, zth))
        for (dx_, dy_, dth) in self.brain.discard_piles:      # our discarded extras next to a wall
            arr.markers.append(self._obj_marker(mid, stamp, dx_, dy_, dth, f.box_length, f.box_width))
            mid += 1
            castle.append(rect_poly(dx_, dy_, f.box_length, f.box_width, dth))
        # close the gaps between castle objects the robot cannot pass (Y2-Y4, Y1-Y3, ...): Nav2 only
        # keeps the bare robot's inscribed radius clear and planned through them
        for (cx, cy, th, ln, wd) in gap_fillers(castle, self.brain.gap_block):
            arr.markers.append(self._obj_marker(mid, stamp, cx, cy, th, ln, wd))
            mid += 1
        for (cx, cy, ln, wd) in self._keepout_always + (self._keepout_castle if castle_on else []):
            arr.markers.append(self._obj_marker(mid, stamp, cx, cy, 0.0, ln, wd))
            mid += 1
        self.pub_objects.publish(arr)

    def _obj_marker(self, mid, stamp, x, y, th, lx, ly):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.map_frame, stamp
        m.ns, m.id, m.type, m.action = 'belief_objects', mid, Marker.CUBE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.055
        q = _quat(th)
        m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
        m.scale.x, m.scale.y, m.scale.z = float(lx), float(ly), 0.11
        m.color = ColorRGBA(r=0.9, g=0.3, b=0.3, a=0.5)
        return m

    def _on_opp_odom(self, msg):
        self._opp_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # ---------------------------------------------------------------- tick
    def _tick(self):
        if not self._got_time or not self._update_pose():
            if self._step_fut is None:
                self.pub_cmd.publish(Twist())
            return

        fut = self._step_fut
        if fut is not None:
            if not fut.done():
                # the brain is still thinking (replan): never block the control tick on it
                if self._last_cmd_direct:
                    self.pub_cmd.publish(Twist())
                return
            self._collect(fut)

        self._launch_step()
        # usual case: the step is quick - wait a fraction of the period and apply it this same
        # tick (no added latency for the docking controller); a long replan is collected later
        try:
            self._step_fut.result(timeout=0.6 * self.dt)
        except Exception:
            pass
        if self._step_fut.done():
            self._collect(self._step_fut)

    def _collect(self, fut):
        self._step_fut = None
        try:
            cmd = fut.result()
        except Exception as e:      # keep the node alive; the next step retries
            self.get_logger().error(f'brain step failed: {e!r}')
            self.pub_cmd.publish(Twist())
            return
        self._apply_cmd(self._step_inputs, cmd)

    def _launch_step(self):
        scan = self._last_scan
        self._last_scan = None
        cam = self._last_cam
        self._last_cam = None
        result = self._result
        self._result = None
        flip_result = self._flip_result
        self._flip_result = None

        if scan is not None:
            ranges = list(scan.ranges)
            angle_min, angle_inc = scan.angle_min, scan.angle_increment
            # LaserScan.angle_min/max are in the scan's OWN frame - they do
            # NOT include the sensor's mount yaw (base_scan is yawed by pi
            # off base_footprint, see CLAUDE.md/the URDF). world.py's own
            # raycast bakes that offset into the angles it simulates with
            # (lidar.yaw_offset, see headless.py's identical lookup), so
            # skipping it here silently reflected every tracked opponent
            # position by 180 degrees through our own robot. Resolved once
            # via TF (static transform) and cached - real robot-portable,
            # unlike reading sim.yaml's yaw_offset directly.
            lidar_yaw = self._get_scan_yaw_offset(scan.header.frame_id)
        else:
            ranges, angle_min, angle_inc, lidar_yaw = None, -math.pi, 0.0, 0.0

        # match_time derived from the authoritative /game/time_remaining
        # clock (not wall time, which wouldn't track a sim rtf != 1 and
        # would start counting before the match even begins).
        t = max(0.0, self.field.match_duration - self.time_left)

        obs = Observation(t=t, active=self.active, time_left=self.time_left,
                          pose=self.pose, ranges=ranges, angle_min=angle_min,
                          angle_inc=angle_inc, lidar_yaw=lidar_yaw, cam=cam,
                          tool_busy=self._action_in_flight, result=result,
                          flip_busy=self._flip_in_flight, flip_result=flip_result,
                          opp_xy=self._opp_xy, evac=self._evac_point())
        # real elapsed (sim) time since the last step: a step after a long replan covers more
        # than one control period (blocked / stall timers must count it)
        now_ns = self.get_clock().now().nanoseconds
        dt = self.dt if self._last_step_ns is None else (now_ns - self._last_step_ns) * 1e-9
        dt = max(self.dt, min(1.0, dt))
        self._last_step_ns = now_ns
        self._step_inputs = t
        self._step_fut = self._worker.submit(self.brain.step, obs, dt)

    def _apply_cmd(self, t, cmd):
        self._drain_debug_log()
        # Published after step(), not before: a drop result processed this
        # tick updates belief inside step() - publishing beforehand meant
        # the costmap (and any Nav2 goal sent later this same tick) used
        # last tick's stale belief, occasionally planning a path straight
        # through the zone/stack that was *just* dropped into.
        self._publish_belief_objects(t)
        self._publish_footprint()
        self._update_inflation()
        self._publish_status(t, cmd)
        self._publish_opponent_marker()
        self._publish_goal(cmd)

        if cmd.goal_pose is not None:
            self._last_cmd_direct = False
            if not self._nav_matches(cmd.goal_pose):
                self._send_nav_goal(cmd.goal_pose)
        else:
            self._last_cmd_direct = True
            if self._nav_target is not None:
                self._cancel_nav()
            tw = Twist()
            tw.linear.x, tw.linear.y, tw.angular.z = cmd.vx, cmd.vy, cmd.wz
            self.pub_cmd.publish(tw)

        if cmd.action is not None and not self._action_in_flight:
            self._send_action(*cmd.action)
        if cmd.flip_action is not None and not self._flip_in_flight:
            self._send_flip_action(*cmd.flip_action)


def main(args=None):
    rclpy.init(args=args)
    node = StrategyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
