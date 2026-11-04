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

import yaml

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, Point32, Polygon, PoseStamped, Twist
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
from table_sim.geometry import polygon_from_face_angles, rect_poly
from table_sim_interfaces.action import FaceTool
from table_sim_interfaces.msg import CameraDetections

from .core import Observation, StrategyCore

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
        self.brain = StrategyCore(self.field, self.team, strategy_params)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose = None
        self.time_left = self.field.match_duration
        self.active = False
        self._got_time = False
        self._last_scan = None
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

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(CameraDetections, '/camera/detections', self._on_cam, 10)
        self.create_subscription(Float32, '/game/time_remaining', self._on_time, 10)
        self.create_subscription(Bool, '/game/match_active', self._on_active, 10)
        self.create_subscription(Int32, '/sim/reset_event', self._on_reset, 10)

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
        self.pub_footprint_local = self.create_publisher(Polygon, '/local_costmap/footprint', latched)
        self.pub_footprint_global = self.create_publisher(Polygon, '/global_costmap/footprint', latched)
        self._footprint_key = None

        # ------------------------------------------------- status/markers
        self.pub_status = self.create_publisher(String, '/strategy/status', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/strategy/markers', 10)
        # The required heading at every pick/drop only ever existed inside a
        # NavigateToPose action goal (a service under the hood, so not
        # recordable) - publish it as a plain topic so a bag replay can show
        # what angle was asked for next to what the robot actually did.
        self.pub_goal = self.create_publisher(PoseStamped, '/strategy/goal', 10)

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
        b = self.brain
        pts = [tuple(p) for p in polygon_from_face_angles(b.side, b.face_angles)]
        for f, n in enumerate(b.faces):
            if n <= 0:
                continue
            a = b.face_angles[f]
            cx, cy = b.r_in * math.cos(a), b.r_in * math.sin(a)
            nx, ny = math.cos(a), math.sin(a)
            px = cx + nx * (self.field.box_width / 2 + 0.003)
            py = cy + ny * (self.field.box_width / 2 + 0.003)
            pts.extend(tuple(p) for p in rect_poly(px, py, self.field.box_length,
                                                   self.field.box_width, a + math.pi / 2))
        return _convex_hull(pts)

    def _publish_footprint(self):
        key = tuple(self.brain.faces)
        if key == self._footprint_key:
            return
        self._footprint_key = key
        # nav2_costmap_2d's footprint_sub_ is a plain geometry_msgs/Polygon,
        # not PolygonStamped (costmap_2d_ros.hpp) - that type mismatch meant
        # these were never actually being received at all, silently leaving
        # Nav2 on its default circular robot_radius_ footprint.
        poly = Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in self._footprint_polygon()])
        self.pub_footprint_local.publish(poly)
        self.pub_footprint_global.publish(poly)

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
        key = tuple(sorted(s.name for s in occ_stacks) + sorted(z.name for z in occ_zones))
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

    # ---------------------------------------------------------------- tick
    def _tick(self):
        if not self._got_time or not self._update_pose():
            self.pub_cmd.publish(Twist())
            return

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
            # the live LaserScan's angles already account for the mount
            # offset (base_scan's own TF frame/URDF yaw) - unlike the
            # headless/direct-array path, nothing extra to add here.
            lidar_yaw = 0.0
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
                          flip_busy=self._flip_in_flight, flip_result=flip_result)
        cmd = self.brain.step(obs, self.dt)
        # Published after step(), not before: a drop result processed this
        # tick updates belief inside step() - publishing beforehand meant
        # the costmap (and any Nav2 goal sent later this same tick) used
        # last tick's stale belief, occasionally planning a path straight
        # through the zone/stack that was *just* dropped into.
        self._publish_belief_objects(t)
        self._publish_footprint()
        self._publish_status(t, cmd)
        self._publish_opponent_marker()
        self._publish_goal(cmd)

        if cmd.goal_pose is not None:
            if not self._nav_matches(cmd.goal_pose):
                self._send_nav_goal(cmd.goal_pose)
        else:
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
