#!/usr/bin/env python3
"""ROS 2 node wrapping the 2027 table simulator.

Our robot (same interfaces as the real one):
  sub  /cmd_vel                 geometry_msgs/Twist  body frame (vx, vy, wz)
  pub  /odom                    nav_msgs/Odometry    odom -> base_footprint
  tf   odom -> base_footprint   (+ static map -> odom = identity)
  pub  /scan                    sensor_msgs/LaserScan  frame base_scan, 10 Hz,
                                only the opponent mast within ~2.8 m is seen
  pub  /camera/detections       table_sim_interfaces/msg/CameraDetections,
                                15 Hz, stacks/zones in the face-0 FOV cone
  pub  /joint_states            wheel joints at 0 (for robot_state_publisher)
  act  /face_tool               table_sim_interfaces/action/FaceTool (pick/drop)

Opponent:
  internal scripted behaviour (param `opponent`) or /opponent/cmd_vel when
  opponent:=teleop; ground truth on /opponent/odom and TF map -> opponent/base_footprint

Game:
  srv  /sim/start, /sim/reset, /game/start_match   std_srvs/Trigger
  pub  /game/match_active (Bool), /game/time_remaining (Float32),
       /game/score (Int32, ours), /sim/score (String, JSON both teams)

RViz:
  /sim/table    MarkerArray (latched): textured table, borders, zone outlines
  /sim/markers  MarkerArray: stacks, piles, held boxes, robots, scoreboard

The node publishes /clock (simulated time); run the other nodes with
use_sim_time:=true. `real_time_factor` > 1 runs faster than real time.
"""

import json
import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, Point32, Polygon, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, ColorRGBA, Float32, Int32, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from table_sim_interfaces.action import FaceTool
from table_sim_interfaces.msg import CameraDetections

from .field import Field, default_field_file
from .geometry import transform
from .opponents import BEHAVIORS, ScriptedOpponent
from .world import World

TEAM_RGB = {'yellow': (0.95, 0.75, 0.15), 'blue': (0.25, 0.45, 0.85)}
BOX_RGB = (0.79, 0.60, 0.36)


def _stamp(t):
    sec = int(math.floor(t))
    return TimeMsg(sec=sec, nanosec=int((t - sec) * 1e9))


def _quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _color(rgb, a=1.0):
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))


def _share_file(sub, name):
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('table_sim'), sub, name)
        if os.path.exists(p):
            return p
    except Exception:  # noqa: BLE001
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, '..', sub, name)


class TableSimNode(Node):
    def __init__(self):
        super().__init__('table_sim')
        dp = self.declare_parameter
        dp('field_file', '')
        dp('sim_file', '')
        dp('team', 'yellow')                 # our colour
        dp('opponent', '')                   # overrides sim.yaml opponent.behavior
        dp('opponent_avoid', True)
        dp('our_robot', 'cmd_vel')           # cmd_vel | scripted:<behavior>
        dp('rate', 50.0)
        dp('real_time_factor', 1.0)
        dp('auto_start', True)
        dp('auto_start_delay', 3.0)
        dp('map_frame', 'map')
        dp('odom_frame', 'odom')
        dp('base_frame', 'base_footprint')
        dp('scan_frame', 'base_scan')
        dp('wheel_joints', ['front_wheel_joint', 'left_rear_wheel_joint', 'right_rear_wheel_joint'])
        dp('robot_height', 0.35)
        dp('mast_height', 0.43)
        dp('pick_target_timeout_s', 20.0)
        # Must match odom_sim's own transform_tolerance_sec: odom_sim
        # future-dates the odom->base_footprint/map->odom TF it broadcasts
        # by this much (see its docstring), so a TF lookup for "now" never
        # extrapolates. /scan's geometry is computed from our robot's REAL
        # current pose, but a consumer (RViz, or anything else composing
        # scan points through the map->...->base_scan chain) resolves that
        # chain at /scan's OWN stamp - if that stamp is the real "now" while
        # the TF buffer only has future-dated samples, the resolved pose is
        # effectively ~0.2s stale relative to the one the scan was actually
        # computed from. While driving, that lag between "true current
        # heading used for the ray math" and "stale displayed sensor
        # orientation" makes the hit points appear to swim relative to a
        # perfectly static real-world object - confirmed by a live report
        # (points on the opponent's mast visibly moving while only OUR
        # robot moved). Stamping /scan with the same future offset makes a
        # TF lookup at the scan's own stamp resolve to the matching sample.
        dp('transform_tolerance_sec', 0.2)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.field = Field(g('field_file') or default_field_file())
        self._name_by_label = self.field.name_by_label()
        sim_file = g('sim_file') or _share_file('config', 'sim.yaml')
        with open(sim_file) as f:
            self.sim_cfg = yaml.safe_load(f)
        self.team = g('team')
        opp_cfg = self.sim_cfg.get('opponent', {}) or {}
        self.opp_behavior = g('opponent') or opp_cfg.get('behavior', 'mirror')
        if self.opp_behavior not in BEHAVIORS:
            raise ValueError(f'opponent must be one of {BEHAVIORS}')
        self.opp_avoid = bool(g('opponent_avoid'))
        self.our_mode = g('our_robot')
        self.rtf = float(g('real_time_factor'))
        self.dt = 1.0 / float(g('rate'))
        self.auto_start = bool(g('auto_start'))
        self.auto_delay = float(g('auto_start_delay'))
        self.frames = {k: g(k) for k in ('map_frame', 'odom_frame', 'base_frame', 'scan_frame')}
        self.robot_h = float(g('robot_height'))
        self.mast_h = float(g('mast_height'))
        self.wheels = list(g('wheel_joints'))
        self.pick_target_timeout_s = float(g('pick_target_timeout_s'))
        self.tf_tolerance = float(g('transform_tolerance_sec'))
        self._pick_target_us = None
        self._pick_target_set_t = None
        self._pick_target_acting = False
        self._custom_objects_key = None
        self._next_custom_objects = 0.0
        self._opponent_marker_key = None
        self._next_opponent_marker = 0.0

        self.lock = threading.RLock()
        self._build_world()

        # ---------------------------------------------------------- comms
        cb = ReentrantCallbackGroup()
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_true_odom = self.create_publisher(Odometry, '/sim/true_odom', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_camera = self.create_publisher(CameraDetections, '/camera/detections', 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_opp_odom = self.create_publisher(Odometry, '/opponent/odom', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/sim/markers', 10)
        self.pub_table = self.create_publisher(MarkerArray, '/sim/table', latched)
        self.pub_custom_objects = self.create_publisher(MarkerArray, '/custom_objects', latched)
        # Ground-truth opponent occupancy for Nav2's costmaps (nav2:=true,
        # see opponent_layer in nav2_params_sim.yaml), replacing an earlier
        # /scan-fed obstacle_layer that left a persistent trail: a single
        # raycast hit only clears cells ALONG that one ray, so any past
        # opponent position sitting in the current shadow (e.g. directly
        # behind the opponent as seen from our robot, which happens a lot
        # when the opponent is driven back and forth along roughly the same
        # bearing via teleop) never got swept - confirmed live, a long trail
        # of stale lethal blobs down the middle of the table. A snapshot
        # layer (DELETEALL + re-ADD every publish, see _publish_opponent_marker)
        # has no raycast/shadow concept at all: it only ever shows the
        # opponent where it IS right now, nothing more. Visibility-gated the
        # same way the real /scan would be (the mast lidar only ever sees
        # the opponent within lidar range, see CLAUDE.md), so this stays
        # exactly as sensor-realistic as the obstacle_layer it replaces -
        # the opponent simply vanishes from the costmap set when out of range.
        self.pub_opponent_marker = self.create_publisher(MarkerArray, '/opponent_marker', latched)
        # Bare (unloaded) footprint, so Nav2's costmaps always see the REAL
        # robot hexagon instead of their own default-circle fallback
        # whenever nothing else is publishing to this topic - confirmed
        # live: with only this sim running (no adaptive_strategy/
        # strategy_node, which normally owns this topic to also grow the
        # footprint for held boxes), Nav2 fell back to a small default
        # circle, well short of the real hexagon's vertices, so
        # motion_controller's mechanisms (which read this same footprint
        # via costmap_ros_->get[Unpadded]RobotFootprint()) were checking
        # collisions against a footprint smaller than the actual robot.
        # Publishing once at construction was NOT enough, even with this
        # publisher latched (TRANSIENT_LOCAL): Costmap2DROS's own
        # footprint_sub_ subscribes VOLATILE/BEST_EFFORT (confirmed live via
        # `ros2 topic info -v`), which only receives messages published
        # WHILE it's already connected - a transient-local publisher's
        # backlog only replays to transient-local readers, so a one-shot
        # publish before that subscription exists is silently lost forever,
        # with no error on either side. Republishing on a slow timer instead
        # guarantees the subscriber picks it up on whichever tick it happens
        # to already be connected for, regardless of startup ordering.
        # When strategy_node IS also running, it owns this topic instead
        # (its own loaded-aware footprint, republished on face/vertical
        # changes) - _publish_footprint below skips once it detects
        # another publisher, rather than fighting it every tick (previously
        # this timer's bare fallback kept clobbering strategy_node's
        # extended footprint a second later, flickering Nav2's footprint
        # extended/bare the whole time boxes were held).
        self.pub_footprint_local = self.create_publisher(Polygon, '/local_costmap/footprint', latched)
        self.pub_footprint_global = self.create_publisher(Polygon, '/global_costmap/footprint', latched)
        self._footprint_poly = Polygon(points=[
            Point32(x=float(x), y=float(y), z=0.0)
            for x, y in self.world.robots['us'].model.local_poly])
        self._publish_footprint()
        self.create_timer(1.0, self._publish_footprint)
        self.pub_active = self.create_publisher(Bool, '/game/match_active', 10)
        self.pub_left = self.create_publisher(Float32, '/game/time_remaining', 10)
        self.pub_score = self.create_publisher(Int32, '/game/score', 10)
        self.pub_score_json = self.create_publisher(String, '/sim/score', 10)
        self.pub_reset_event = self.create_publisher(Int32, '/sim/reset_event', 10)
        self._reset_count = 0
        self.tf = TransformBroadcaster(self)

        # Nav2's costmaps (nav2:=true) are owned by their own long-lived
        # nodes, so nothing clears their obstacle_layer across a /sim/reset -
        # a mark from a previous match (e.g. wherever the opponent last was)
        # otherwise sits there until some later scan happens to raytrace back
        # over it. Best-effort: these clients just never get a response if
        # nav2 isn't running (nav2:=false), which is fine.
        self._clear_costmap_clients = [
            self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'),
            self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'),
        ]

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10, callback_group=cb)
        self.create_subscription(Twist, '/opponent/cmd_vel', self._on_opp_cmd, 10, callback_group=cb)
        self.create_subscription(String, '/sim/pick_target_us', self._on_pick_target, 10, callback_group=cb)
        self.create_service(Trigger, '/sim/start', self._srv_start, callback_group=cb)
        self.create_service(Trigger, '/game/start_match', self._srv_start, callback_group=cb)
        self.create_service(Trigger, '/sim/reset', self._srv_reset, callback_group=cb)
        self.create_service(SetBool, '/sim/set_infinite_time', self._srv_infinite_time, callback_group=cb)
        self._action = ActionServer(self, FaceTool, '/face_tool', self._execute,
                                    goal_callback=lambda _g: GoalResponse.ACCEPT,
                                    cancel_callback=lambda _g: CancelResponse.ACCEPT,
                                    callback_group=cb)

        self._next_scan = 0.0
        self._next_camera = 0.0
        self._next_markers = 0.0
        self._next_table = 0.0
        self.create_timer(self.dt, self._tick, callback_group=self.timer_cb)
        self.get_logger().info(
            f'table_sim ready: we are {self.team} ({self.our_mode}), opponent "{self.opp_behavior}", '
            f'rtf {self.rtf}. Start: ros2 service call /sim/start std_srvs/srv/Trigger'
            + (f' (auto start in {self.auto_delay:.0f}s)' if self.auto_start else ''))

    # ------------------------------------------------------------- world
    def _build_world(self):
        # Keep the sim clock monotonic across /sim/reset: a brand new World
        # starts its own .time at 0, which would make /clock (and every
        # stamped message/TF) jump backward on reset. tf2 correctly reacts
        # to a backward jump by rejecting "old" data for a long stretch
        # afterward, which silently breaks TF lookups (goal_driver's pose,
        # Nav2's costmaps) well past the reset itself. Only match/game state
        # should reset, not the global clock.
        prev_time = self.world.time if hasattr(self, 'world') else 0.0
        prev_infinite_time = self.world.infinite_time if hasattr(self, 'world') else False
        self.world = World(self.field, self.sim_cfg, team_us=self.team,
                           seed=int((self.sim_cfg.get('opponent') or {}).get('seed', 0)))
        self.world.time = prev_time
        self.world.infinite_time = prev_infinite_time
        speed = float((self.sim_cfg.get('opponent') or {}).get('speed_scale', 1.0))
        self.opp = ScriptedOpponent(self.world, self.opp_behavior, self.opp_avoid, speed)
        self.us_script = None
        if self.our_mode.startswith('scripted'):
            beh = self.our_mode.split(':', 1)[1] if ':' in self.our_mode else 'mirror'
            self.us_script = ScriptedOpponent(self.world, beh, True, 1.0, name='us')
        self.t_built = self.world.time

    # ---------------------------------------------------------- callbacks
    def _on_cmd(self, msg: Twist):
        if self.us_script is None:
            with self.lock:
                self.world.set_cmd('us', msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_opp_cmd(self, msg: Twist):
        if self.opp_behavior == 'teleop':
            with self.lock:
                self.world.set_cmd('opp', msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_pick_target(self, msg: String):
        name = msg.data.strip()
        with self.lock:
            if not name:
                self._pick_target_us = None
                self._pick_target_acting = False
                return
            name = self._name_by_label.get(name, name)
            self._pick_target_us = name
            self._pick_target_set_t = self.world.time
            self._pick_target_acting = False
            self.get_logger().info(f'pick target set: {name} (excluded from /custom_objects)')

    def _srv_start(self, _req, resp):
        with self.lock:
            self.world.start_match()
        resp.success, resp.message = True, 'match started'
        return resp

    def _srv_reset(self, _req, resp):
        with self.lock:
            self._build_world()
            self._next_scan = self._next_camera = self._next_markers = self._next_table = 0.0
            self._pick_target_us = None
            self._pick_target_set_t = None
            self._pick_target_acting = False
            self._custom_objects_key = None
            self._next_custom_objects = 0.0
            self._opponent_marker_key = None
            self._next_opponent_marker = 0.0
            self._reset_count += 1
            self.pub_reset_event.publish(Int32(data=self._reset_count))
        for client in self._clear_costmap_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
        resp.success, resp.message = True, 'world reset (call /sim/start)'
        return resp

    def _srv_infinite_time(self, req, resp):
        with self.lock:
            self.world.infinite_time = bool(req.data)
        resp.success = True
        resp.message = f'infinite_time {"on" if req.data else "off"}'
        return resp

    def _execute(self, goal_handle):
        g = goal_handle.request
        result = FaceTool.Result()
        if self.us_script is not None:
            goal_handle.abort()
            result.message = 'our robot is scripted (our_robot param)'
            return result
        # FLIP runs in its own concurrent slot (world.py) so it can overlap a
        # PICK/DROP on another face - each goal must only ever touch its own
        # slot's result/progress, never the other in-flight goal's.
        is_flip = int(g.mode) == FaceTool.Goal.FLIP
        with self.lock:
            self.world.pop_result('us', flip=is_flip)
            ok, why = self.world.request_action('us', int(g.face), int(g.mode), int(g.count))
        if not ok:
            goal_handle.abort()
            result.success, result.message = False, why
            return result
        fb = FaceTool.Feedback()
        while rclpy.ok():
            with self.lock:
                if goal_handle.is_cancel_requested:
                    self.world.cancel_action('us', flip=is_flip)
                    res = self.world.pop_result('us', flip=is_flip)
                    goal_handle.canceled()
                    result.success, result.count = False, int(res.count) if res else 0
                    result.message = 'canceled'
                    return result
                res = self.world.pop_result('us', flip=is_flip)
                prog = self.world.action_progress('us', flip=is_flip)
            if res is not None:
                result.success, result.count, result.message = bool(res.success), int(res.count), res.message
                if res.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
            if prog is not None:
                fb.phase, fb.progress = prog[0], float(prog[1])
                goal_handle.publish_feedback(fb)
            time.sleep(0.02)
        goal_handle.abort()
        return result

    # --------------------------------------------------------------- tick
    def _tick(self):
        with self.lock:
            w = self.world
            h = self.dt * self.rtf
            if self.auto_start and w.match_t0 is None and w.time - self.t_built >= self.auto_delay:
                w.start_match()
                self.get_logger().info('match started')
            self.opp.step(h)
            if self.us_script is not None:
                self.us_script.step(h)
            was_finished = w.finished
            w.step(h)
            if w.finished and not was_finished:
                self.get_logger().info(f'match over: {json.dumps(w.final_score)}')
            now = w.time
            if self._pick_target_us is not None:
                us = w.robots['us']
                if us.action is not None:
                    self._pick_target_acting = True
                elif self._pick_target_acting:
                    self._pick_target_us = None
                    self._pick_target_acting = False
                if (self._pick_target_us is not None
                        and now - self._pick_target_set_t > self.pick_target_timeout_s):
                    self.get_logger().info(f'pick target {self._pick_target_us} timed out, clearing')
                    self._pick_target_us = None
                    self._pick_target_acting = False
        # Publishing only reads world state to build messages - kept outside
        # the lock so it doesn't hold it for the whole 10ms tick (that was
        # starving /cmd_vel's callback thread under GIL contention and
        # pegging a core whenever something actually drove "us").
        stamp = _stamp(now)
        self.pub_clock.publish(Clock(clock=stamp))
        self._publish_robot(stamp)
        if now >= self._next_scan:
            self._next_scan = now + 1.0 / float(self.sim_cfg.get('lidar', {}).get('rate', 10.0))
            self._publish_scan(stamp, now)
        if now >= self._next_camera:
            self._next_camera = now + 1.0 / float(self.sim_cfg.get('camera', {}).get('rate', 15.0))
            self._publish_camera()
        if now >= self._next_markers:
            self._next_markers = now + 0.1
            self._publish_markers(stamp)
            self._publish_game()
        self._publish_custom_objects(stamp, now)
        if now >= self._next_table:
            self._next_table = now + 5.0
            self._publish_table(stamp)

    # -------------------------------------------------------- publishers
    def _odom_msg(self, r, stamp, frame, child):
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.child_frame_id = child
        m.pose.pose.position.x = float(r.x)
        m.pose.pose.position.y = float(r.y)
        q = _quat(r.th)
        m.pose.pose.orientation.x, m.pose.pose.orientation.y = q[0], q[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = q[2], q[3]
        c, s = math.cos(r.th), math.sin(r.th)
        m.twist.twist.linear.x = float(c * r.vx + s * r.vy)
        m.twist.twist.linear.y = float(-s * r.vx + c * r.vy)
        m.twist.twist.angular.z = float(r.w)
        for i in (0, 7, 14, 21, 28, 35):
            m.pose.covariance[i] = 1e-6
            m.twist.covariance[i] = 1e-6
        return m

    def _tf(self, stamp, parent, child, x, y, th):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        q = _quat(th)
        t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        return t

    def _publish_robot(self, stamp):
        f = self.frames
        us = self.world.robots['us']
        opp = self.world.robots['opp']
        self.pub_true_odom.publish(self._odom_msg(us, stamp, f['map_frame'], f['base_frame']))
        self.pub_opp_odom.publish(self._odom_msg(opp, stamp, f['map_frame'], 'opponent/base_footprint'))
        tfs = [self._tf(stamp, f['map_frame'], 'opponent/base_footprint', opp.x, opp.y, opp.th)]
        self.tf.sendTransform(tfs)
        js = JointState()
        js.header.stamp = stamp
        js.name = self.wheels
        js.position = [0.0] * len(self.wheels)
        self.pub_joints.publish(js)

    def _publish_scan(self, stamp, now):
        ranges, a_min, inc = self.world.scan('us')
        L = self.sim_cfg.get('lidar', {})
        m = LaserScan()
        # Future-dated to match odom_sim's TF (see transform_tolerance_sec
        # above) - NOT `stamp`, which is the real "now" the rays were
        # actually computed from.
        m.header.stamp = _stamp(now + self.tf_tolerance)
        m.header.frame_id = self.frames['scan_frame']
        m.angle_min = float(a_min)
        m.angle_increment = float(inc)
        m.angle_max = float(a_min + inc * (len(ranges) - 1))
        m.scan_time = 1.0 / float(L.get('rate', 10.0))
        m.range_min = 0.05
        m.range_max = float(L.get('range_max', 12.0))
        m.ranges = [float(r) for r in ranges]
        self.pub_scan.publish(m)
        self._publish_opponent_marker(stamp, now, visible=bool(np.isfinite(ranges).any()))

    def _publish_footprint(self):
        # Skip once someone else (strategy_node, which republishes its own
        # loaded-aware footprint only on an actual face/vertical change,
        # not on a timer) is also publishing here - otherwise this timer's
        # bare fallback wins every tick regardless of what strategy_node
        # just set, flickering the Nav2 footprint extended/bare/extended
        # every ~1s while boxes are held. count_publishers includes this
        # node's own publisher, hence > 1 (not > 0) means "someone else too".
        if self.count_publishers('/local_costmap/footprint') > 1:
            return
        self.pub_footprint_local.publish(self._footprint_poly)
        self.pub_footprint_global.publish(self._footprint_poly)

    def _publish_opponent_marker(self, stamp, now, visible):
        """Ground-truth opponent occupancy for Nav2 (see pub_opponent_marker
        above): DELETEALL + a single mast-sized box at the opponent's actual
        current pose when visible to our own simulated lidar this tick, or
        just the DELETEALL alone when not - no stale state in between, unlike
        a raycast layer's lingering marks. Mast-sized (sim.yaml's
        mast_size), matching what the lidar itself can actually see (CLAUDE.md:
        only the opponent's mast, not its full body), not the whole opponent.

        Gated like _publish_custom_objects (only on actual change, else a
        1s safety-net resync) - NOT just rate-limited. CustomObjectsLayer's
        markersCallback sets current_=false on every message regardless of
        content, and updateBounds only reports a touched region when
        objects_/prev_objects_ are non-empty - while the opponent stays out
        of range (both permanently empty), repeatedly publishing an
        all-DELETEALL array kept re-arming current_=false with no non-empty
        bounds ever reported to earn it back via updateCosts. Confirmed
        live: goals failed 100% of the time with the opponent out of range,
        worked every time with it in range (which has real bounds each
        update) - a flat rate limit alone (still unconditionally
        republishing empty content) did not fix it; skipping entirely on
        no-op does.
        """
        opp = self.world.robots['opp']
        key = (visible, round(opp.x, 2), round(opp.y, 2)) if visible else (False,)
        if key == self._opponent_marker_key and now < self._next_opponent_marker:
            return
        self._opponent_marker_key = key
        self._next_opponent_marker = now + 1.0
        # obstacle size for Nav2: sim.yaml lidar.opponent_obstacle_m (Max: the bare mast let our robot
        # brush past the opponent's body); default the mast itself
        mast = float((self.sim_cfg.get('lidar') or {}).get('opponent_obstacle_m', opp.model.mast))
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        if visible:
            arr.markers.append(self._box('objects', 0, stamp, opp.x, opp.y, mast / 2, opp.th,
                                         mast, mast, mast, (0.9, 0.3, 0.3), 0.8))
        self.pub_opponent_marker.publish(arr)

    def _publish_camera(self):
        view = self.world.camera_view('us')
        m = CameraDetections()
        m.names = list(view.keys())
        m.counts = [int(view[n]) for n in m.names]
        self.pub_camera.publish(m)

    def _publish_game(self):
        w, f = self.world, self.field
        self.pub_active.publish(Bool(data=bool(w.running)))
        self.pub_left.publish(Float32(data=float(w.time_left)))
        sc = {t: w.score(t) for t in ('yellow', 'blue')}
        self.pub_score.publish(Int32(data=int(sc[self.team]['total'])))
        robots = {}
        for name, r in w.robots.items():
            ph = w.action_progress(name)
            fph = w.action_progress(name, flip=True)
            action = f'{ph[0]} {int(ph[1] * 100)}%' if ph else ''
            if fph:
                flip_str = f'flip(f{r.flip_action.face}) {fph[0]} {int(fph[1] * 100)}%'
                action = f'{action} + {flip_str}' if action else flip_str
            robots[name] = {
                'team': r.team, 'x': round(r.x, 3), 'y': round(r.y, 3),
                'th_deg': round(math.degrees(r.th), 1), 'faces': list(r.faces),
                'vertical': list(r.vertical),
                'door': 'built' if r.door_used else (f.label(r.door_zone) if r.door_zone else ''),
                'action': action,
                'contacts': r.contacts,
            }
        robots['opp']['behavior'] = self.opp_behavior
        robots['opp']['status'] = self.opp.status
        if self.us_script is not None:
            robots['us']['status'] = self.us_script.status
        self.pub_score_json.publish(String(data=json.dumps(
            {'time': round(w.match_time, 2), 'time_left': round(w.time_left, 2),
             'infinite_time': w.infinite_time,
             'started': w.match_t0 is not None, 'running': w.running, 'finished': w.finished,
             'team': self.team, 'score': sc, 'robots': robots,
             'stacks': {f.label(k): v for k, v in w.stacks.items()},
             'zones': {f.label(k): v for k, v in w.zones.items()},
             'zone_capacities': {f.label(k): f.zone(k).capacity for k in w.zones},
             'extra': {k: [round(v[0], 3), round(v[1], 3), round(v[2], 4), v[3], f.label(v[4]), bool(v[5])]
                       for k, v in w.extra_piles.items()},
             'pile_poses': {k: [round(v[0], 3), round(v[1], 3), round(v[2], 4)] for k, v in w.pile_pose.items()},
             'events': [f'{t:5.1f}  {m}' for t, m in w.events[-15:]]})))

    # ------------------------------------------------------------ markers
    def _mk(self, ns, mid, mtype, stamp, frame=None):
        m = Marker()
        m.header.frame_id = frame or self.frames['map_frame']
        m.header.stamp = stamp
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _box(self, ns, mid, stamp, x, y, z, th, sx, sy, sz, rgb, a=1.0):
        m = self._mk(ns, mid, Marker.CUBE, stamp)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        q = _quat(th)
        m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color = _color(rgb, a)
        return m

    def _text(self, ns, mid, stamp, x, y, z, text, size=0.06, rgb=(1, 1, 1)):
        m = self._mk(ns, mid, Marker.TEXT_VIEW_FACING, stamp)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        # RViz only uses scale.z for text, but a zero x/y scale breaks the
        # glyph spacing (huge gaps at spaces) on some Ogre versions
        m.scale.x = m.scale.y = m.scale.z = float(size)
        m.color = _color(rgb)
        m.text = text
        return m

    def _outline(self, ns, mid, stamp, poly, rgb, z=0.003, width=0.012):
        m = self._mk(ns, mid, Marker.LINE_STRIP, stamp)
        m.scale.x = width
        m.color = _color(rgb)
        for p in list(poly) + [poly[0]]:
            m.points.append(Point(x=float(p[0]), y=float(p[1]), z=z))
        return m

    def _publish_table(self, stamp):
        f = self.field
        arr = MarkerArray()
        floor = self._mk('floor', 0, Marker.MESH_RESOURCE, stamp)
        floor.mesh_resource = 'package://table_sim/meshes/table_2027.dae'
        floor.mesh_use_embedded_materials = True
        floor.pose.position.z = -0.001
        floor.scale.x = floor.scale.y = floor.scale.z = 1.0
        floor.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
        arr.markers.append(floor)
        t, h = 0.022, 0.07
        L, W = f.length, f.width
        for i, (x, y, sx, sy) in enumerate(((L / 2, -t / 2, L + 2 * t, t), (L / 2, W + t / 2, L + 2 * t, t),
                                            (-t / 2, W / 2, t, W), (L + t / 2, W / 2, t, W))):
            arr.markers.append(self._box('borders', i, stamp, x, y, h / 2, 0.0, sx, sy, h, (0.55, 0.55, 0.55)))
        for i, z in enumerate(f.zones):
            arr.markers.append(self._outline('zones', i, stamp, z.poly, TEAM_RGB[z.team]))
            arr.markers.append(self._text('zone_ids', i, stamp, z.x, z.y, 0.02, z.label, 0.08, (1, 1, 1)))
        for i, s in enumerate(f.stacks):
            arr.markers.append(self._outline('pick_zones', i, stamp, s.poly, (0.9, 0.9, 0.9), width=0.006))
            # ID next to the stack, on the side the robot comes from
            a = s.approach[0]
            arr.markers.append(self._text('pick_ids', i, stamp, s.x + 0.13 * math.cos(a),
                                          s.y + 0.13 * math.sin(a), 0.02, s.label, 0.08, (1, 1, 1)))
        for i, (team, z) in enumerate(f.start_zones.items()):
            arr.markers.append(self._outline('start_zones', i, stamp, z.poly, TEAM_RGB[team], width=0.02))
        for i, (team, poly) in enumerate(f.castle.items()):
            if len(poly) > 0:
                arr.markers.append(self._outline('castle', i, stamp, poly, TEAM_RGB[team],
                                                  z=0.001, width=0.01))
        self.pub_table.publish(arr)

    def _publish_custom_objects(self, stamp, now):
        """Oriented-box obstacles for nav2_costmap_2d's CustomObjectsLayer, one
        per occupied stack/pile. Always a full DELETEALL + re-ADD snapshot (no
        incremental deletes from a second publisher) so a late-joining or
        restarted costmap layer can never replay a stale ADD after a delete -
        every publish is a complete, self-healing resync.
        The active pick/drop target (self._pick_target_us) is left out so the
        robot can actually reach it; every other stack/pile stays an obstacle.

        Only actually publishes when the occupied set changes, or at most
        once a second otherwise (safety-net resync). CustomObjectsLayer marks
        itself "not current" on every received message regardless of content,
        and nav2's planner/controller abort with "costmap timed out waiting
        for update" if the costmap can't go "current" for a few seconds in a
        row - publishing a no-op snapshot every ~100ms (faster than the
        costmap's own 5 Hz update_frequency) starved it permanently. Gating on
        actual change keeps updates near-instant on a pick/drop while leaving
        the costmap long idle windows to settle "current" the rest of the time.
        """
        f, w = self.field, self.world
        skip = self._pick_target_us
        included = frozenset(
            [s.name for s in f.stacks if s.name != skip and w.stacks[s.name] > 0]
            + [z.name for z in f.zones if z.name != skip and w.zones[z.name] > 0])
        if included == self._custom_objects_key and now < self._next_custom_objects:
            return
        self._custom_objects_key = included
        self._next_custom_objects = now + 1.0

        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        mid = 0
        for s in f.stacks:
            if s.name not in included:
                continue
            arr.markers.append(self._box('objects', mid, stamp, s.x, s.y, f.box_height / 2, s.th,
                                         f.box_length, f.box_width, f.box_height, (0.9, 0.3, 0.3), 0.5))
            mid += 1
        for z in f.zones:
            if z.name not in included:
                continue
            fp = (f.box_width, f.box_width) if z.shape == 'circle' else (f.box_length, f.box_width)
            # Real pose of where the pile actually landed, not always the
            # zone's idealised preset centre - same reasoning as the
            # /sim/markers 'piles' rendering below.
            zx, zy, zth = w.pile_pose.get(z.name, (z.x, z.y, z.th))
            arr.markers.append(self._box('objects', mid, stamp, zx, zy, f.box_height / 2, zth,
                                         fp[0], fp[1], f.box_height, (0.9, 0.3, 0.3), 0.5))
            mid += 1
        self.pub_custom_objects.publish(arr)

    def _robot_prism(self, ns, mid, stamp, r, rgb):
        m = self._mk(ns, mid, Marker.TRIANGLE_LIST, stamp)
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = _color(rgb, 0.9)
        v = transform(r.model.local_poly, r.x, r.y, r.th)
        n = len(v)
        z0, z1 = 0.0, self.robot_h
        P = lambda p, z: Point(x=float(p[0]), y=float(p[1]), z=float(z))  # noqa: E731
        # Fan-triangulate the top cap and walk all n side walls - was
        # hardcoded to n=3 (a bare triangle), so for the hexagon footprint
        # (n=6) the top cap only covered half the shape and the last "wall"
        # connected v[2] straight to v[0], a diagonal chord across the
        # hexagon instead of its actual short edge - confirmed as exactly
        # the malformed face Max saw live. Works for any convex n-gon now.
        tris = [(P(v[0], z1), P(v[i], z1), P(v[i + 1], z1)) for i in range(1, n - 1)]
        for k in range(n):
            a, b = v[k], v[(k + 1) % n]
            tris += [(P(a, z0), P(b, z0), P(b, z1)), (P(a, z0), P(b, z1), P(a, z1))]
        for tri in tris:
            m.points.extend(tri)
        return m

    def _publish_markers(self, stamp):
        w, f = self.world, self.field
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        bl, bw, bh = f.box_length, f.box_width, f.box_height
        mid = 0
        # stacks and piles: one cube per box
        for s in f.stacks:
            if s.steal_zone:
                continue
            n = w.stacks[s.name]
            for k in range(n):
                arr.markers.append(self._box('stacks', mid, stamp, s.x, s.y, bh / 2 + k * bh, s.th,
                                             bl, bw, bh * 0.97, BOX_RGB))
                mid += 1
            arr.markers.append(self._text('stack_count', mid, stamp, s.x, s.y, n * bh + 0.06,
                                          f'{s.label}: {n}', 0.06))
            mid += 1
        # stacks put down on the bare floor by a regrip (DROP_FLOOR)
        for name, (fx, fy, fth, n) in w.floor_piles.items():
            for k in range(n):
                arr.markers.append(self._box('floor_piles', mid, stamp, fx, fy, bh / 2 + k * bh, fth,
                                             bl, bw, bh * 0.97, BOX_RGB))
                mid += 1
            arr.markers.append(self._text('floor_count', mid, stamp, fx, fy, n * bh + 0.06,
                                          f'{name}: {n}', 0.06))
            mid += 1
        # over-picked stacks discarded next to a built wall (extras of that zone)
        extras = {}
        for name, (ex, ey, eth, n, ez, ev) in w.extra_piles.items():
            extras[ez] = extras.get(ez, 0) + n
            for k in range(n):
                if ev:        # a standing box, as it stood on the robot's face
                    arr.markers.append(self._box('extra_piles', mid, stamp, ex, ey, bl / 2, eth,
                                                 bw, bw, bl * 0.97, BOX_RGB))
                else:
                    arr.markers.append(self._box('extra_piles', mid, stamp, ex, ey, bh / 2 + k * bh, eth,
                                                 bl, bw, bh * 0.97, BOX_RGB))
                mid += 1
            arr.markers.append(self._text('extra_count', mid, stamp, ex, ey, n * bh + 0.06,
                                          f'{f.label(ez)} extra: {n}', 0.06, TEAM_RGB[f.zone(ez).team]))
            mid += 1
        for z in f.zones:
            n = w.zones[z.name]
            vcount = w.zone_vertical[z.name]
            # Real pose of where this pile was actually dropped (plain
            # horizontal case only - flip/door is v2 scope), falling back
            # to the zone's own preset centre until a drop has happened.
            zx, zy, zth = w.pile_pose.get(z.name, (z.x, z.y, z.th))
            if z.shape == 'circle':
                # a flipped box stands on its bw x bw end, height bl
                for k in range(n):        # at the real drop pose: as the box stood on the robot's face
                    arr.markers.append(self._box('piles', mid, stamp, zx, zy, bl / 2, zth,
                                                 bw, bw, bl * 0.97, BOX_RGB))
                    mid += 1
                top = bl
            elif vcount > 0:
                # door: both posts dropped together, side by side along the
                # zone's long axis (not toward/away from the robot - that
                # was the earlier bug), centred on the zone exactly like any
                # other drop, + the horizontal lid on top once present.
                tangent = np.array([math.cos(z.th), math.sin(z.th)])
                for k, off in enumerate((-0.6, 0.6)):
                    if k >= vcount:
                        break
                    bp = np.array([z.x, z.y]) + tangent * off * bw
                    arr.markers.append(self._box('piles', mid, stamp, bp[0], bp[1], bl / 2, z.th,
                                                 bw, bw, bl * 0.97, BOX_RGB))
                    mid += 1
                if n > vcount:
                    arr.markers.append(self._box('piles', mid, stamp, z.x, z.y, bl + bh / 2, z.th,
                                                 bl, bw, bh * 0.97, BOX_RGB))
                    mid += 1
                top = bl + bh
            else:
                for k in range(n - extras.get(z.name, 0)):
                    arr.markers.append(self._box('piles', mid, stamp, zx, zy, bh / 2 + k * bh, zth,
                                                 bl, bw, bh * 0.97, BOX_RGB))
                    mid += 1
                top = n * bh
            if n:
                arr.markers.append(self._text('pile_count', mid, stamp, zx, zy, top + 0.06,
                                              f'{z.label}: {n}/{z.capacity}', 0.06, TEAM_RGB[z.team]))
                mid += 1
        # robots, held boxes, masts
        for name, r in w.robots.items():
            rgb = TEAM_RGB[r.team]
            arr.markers.append(self._robot_prism('robots', mid, stamp, r, rgb))
            mid += 1
            mast = self._mk('robots', mid, Marker.CYLINDER, stamp)
            mast.pose.position.x, mast.pose.position.y = float(r.x), float(r.y)
            mast.pose.position.z = self.mast_h / 2
            mast.scale.x = mast.scale.y = r.model.mast
            mast.scale.z = self.mast_h
            mast.color = _color((0.15, 0.15, 0.15))
            arr.markers.append(mast)
            mid += 1
            for fi, n in enumerate(r.faces):
                c, nv = w.face_frame(r, fi)
                p = c + nv * (bw / 2 + 0.003)
                yaw = math.atan2(nv[1], nv[0]) + math.pi / 2
                if r.vertical[fi]:
                    # flipped: boxes stand on their bw x bw end (height bl),
                    # in their own fixed slot (left/centre/right, each bw
                    # apart - world.py's held_poly_slot) - never recentred
                    # as neighbouring slots get dropped.
                    tangent = np.array([-nv[1], nv[0]])
                    for slot in range(3):
                        if not r.vertical_slots[fi][slot]:
                            continue
                        bp = p + tangent * (slot - 1) * bw
                        arr.markers.append(self._box('held', mid, stamp, bp[0], bp[1], 0.03 + bl / 2,
                                                     yaw, bw, bw, bl * 0.97, BOX_RGB))
                        mid += 1
                else:
                    for k in range(n):
                        arr.markers.append(self._box('held', mid, stamp, p[0], p[1], 0.03 + bh / 2 + k * bh,
                                                     yaw, bl, bw, bh * 0.97, BOX_RGB))
                        mid += 1
                # face numbers, so you can check which face is which
                lp = c - nv * 0.04
                arr.markers.append(self._text('face_ids', mid, stamp, lp[0], lp[1], self.robot_h + 0.02,
                                              str(fi), 0.05, (0, 0, 0)))
                mid += 1
            ph = w.action_progress(name)
            label = 'US' if name == 'us' else f'OPP ({self.opp_behavior})'
            if ph:
                label += f'\n{ph[0]} {int(ph[1] * 100)}%'
            arr.markers.append(self._text('labels', mid, stamp, r.x, r.y, self.mast_h + 0.12, label, 0.06))
            mid += 1
        # lidar detection range around us
        us = w.robots['us']
        circ = self._mk('lidar_range', mid, Marker.LINE_STRIP, stamp)
        circ.scale.x = 0.006
        circ.color = _color((1.0, 0.2, 0.2), 0.6)
        R = float(self.sim_cfg.get('lidar', {}).get('detect_range', 1.0))
        for k in range(73):
            a = 2 * math.pi * k / 72
            circ.points.append(Point(x=float(us.x + R * math.cos(a)), y=float(us.y + R * math.sin(a)),
                                     z=self.mast_h))
        arr.markers.append(circ)
        mid += 1
        # scoreboard
        sy, sb = w.score('yellow'), w.score('blue')
        status = 'waiting for start' if w.match_t0 is None else ('MATCH OVER' if w.finished else 'running')
        txt = (f"YELLOW {sy['total']}  :  {sb['total']} BLUE      t = {w.match_time:4.1f} s "
               f"(left {w.time_left:4.1f})   {status}")
        arr.markers.append(self._text('scoreboard', mid, stamp, f.length / 2, f.width + 0.2, 0.25, txt, 0.09))
        self.pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = TableSimNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


__all__ = ['TableSimNode', 'main', 'np']
