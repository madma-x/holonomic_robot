#!/usr/bin/env python3
"""ROS 2 node wrapping the 2027 table simulator.

Our robot (same interfaces as the real one):
  sub  /cmd_vel                 geometry_msgs/Twist  body frame (vx, vy, wz)
  pub  /odom                    nav_msgs/Odometry    odom -> base_footprint
  tf   odom -> base_footprint   (+ static map -> odom = identity)
  pub  /scan                    sensor_msgs/LaserScan  frame base_scan, 10 Hz,
                                only the opponent mast within 1 m is seen
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
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, ColorRGBA, Float32, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from table_sim_interfaces.action import FaceTool

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
        dp('publish_odom_tf', True)
        dp('publish_map_odom', True)
        dp('wheel_joints', ['front_wheel_joint', 'left_rear_wheel_joint', 'right_rear_wheel_joint'])
        dp('robot_height', 0.35)
        dp('mast_height', 0.43)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.field = Field(g('field_file') or default_field_file())
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

        self.lock = threading.RLock()
        self._build_world()

        # ---------------------------------------------------------- comms
        cb = ReentrantCallbackGroup()
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_opp_odom = self.create_publisher(Odometry, '/opponent/odom', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/sim/markers', 10)
        self.pub_table = self.create_publisher(MarkerArray, '/sim/table', latched)
        self.pub_active = self.create_publisher(Bool, '/game/match_active', 10)
        self.pub_left = self.create_publisher(Float32, '/game/time_remaining', 10)
        self.pub_score = self.create_publisher(Int32, '/game/score', 10)
        self.pub_score_json = self.create_publisher(String, '/sim/score', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10, callback_group=cb)
        self.create_subscription(Twist, '/opponent/cmd_vel', self._on_opp_cmd, 10, callback_group=cb)
        self.create_service(Trigger, '/sim/start', self._srv_start, callback_group=cb)
        self.create_service(Trigger, '/game/start_match', self._srv_start, callback_group=cb)
        self.create_service(Trigger, '/sim/reset', self._srv_reset, callback_group=cb)
        self._action = ActionServer(self, FaceTool, '/face_tool', self._execute,
                                    goal_callback=lambda _g: GoalResponse.ACCEPT,
                                    cancel_callback=lambda _g: CancelResponse.ACCEPT,
                                    callback_group=cb)

        if g('publish_map_odom'):
            t = TransformStamped()
            t.header.stamp = _stamp(0.0)
            t.header.frame_id = self.frames['map_frame']
            t.child_frame_id = self.frames['odom_frame']
            t.transform.rotation.w = 1.0
            self.static_tf.sendTransform(t)
        self.publish_odom_tf = bool(g('publish_odom_tf'))

        self._next_scan = 0.0
        self._next_markers = 0.0
        self._next_table = 0.0
        self.create_timer(self.dt, self._tick, callback_group=self.timer_cb)
        self.get_logger().info(
            f'table_sim ready: we are {self.team} ({self.our_mode}), opponent "{self.opp_behavior}", '
            f'rtf {self.rtf}. Start: ros2 service call /sim/start std_srvs/srv/Trigger'
            + (f' (auto start in {self.auto_delay:.0f}s)' if self.auto_start else ''))

    # ------------------------------------------------------------- world
    def _build_world(self):
        self.world = World(self.field, self.sim_cfg, team_us=self.team,
                           seed=int((self.sim_cfg.get('opponent') or {}).get('seed', 0)))
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

    def _srv_start(self, _req, resp):
        with self.lock:
            self.world.start_match()
        resp.success, resp.message = True, 'match started'
        return resp

    def _srv_reset(self, _req, resp):
        with self.lock:
            self._build_world()
            self._next_scan = self._next_markers = self._next_table = 0.0
        resp.success, resp.message = True, 'world reset (call /sim/start)'
        return resp

    def _execute(self, goal_handle):
        g = goal_handle.request
        result = FaceTool.Result()
        if self.us_script is not None:
            goal_handle.abort()
            result.message = 'our robot is scripted (our_robot param)'
            return result
        with self.lock:
            self.world.pop_result('us')
            ok, why = self.world.request_action('us', int(g.face), int(g.mode))
        if not ok:
            goal_handle.abort()
            result.success, result.message = False, why
            return result
        fb = FaceTool.Feedback()
        while rclpy.ok():
            with self.lock:
                if goal_handle.is_cancel_requested:
                    self.world.cancel_action('us')
                    res = self.world.pop_result('us')
                    goal_handle.canceled()
                    result.success, result.count = False, int(res.count) if res else 0
                    result.message = 'canceled'
                    return result
                res = self.world.pop_result('us')
                prog = self.world.action_progress('us')
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
            stamp = _stamp(now)
            self.pub_clock.publish(Clock(clock=stamp))
            self._publish_robot(stamp)
            if now >= self._next_scan:
                self._next_scan = now + 1.0 / float(self.sim_cfg.get('lidar', {}).get('rate', 10.0))
                self._publish_scan(stamp)
            if now >= self._next_markers:
                self._next_markers = now + 0.1
                self._publish_markers(stamp)
                self._publish_game()
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
        self.pub_odom.publish(self._odom_msg(us, stamp, f['odom_frame'], f['base_frame']))
        self.pub_opp_odom.publish(self._odom_msg(opp, stamp, f['map_frame'], 'opponent/base_footprint'))
        tfs = [self._tf(stamp, f['map_frame'], 'opponent/base_footprint', opp.x, opp.y, opp.th)]
        if self.publish_odom_tf:
            tfs.append(self._tf(stamp, f['odom_frame'], f['base_frame'], us.x, us.y, us.th))
        self.tf.sendTransform(tfs)
        js = JointState()
        js.header.stamp = stamp
        js.name = self.wheels
        js.position = [0.0] * len(self.wheels)
        self.pub_joints.publish(js)

    def _publish_scan(self, stamp):
        ranges, a_min, inc = self.world.scan('us')
        L = self.sim_cfg.get('lidar', {})
        m = LaserScan()
        m.header.stamp = stamp
        m.header.frame_id = self.frames['scan_frame']
        m.angle_min = float(a_min)
        m.angle_increment = float(inc)
        m.angle_max = float(a_min + inc * (len(ranges) - 1))
        m.scan_time = 1.0 / float(L.get('rate', 10.0))
        m.range_min = 0.05
        m.range_max = float(L.get('range_max', 12.0))
        m.ranges = [float(r) for r in ranges]
        self.pub_scan.publish(m)

    def _publish_game(self):
        w, f = self.world, self.field
        self.pub_active.publish(Bool(data=bool(w.running)))
        self.pub_left.publish(Float32(data=float(w.time_left)))
        sc = {t: w.score(t) for t in ('yellow', 'blue')}
        self.pub_score.publish(Int32(data=int(sc[self.team]['total'])))
        self.pub_score_json.publish(String(data=json.dumps(
            {'time': round(w.match_time, 2), 'running': w.running, 'score': sc,
             'stacks': {f.label(k): v for k, v in w.stacks.items()},
             'zones': {f.label(k): v for k, v in w.zones.items()}})))

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
        m.scale.z = float(size)
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
        self.pub_table.publish(arr)

    def _robot_prism(self, ns, mid, stamp, r, rgb):
        m = self._mk(ns, mid, Marker.TRIANGLE_LIST, stamp)
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = _color(rgb, 0.9)
        v = transform(r.model.local_poly, r.x, r.y, r.th)
        z0, z1 = 0.0, self.robot_h
        P = lambda p, z: Point(x=float(p[0]), y=float(p[1]), z=float(z))  # noqa: E731
        tris = [(P(v[0], z1), P(v[1], z1), P(v[2], z1))]
        for k in range(3):
            a, b = v[k], v[(k + 1) % 3]
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
            n = w.stacks[s.name]
            for k in range(n):
                arr.markers.append(self._box('stacks', mid, stamp, s.x, s.y, bh / 2 + k * bh, s.th,
                                             bl, bw, bh * 0.97, BOX_RGB))
                mid += 1
            arr.markers.append(self._text('stack_count', mid, stamp, s.x, s.y, n * bh + 0.06,
                                          f'{s.label}: {n}', 0.06))
            mid += 1
        for z in f.zones:
            n = w.zones[z.name]
            for k in range(n):
                arr.markers.append(self._box('piles', mid, stamp, z.x, z.y, bh / 2 + k * bh, z.th,
                                             bl, bw, bh * 0.97, BOX_RGB))
                mid += 1
            if n:
                arr.markers.append(self._text('pile_count', mid, stamp, z.x, z.y, n * bh + 0.06,
                                              f'{z.label}: {n}/{f.zone_capacity}', 0.06, TEAM_RGB[z.team]))
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
