"""2D world for the 2027 table: two triangle robots, box stacks, drop zones.

Pure python + numpy so it runs inside the ROS node or headless at many
times real time.

Simplifications (on purpose):
  * a stack is ONE object with a box count; picking takes up to 3 boxes onto
    a face, dropping puts the face's boxes into a zone as one pile;
  * no box physics: nothing gets pushed or knocked over;
  * robots are kinematic (velocity commands + acceleration limits) and are
    stopped by the borders, the stacks, the piles in the zones and the other
    robot. Boxes held on a face are part of the robot's footprint.
"""

import math
import random
from dataclasses import dataclass, field as dc_field
from typing import Dict, List, Optional

import numpy as np

from .field import Field, other_team
from .geometry import (closest_point_on_poly, poly_inside_poly, poly_segments,
                       polygon_from_face_angles, raycast, rect_poly, rot, sat_mtv,
                       transform, wrap)

PICK = 0
DROP = 1


class RobotModel:
    def __init__(self, cfg: dict):
        self.side = float(cfg.get('side', 0.34))
        self.face_angles = [math.radians(a) for a in cfg.get('face_angles_deg', [0, 120, 240])]
        self.capacity = int(cfg.get('boxes_per_face', 3))
        self.vmax = float(cfg.get('max_lin_vel', 1.0))
        self.amax = float(cfg.get('max_lin_acc', 1.5))
        self.wmax = float(cfg.get('max_ang_vel', 4.0))
        self.awmax = float(cfg.get('max_ang_acc', 12.0))
        self.cmd_timeout = float(cfg.get('cmd_timeout', 0.5))
        self.mast = float(cfg.get('mast_size', 0.08))
        self.r_in = self.side / (2.0 * math.sqrt(3.0))
        self.r_out = self.side / math.sqrt(3.0)
        self.local_poly = polygon_from_face_angles(self.side, self.face_angles)

    def face_center_local(self, f):
        a = self.face_angles[f]
        return np.array([self.r_in * math.cos(a), self.r_in * math.sin(a)])


@dataclass
class Action:
    face: int
    mode: int
    t_start: float
    duration: float
    engage: float = 0.0
    attached: bool = False
    done: bool = False
    success: bool = False
    count: int = 0
    message: str = ''
    phase: str = 'start'
    target: Optional[str] = None


@dataclass
class RobotState:
    name: str
    team: str
    model: RobotModel
    x: float = 0.0
    y: float = 0.0
    th: float = 0.0
    vx: float = 0.0          # world-frame velocity
    vy: float = 0.0
    w: float = 0.0
    cmd: tuple = (0.0, 0.0, 0.0)  # body-frame command
    last_cmd_t: float = -1e9
    faces: List[int] = dc_field(default_factory=list)   # boxes held per face
    action: Optional[Action] = None
    last_result: Optional[Action] = None
    contacts: int = 0
    in_contact: bool = False

    @property
    def poly(self):
        return transform(self.model.local_poly, self.x, self.y, self.th)

    def free_slots(self, f):
        return self.model.capacity - self.faces[f]

    def carried(self):
        return sum(self.faces)


class World:
    def __init__(self, field: Field, sim_cfg: dict, team_us='yellow', seed=0):
        self.field = field
        self.cfg = sim_cfg
        self.team_us = team_us
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)
        rob_cfg = dict(sim_cfg.get('robot', {}))
        opp_cfg = dict(rob_cfg)
        opp_cfg.update(sim_cfg.get('opponent_robot', {}) or {})
        self.models = {'us': RobotModel(rob_cfg), 'opp': RobotModel(opp_cfg)}
        self.tool = dict(sim_cfg.get('tool', {}))
        self.lidar = dict(sim_cfg.get('lidar', {}))
        self.dt = float(sim_cfg.get('physics', {}).get('dt', 0.01))
        self.duration = field.match_duration
        self.reset()

    # ----------------------------------------------------------------- setup
    def reset(self):
        f = self.field
        self.time = 0.0
        self.match_t0 = None
        self.finished = False
        self.final_score = None
        self.events = []
        self.stacks: Dict[str, int] = {s.name: f.boxes_per_stack for s in f.stacks}
        self.zones: Dict[str, int] = {z.name: 0 for z in f.zones}
        self.zone_full_stack: Dict[str, bool] = {z.name: False for z in f.zones}
        self.stack_polys = {s.name: s.poly for s in f.stacks}
        self.pile_polys = {z.name: f.pile_poly(z) for z in f.zones}
        self.robots: Dict[str, RobotState] = {}
        for name, team in (('us', self.team_us), ('opp', other_team(self.team_us))):
            m = self.models[name]
            x, y, th = f.start_pose(team)
            self.robots[name] = RobotState(name, team, m, x, y, th,
                                           faces=[0 for _ in m.face_angles])

    # ----------------------------------------------------------------- match
    @property
    def running(self):
        return self.match_t0 is not None and not self.finished

    @property
    def match_time(self):
        if self.match_t0 is None:
            return 0.0
        return min(self.time - self.match_t0, self.duration)

    @property
    def time_left(self):
        return self.duration - self.match_time

    def start_match(self):
        if self.match_t0 is None:
            self.match_t0 = self.time
            self._event('match started')

    def _event(self, msg):
        self.events.append((round(self.match_time, 2), msg))

    # -------------------------------------------------------------- commands
    def set_cmd(self, name, vx, vy, wz):
        r = self.robots[name]
        r.cmd = (float(vx), float(vy), float(wz))
        r.last_cmd_t = self.time

    def request_action(self, name, face, mode):
        """Start a pick/drop on a face. Returns (accepted, reason)."""
        r = self.robots[name]
        if not self.running:
            return False, 'match not running'
        if r.action is not None and not r.action.done:
            return False, 'tool busy'
        if face < 0 or face >= len(r.faces):
            return False, 'bad face'
        if mode == PICK:
            if r.free_slots(face) == 0:
                return False, 'face full'
            e = float(self.tool.get('pick_engage_time', 0.4))
            r.action = Action(face, PICK, self.time, e, engage=e, phase='engaging')
        elif mode == DROP:
            if r.faces[face] == 0:
                return False, 'face empty'
            r.action = Action(face, DROP, self.time, float(self.tool.get('drop_time', 0.6)),
                              phase='releasing')
        else:
            return False, 'bad mode'
        return True, ''

    def cancel_action(self, name):
        r = self.robots[name]
        a = r.action
        if a is not None and not a.done:
            self._finish(r, a, a.attached, a.count, 'canceled')
            return True
        return False

    def action_progress(self, name):
        a = self.robots[name].action
        if a is None or a.done:
            return None
        return a.phase, min(1.0, (self.time - a.t_start) / max(a.duration, 1e-6))

    def pop_result(self, name):
        r = self.robots[name]
        res, r.last_result = r.last_result, None
        return res

    def _finish(self, r, a, success, count, msg):
        a.done, a.success, a.count, a.message, a.phase = True, success, count, msg, 'done'
        r.last_result = a
        r.action = None

    # ------------------------------------------------------------------- tool
    def face_frame(self, r: RobotState, f):
        a = r.th + r.model.face_angles[f]
        n = np.array([math.cos(a), math.sin(a)])
        c = np.array([r.x, r.y]) + rot(r.th) @ r.model.face_center_local(f)
        return c, n

    def held_poly(self, r: RobotState, f):
        """Footprint of the boxes held on face f (stacked, long side along the face)."""
        c, n = self.face_frame(r, f)
        bw = self.field.box_width
        p = c + n * (bw / 2 + 0.003)
        return rect_poly(p[0], p[1], self.field.box_length, bw, math.atan2(n[1], n[0]) + math.pi / 2)

    def robot_polys(self, r: RobotState):
        polys = [r.poly]
        for f, n in enumerate(r.faces):
            if n:
                polys.append(self.held_poly(r, f))
        return polys

    def stack_in_reach(self, r: RobotState, f):
        reach = float(self.tool.get('pick_reach', 0.05))
        max_ang = math.radians(float(self.tool.get('pick_max_angle_deg', 25.0)))
        max_lat = float(self.tool.get('pick_max_lateral', 0.12))
        c, n = self.face_frame(r, f)
        best, best_d = None, float('inf')
        for name, count in self.stacks.items():
            if count <= 0:
                continue
            poly = self.stack_polys[name]
            _, d = closest_point_on_poly(c, poly)
            if d > reach:
                continue
            k_best, dot_best = None, -2.0
            for k in range(4):
                e = poly[(k + 1) % 4] - poly[k]
                en = np.array([e[1], -e[0]]) / np.linalg.norm(e)
                if (-en) @ n > dot_best:
                    dot_best, k_best = (-en) @ n, k
            if dot_best < math.cos(max_ang):
                continue
            mid = (poly[k_best] + poly[(k_best + 1) % 4]) / 2
            e = poly[(k_best + 1) % 4] - poly[k_best]
            if abs((c - mid) @ (e / np.linalg.norm(e))) > max_lat:
                continue
            if d < best_d:
                best, best_d = name, d
        return best

    def zone_under_face(self, r: RobotState, f):
        c, n = self.face_frame(r, f)
        p = c + n * (self.field.box_width / 2 + 0.003)
        for z in self.field.zones:
            if z.contains(p, margin=0.01):
                return z.name
        return None

    def _update_action(self, r: RobotState):
        a = r.action
        if a is None or a.done:
            return
        el = self.time - a.t_start
        if a.mode == PICK:
            if not a.attached and el >= a.engage:
                name = self.stack_in_reach(r, a.face)
                if name is None:
                    self._finish(r, a, False, 0, 'no stack in reach')
                    return
                n = min(r.free_slots(a.face), self.stacks[name])
                self.stacks[name] -= n
                r.faces[a.face] += n
                a.attached, a.count, a.target = True, n, name
                a.duration = a.engage + n * float(self.tool.get('pick_time_per_box', 0.2))
                a.phase = 'lifting'
                self._event(f'{r.name} picked {n} from {self.field.label(name)} on face {a.face}')
            elif a.attached and el >= a.duration:
                self._finish(r, a, True, a.count, f'picked {a.count} from {self.field.label(a.target)}')
        else:
            if el >= a.duration:
                z = self.zone_under_face(r, a.face)
                if z is None:
                    self._finish(r, a, False, 0, 'not above a drop zone')
                    return
                space = self.field.zone_capacity - self.zones[z]
                if space <= 0:
                    self._finish(r, a, False, 0, f'{self.field.label(z)} is full')
                    return
                n = min(space, r.faces[a.face])
                one_stack = (self.zones[z] == 0 and n == self.field.zone_capacity)
                self.zones[z] += n
                if one_stack:
                    self.zone_full_stack[z] = True
                r.faces[a.face] -= n
                self._event(f'{r.name} dropped {n} in {self.field.label(z)} from face {a.face}')
                self._finish(r, a, True, n, f'dropped {n} in {self.field.label(z)}')

    # ---------------------------------------------------------------- physics
    def step(self, dt):
        n = max(1, int(round(dt / self.dt)))
        h = dt / n
        for _ in range(n):
            self._substep(h)

    def _substep(self, h):
        self.time += h
        if self.running and self.match_time >= self.duration:
            self._end_match()
        active = self.running
        for r in self.robots.values():
            m = r.model
            if active and self.time - r.last_cmd_t <= m.cmd_timeout:
                cvx, cvy, cw = r.cmd
            else:
                cvx = cvy = cw = 0.0
            sp = math.hypot(cvx, cvy)
            if sp > m.vmax:
                cvx, cvy = cvx * m.vmax / sp, cvy * m.vmax / sp
            cw = max(-m.wmax, min(m.wmax, cw))
            c, s = math.cos(r.th), math.sin(r.th)
            tvx, tvy = c * cvx - s * cvy, s * cvx + c * cvy
            dvx, dvy = tvx - r.vx, tvy - r.vy
            dv = math.hypot(dvx, dvy)
            lim = m.amax * h
            if dv > lim:
                dvx, dvy = dvx * lim / dv, dvy * lim / dv
            r.vx += dvx
            r.vy += dvy
            r.w += max(-m.awmax * h, min(m.awmax * h, cw - r.w))
            r.x += r.vx * h
            r.y += r.vy * h
            r.th = wrap(r.th + r.w * h)
        self._collide()
        for r in self.robots.values():
            self._update_action(r)

    def _push_robot(self, r: RobotState, nvec, depth):
        r.x += nvec[0] * depth
        r.y += nvec[1] * depth
        vn = r.vx * nvec[0] + r.vy * nvec[1]
        if vn < 0:
            r.vx -= vn * nvec[0]
            r.vy -= vn * nvec[1]

    def _static_obstacles(self):
        f = self.field
        out = [(self.stack_polys[n], f.stack(n).x, f.stack(n).y, 0.2)
               for n, c in self.stacks.items() if c > 0]
        out += [(self.pile_polys[n], f.zone(n).x, f.zone(n).y, 0.2)
                for n, c in self.zones.items() if c > 0]
        return out

    def _collide(self):
        f = self.field
        robots = list(self.robots.values())
        statics = self._static_obstacles()
        for r in robots:
            reach = r.model.r_out + 0.01
            # held boxes are lifted above the borders, stacks and piles:
            # only the robot body is stopped by them
            poly = r.poly
            for axis, lo, hi in ((0, 0.0, f.length), (1, 0.0, f.width)):
                mn, mx = poly[:, axis].min(), poly[:, axis].max()
                nvec = np.zeros(2)
                if mn < lo:
                    nvec[axis] = 1.0
                    self._push_robot(r, nvec, lo - mn)
                elif mx > hi:
                    nvec[axis] = -1.0
                    self._push_robot(r, nvec, mx - hi)
            for (spoly, sx, sy, sr) in statics:
                if math.hypot(r.x - sx, r.y - sy) > reach + sr:
                    continue
                res = sat_mtv(r.poly, spoly)
                if res:
                    self._push_robot(r, res[0], res[1])
        a, b = robots
        touching = False
        if math.hypot(a.x - b.x, a.y - b.y) < 2 * (a.model.r_out + f.box_width):
            for pa in self.robot_polys(a):
                for pb in self.robot_polys(b):
                    res = sat_mtv(pa, pb)
                    if res:
                        touching = True
                        nvec, depth = res
                        rel = (a.vx - b.vx) * nvec[0] + (a.vy - b.vy) * nvec[1]
                        self._push_robot(a, nvec, depth / 2)
                        self._push_robot(b, -nvec, depth / 2)
                        if not a.in_contact and rel < -0.02:
                            a.contacts += 1
                            b.contacts += 1
                            self._event(f'robots collided (closing speed {-rel:.2f} m/s)')
        a.in_contact = b.in_contact = touching

    # ------------------------------------------------------------------ lidar
    def scan(self, name):
        """Simulated mast lidar of robot `name`, in its base_scan frame.

        Returns (ranges, angle_min, angle_increment)."""
        L = self.lidar
        n = int(L.get('samples', 500))
        inc = 2 * math.pi / n
        a_min = -math.pi
        r = self.robots[name]
        world_angles = r.th + float(L.get('yaw_offset', math.pi)) + a_min + inc * np.arange(n)
        segs = []
        for o in self.robots.values():
            if o is r:
                continue
            segs.append(poly_segments(rect_poly(o.x, o.y, o.model.mast, o.model.mast, o.th)))
            if L.get('see_bodies', False):
                segs.append(poly_segments(o.poly))
        if L.get('see_walls', False):
            segs.append(poly_segments(rect_poly(self.field.length / 2, self.field.width / 2,
                                                self.field.length, self.field.width)))
        segs = np.concatenate(segs) if segs else np.zeros((0, 2, 2))
        ranges = raycast(np.array([r.x, r.y]), world_angles, segs, float(L.get('detect_range', 1.0)))
        hit = np.isfinite(ranges)
        if hit.any():
            ranges[hit] += self.np_rng.normal(0.0, float(L.get('noise_std', 0.0)), hit.sum())
            drop = self.np_rng.random(n) < float(L.get('dropout', 0.0))
            ranges[hit & drop] = np.inf
        return ranges, a_min, inc

    # ---------------------------------------------------------------- scoring
    def score(self, team):
        f = self.field
        sc = f.scoring
        boxes = full = 0
        for z in f.team_zones(team):
            boxes += self.zones[z.name]
            if self.zones[z.name] >= f.zone_capacity:
                full += 1
        robot = next(r for r in self.robots.values() if r.team == team)
        ez = f.end_zones[team]
        poly = robot.poly
        if poly_inside_poly(poly, ez.poly):
            zone_state = 'full'
        elif any(ez.contains(v) for v in poly):
            zone_state = 'partial'
        else:
            zone_state = 'out'
        total = boxes * sc.get('box_in_zone', 0) + full * sc.get('full_stack_bonus', 0)
        if zone_state == 'full':
            total += sc.get('robot_in_end_zone', 0)
        elif zone_state == 'partial':
            total += sc.get('robot_partly_in_end_zone', 0)
        return {'boxes': boxes, 'full_stacks': full, 'end_zone': zone_state,
                'carried': robot.carried(), 'contacts': robot.contacts, 'total': int(total)}

    def _end_match(self):
        self.finished = True
        for r in self.robots.values():
            if r.action is not None and not r.action.done:
                self._finish(r, r.action, False, r.action.count, 'match over')
            r.vx = r.vy = r.w = 0.0
        self.final_score = {t: self.score(t) for t in ('yellow', 'blue')}
        self._event('match over')

    def snapshot(self):
        """Compact state for replays / animations."""
        return {
            't': round(self.match_time, 2),
            'robots': {k: (r.x, r.y, r.th, list(r.faces), r.action.phase if r.action else '')
                       for k, r in self.robots.items()},
            'stacks': dict(self.stacks),
            'zones': dict(self.zones),
        }
