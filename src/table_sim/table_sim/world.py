"""2D world for the 2027 table: two hexagonal robots, box stacks, drop zones.

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
                       polygon_from_truncated_triangle, raycast, rect_poly, rot, sat_mtv,
                       transform, truncated_triangle_apothem, truncated_triangle_circumradius,
                       wrap)

PICK = 0
DROP = 1
FLIP = 2
DROP_FLOOR = 3     # put a face's horizontal stack down on the bare floor (anywhere), to repick it with another face


class RobotModel:
    def __init__(self, cfg: dict):
        # Hexagon, not a bare equilateral triangle: 3 long (face) edges
        # alternating with 3 short (corner) edges - an equilateral triangle
        # with each 60deg corner symmetrically cut off (see
        # geometry.polygon_from_truncated_triangle), matching the real
        # chassis's cut corners instead of the earlier pointy-vertex
        # placeholder.
        self.face_long_m = float(cfg.get('face_long_m', 0.22))
        self.face_short_m = float(cfg.get('face_short_m', 0.08))
        self.face_angles = [math.radians(a) for a in cfg.get('face_angles_deg', [0, 120, 240])]
        self.capacity = int(cfg.get('boxes_per_face', 3))
        self.vmax = float(cfg.get('max_lin_vel', 1.0))
        self.amax = float(cfg.get('max_lin_acc', 1.5))
        self.wmax = float(cfg.get('max_ang_vel', 4.0))
        self.awmax = float(cfg.get('max_ang_acc', 12.0))
        self.cmd_timeout = float(cfg.get('cmd_timeout', 0.5))
        self.mast = float(cfg.get('mast_size', 0.08))
        self.wheel_coupling = bool(cfg.get('wheel_coupling', False))
        self.wheel_angles = [math.radians(a) for a in cfg.get('wheel_angles_deg', [60, 180, 300])]
        self.wheel_base = float(cfg.get('wheel_base_m', 0.12))
        self.wheel_vmax = float(cfg.get('wheel_max_speed', self.vmax))
        self.flip_face = int(cfg.get('flip_face', 0))   # the one face with the flip mechanism
        # r_in (apothem, centre -> long-edge midpoint) is unchanged by the
        # corner cut - face_center_local below still sits exactly on it.
        # r_out (circumradius) is now exact, not an approximation: all 6
        # vertices of this shape sit at the same distance from the centre.
        self.r_in = truncated_triangle_apothem(self.face_long_m, self.face_short_m)
        self.r_out = truncated_triangle_circumradius(self.face_long_m, self.face_short_m)
        self.local_poly = polygon_from_truncated_triangle(
            self.face_long_m, self.face_short_m, self.face_angles)

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
    req_count: int = 0       # DROP only: boxes requested (0 = as many as fit)


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
    vertical: List[bool] = dc_field(default_factory=list)  # face's boxes flipped vertical?
    # Per face, which of the 3 fixed side-by-side slots (0=left, 1=centre,
    # 2=right, each box_width apart) are occupied while vertical - a flipped
    # box sits exactly where it was flipped, dropping one never reshuffles
    # the others into a re-centred arrangement (no teleporting).
    vertical_slots: List[List[bool]] = dc_field(default_factory=list)
    action: Optional[Action] = None
    last_result: Optional[Action] = None
    flip_action: Optional[Action] = None     # FLIP has its own slot - async, runs
    last_flip_result: Optional[Action] = None  # alongside a PICK/DROP on another face
    contacts: int = 0
    in_contact: bool = False
    door_used: bool = False          # only one door per robot per match
    door_zone: Optional[str] = None  # zone this robot's in-progress door is in

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
        self.camera = dict(sim_cfg.get('camera', {}))
        self.dt = float(sim_cfg.get('physics', {}).get('dt', 0.01))
        # Low-level safety stop on the OPPONENT's commanded velocity, applied
        # in _substep regardless of what's driving it (scripted behaviour,
        # teleop, avoid_us) - a real opponent robot would have its own
        # emergency brake too, and without one here a teleop/dumb-scripted
        # opponent plows straight through our robot at full commanded speed
        # with no reaction at all, which made our avoidance tests entirely
        # one-sided. Only brakes the velocity COMPONENT closing on us, not
        # the whole command, so the opponent can still strafe/back off.
        opp_cfg2 = dict(sim_cfg.get('opponent', {}) or {})
        self.opp_safety_stop_enabled = bool(opp_cfg2.get('safety_stop_enabled', True))
        self.opp_safety_stop_margin_m = float(opp_cfg2.get('safety_stop_margin_m', 0.2))
        self.opp_safety_stop_min_gap_m = float(opp_cfg2.get('safety_stop_min_gap_m', 0.03))
        self.duration = field.match_duration
        # Testing convenience (toggled via sim_node's /sim/set_infinite_time,
        # the match panel's button) - lets a tuning session run as long as
        # needed without the 85s match ending underneath it. Preserved
        # across /sim/reset by sim_node._build_world (same pattern as
        # world.time), since a fresh World() would otherwise default it
        # back to False every reset.
        self.infinite_time = False
        self.reset()

    # ----------------------------------------------------------------- setup
    def reset(self):
        f = self.field
        self.time = 0.0
        self.match_t0 = None
        self.finished = False
        self.final_score = None
        self.events = []
        self.castle_faults = 0          # times OUR robot entered the opponent's castle (forbidden)
        self._in_keepout = False
        self._in_start_keepout = False
        self.keepout_from_s = 20.0     # opponent castle is only forbidden from this match time
        self.stacks: Dict[str, int] = {s.name: (0 if s.steal_zone else f.boxes_per_stack) for s in f.stacks}
        self.zones: Dict[str, int] = {z.name: 0 for z in f.zones}
        self.zone_vertical: Dict[str, int] = {z.name: 0 for z in f.zones}  # of the above, how many are vertical (door posts)
        # First (and only bonus-earning) construction per zone: None / 'wall' /
        # 'tower' / 'door'. Later boxes in the zone score +1 each, no bonus.
        self.zone_kind: Dict[str, Optional[str]] = {z.name: None for z in f.zones}
        # Real pose of a zone's pile, set to where the robot's face actually
        # was on its first drop into that zone - "no box physics on
        # purpose" (one object per zone, just a count) still holds, but the
        # one object's rendered/collision pose should be where it was
        # really placed, not always snapped to the zone's idealized preset
        # centre regardless of how/where the robot approached from.
        self.pile_pose: Dict[str, tuple] = {}
        self.stack_polys = {s.name: s.poly for s in f.stacks}
        self._steal_zone = {s.name: s.steal_zone for s in f.stacks if s.steal_zone}
        self.pile_polys = {z.name: f.pile_poly(z) for z in f.zones}
        # Stacks put down on the bare floor (DROP_FLOOR): {name: [x, y, th, count]}. Score nothing,
        # are obstacles for both robots, any face can pick them back up.
        self.floor_piles: Dict[str, list] = {}
        # Over-picked stacks discarded next to a built wall (field discard_docks:): {name: [x, y, th,
        # count, zone]}. Their boxes are counted in self.zones[zone] (extras, +1 each); solid like piles.
        self.extra_piles: Dict[str, list] = {}
        self._floor_n = 0
        self.robots: Dict[str, RobotState] = {}
        for name, team in (('us', self.team_us), ('opp', other_team(self.team_us))):
            m = self.models[name]
            x, y, th = f.start_pose(team)
            self.robots[name] = RobotState(name, team, m, x, y, th,
                                           faces=[0 for _ in m.face_angles],
                                           vertical=[False for _ in m.face_angles],
                                           vertical_slots=[[False, False, False] for _ in m.face_angles])

    # ----------------------------------------------------------------- match
    @property
    def running(self):
        return self.match_t0 is not None and not self.finished

    @property
    def match_time(self):
        if self.match_t0 is None:
            return 0.0
        elapsed = self.time - self.match_t0
        return elapsed if self.infinite_time else min(elapsed, self.duration)

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

    def _flip_box_in_castle(self, r: RobotState, face):
        """Flip is gated on the FLIPPING BOXES, not the robot centre - and for
        the whole duration (re-checked every tick in _update_flip_action),
        since a flip can now run concurrently with driving."""
        poly = self.held_poly(r, face)
        return all(self.field.in_castle(r.team, float(p[0]), float(p[1])) for p in poly)

    def request_action(self, name, face, mode, count=0):
        """Start a pick/drop/flip on a face. Returns (accepted, reason).
        FLIP runs in its own slot (r.flip_action), concurrently with a
        PICK/DROP on a *different* face (r.action) - only the face actually
        mid-flip is locked against PICK/DROP, and vice versa."""
        r = self.robots[name]
        if not self.running:
            return False, 'match not running'
        if face < 0 or face >= len(r.faces):
            return False, 'bad face'

        if mode == FLIP:
            if face != r.model.flip_face:
                return False, f'face {face} has no flip mechanism'
            if r.flip_action is not None and not r.flip_action.done:
                return False, 'flip already in progress'
            if r.action is not None and not r.action.done and r.action.face == face:
                return False, 'face busy with pick/drop'
            if r.faces[face] == 0:
                return False, 'face empty'
            if not self._flip_box_in_castle(r, face):
                return False, 'flip only allowed inside the castle area'
            phase = 'elevating' if r.vertical[face] else 'flipping'
            r.flip_action = Action(face, FLIP, self.time, float(self.tool.get('flip_time', 0.5)),
                                    phase=phase)
            return True, ''

        # PICK / DROP: the one main slot, still exclusive - but a flip on a
        # *different* face may be running at the same time.
        if r.action is not None and not r.action.done:
            return False, 'tool busy'
        if r.flip_action is not None and not r.flip_action.done and r.flip_action.face == face:
            return False, 'face is mid-flip'
        if mode == PICK:
            if r.vertical[face]:
                return False, 'face is flipped vertical, drop all boxes first'
            if r.free_slots(face) == 0:
                return False, 'face full'
            e = float(self.tool.get('pick_engage_time', 0.4))
            r.action = Action(face, PICK, self.time, e, engage=e, phase='engaging')
        elif mode == DROP:
            if r.faces[face] == 0:
                return False, 'face empty'
            r.action = Action(face, DROP, self.time, float(self.tool.get('drop_time', 0.6)),
                              phase='releasing', req_count=int(count))
        elif mode == DROP_FLOOR:
            if r.faces[face] == 0:
                return False, 'face empty'
            if r.vertical[face]:
                return False, 'face is flipped vertical, only horizontal stacks go on the floor'
            r.action = Action(face, DROP_FLOOR, self.time, float(self.tool.get('drop_time', 0.6)),
                              phase='releasing')
        else:
            return False, 'bad mode'
        return True, ''

    def cancel_action(self, name, flip=False):
        r = self.robots[name]
        a = r.flip_action if flip else r.action
        if a is not None and not a.done:
            self._finish(r, a, a.attached, a.count, 'canceled')
            return True
        return False

    def action_progress(self, name, flip=False):
        a = self.robots[name].flip_action if flip else self.robots[name].action
        if a is None or a.done:
            return None
        return a.phase, min(1.0, (self.time - a.t_start) / max(a.duration, 1e-6))

    def pop_result(self, name, flip=False):
        r = self.robots[name]
        if flip:
            res, r.last_flip_result = r.last_flip_result, None
        else:
            res, r.last_result = r.last_result, None
        return res

    def _finish(self, r, a, success, count, msg):
        a.done, a.success, a.count, a.message, a.phase = True, success, count, msg, 'done'
        if a.mode == FLIP:
            r.last_flip_result = a
            r.flip_action = None
        else:
            r.last_result = a
            r.action = None

    # ------------------------------------------------------------------- tool
    def face_frame(self, r: RobotState, f):
        a = r.th + r.model.face_angles[f]
        n = np.array([math.cos(a), math.sin(a)])
        c = np.array([r.x, r.y]) + rot(r.th) @ r.model.face_center_local(f)
        return c, n

    def _face_drop_pose(self, r: RobotState, f):
        """Where boxes released from face f actually end up - same position/
        orientation formula as held_poly (just released in place), used to
        render/collide the pile where it was really dropped instead of the
        zone's fixed preset centre. Horizontal only - vertical drops go
        through _face_drop_pose_slot, since each of the 3 side-by-side boxes
        has its own exact spot."""
        c, n = self.face_frame(r, f)
        bw = self.field.box_width
        p = c + n * (bw / 2 + 0.003)
        th = math.atan2(n[1], n[0]) + math.pi / 2
        return float(p[0]), float(p[1]), float(th)

    def _slot_point(self, r: RobotState, f, slot):
        """Centre point of vertical slot `slot` (0=left, 1=centre, 2=right,
        one box_width apart) on face f - fixed relative to the face, not
        reshuffled by how many of the other 2 slots are still occupied."""
        c, n = self.face_frame(r, f)
        bw = self.field.box_width
        tangent = np.array([-n[1], n[0]])
        p = c + n * (bw / 2 + 0.003) + tangent * ((slot - 1) * bw)
        return p, n

    def held_poly_slot(self, r: RobotState, f, slot):
        """Footprint of the single standing box in vertical slot `slot`."""
        p, n = self._slot_point(r, f, slot)
        bw = self.field.box_width
        return rect_poly(p[0], p[1], bw, bw, math.atan2(n[1], n[0]) + math.pi / 2)

    def _face_drop_pose_slot(self, r: RobotState, f, slot):
        """Exact pose a vertical slot's box lands at if dropped right now -
        identical to where it's held (no teleport to zone/face centre)."""
        p, n = self._slot_point(r, f, slot)
        th = math.atan2(n[1], n[0]) + math.pi / 2
        return float(p[0]), float(p[1]), float(th)

    def held_poly(self, r: RobotState, f):
        """Single-footprint summary of what face f is holding, for the
        castle gate check only (see _flip_box_in_castle) - zone_under_face
        and actual drops always go per-slot (held_poly_slot), never this.
        Horizontal: one stacked blob (long side along the face). Vertical:
        the full 3-slot-wide span regardless of how many are occupied - a
        conservative, fixed-size footprint rather than one that would
        shrink/shift as boxes are dropped."""
        c, n = self.face_frame(r, f)
        bw = self.field.box_width
        p = c + n * (bw / 2 + 0.003)
        th = math.atan2(n[1], n[0]) + math.pi / 2
        along = 3 * bw if r.vertical[f] else self.field.box_length
        return rect_poly(p[0], p[1], along, bw, th)

    def held_polys(self, r: RobotState, f):
        """Like held_poly but one entry per occupied slot when vertical
        (never merged) - used for collision so a gap between 2 remaining
        boxes isn't treated as solid."""
        if r.vertical[f]:
            return [self.held_poly_slot(r, f, s) for s in range(3) if r.vertical_slots[f][s]]
        return [self.held_poly(r, f)]

    def robot_polys(self, r: RobotState):
        polys = [r.poly]
        for f, n in enumerate(r.faces):
            if n:
                polys.extend(self.held_polys(r, f))
        return polys

    def stack_in_reach(self, r: RobotState, f):
        reach = float(self.tool.get('pick_reach', 0.05))
        max_ang = math.radians(float(self.tool.get('pick_max_angle_deg', 25.0)))
        max_lat = float(self.tool.get('pick_max_lateral', 0.12))
        c, n = self.face_frame(r, f)
        best, best_d = None, float('inf')
        floor = {k: v[3] for k, v in self.floor_piles.items()}
        for name, count in list(self.stacks.items()) + list(floor.items()):
            if count <= 0:
                continue
            if name in self.floor_piles:
                x, y, th, _ = self.floor_piles[name]
                poly = rect_poly(x, y, self.field.box_length, self.field.box_width, th)
                sz = None
            else:
                poly = self.stack_polys[name]
                sz = self._steal_zone.get(name)
            if sz is not None:                       # a pile in an enemy drop zone: face 0 only
                own = self.field.zone(sz).team == r.team
                lab = self.field.zone(sz).label
                if f != 0 or (own and not (lab.endswith('5') or lab[1:] in self.field.own_repick_inside)):
                    continue            # own pile: the door zone (repick) and the own_repick_inside walls only
                poly = self.field.pile_poly(self.field.zone(sz), self.pile_pose.get(sz))
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
        """The zone the held box(es) on face f can drop into, and (vertical
        only) which specific slot's box actually reaches it - any overlap
        between a box's real footprint and the zone counts (not just an
        idealized centre point fully inside it), so the robot doesn't need
        to be perfectly centred on the zone to drop. Vertical: checked slot
        by slot (each box's own fixed position), never a merged blob - two
        side-by-side boxes 11cm apart are not both "above" a 20cm-wide
        circle zone just because their combined span is. Returns
        (zone_name, slot) - slot is None for a horizontal drop."""
        if r.vertical[f]:
            for slot in range(3):
                if not r.vertical_slots[f][slot]:
                    continue
                poly = self.held_poly_slot(r, f, slot)
                for z in self.field.zones:
                    if sat_mtv(poly, z.poly):
                        return z.name, slot
            return None, None
        c, n = self.face_frame(r, f)
        bw = self.field.box_width
        p = c + n * (bw / 2 + 0.003)
        poly = rect_poly(p[0], p[1], self.field.box_length, bw, math.atan2(n[1], n[0]) + math.pi / 2)
        for z in self.field.zones:
            if sat_mtv(poly, z.poly):
                return z.name, None
        return None, None

    def dock_under_face(self, r: RobotState, f):
        """Closest own zone with a dock pose (field yaml docks:) for a drop off face f:
        (zone_name, slot, pos_err_m, yaw_err_rad), or None. Vertical: per occupied slot, the robot
        pose shifted so that slot's box sits where the centre slot's would at the dock."""
        a = r.th + r.model.face_angles[f]
        bw = self.field.box_width
        tangent = np.array([-math.sin(a), math.cos(a)])
        slots = [s for s in range(3) if r.vertical_slots[f][s]] if r.vertical[f] else [None]
        best = None
        for z in self.field.team_zones(r.team):
            if z.dock is None or (z.shape == 'circle' and not r.vertical[f]):
                continue
            dx_, dy_, dh = z.dock
            # rect zone: the whole face drops there (horizontal wall, or vertical door posts), robot
            # pose = the dock unshifted; circle: one vertical slot, dock shifted for that slot
            for s_ in (slots if z.shape == 'circle' else [None]):
                p = np.array([r.x, r.y]) + (tangent * ((s_ - 1) * bw) if s_ is not None else 0.0)
                e = math.hypot(p[0] - dx_, p[1] - dy_)
                eth = abs(wrap(a - dh))
                if best is None or e + 0.1 * eth < best[2] + 0.1 * best[3]:
                    best = (z.name, s_, e, eth)
        return best

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
                if name in self.floor_piles:
                    fp = self.floor_piles[name]
                    n = min(r.free_slots(a.face), fp[3])
                    fp[3] -= n
                    if fp[3] <= 0:
                        del self.floor_piles[name]
                    r.faces[a.face] += n
                    a.attached, a.count, a.target = True, n, name
                    a.duration = a.engage + n * float(self.tool.get('pick_time_per_box', 0.2))
                    a.phase = 'lifting'
                    self._event(f'{r.name} picked {n} from floor pile {name} on face {a.face}')
                    return
                n = min(r.free_slots(a.face), self.stacks[name])
                self.stacks[name] -= n
                sz = self._steal_zone.get(name)
                if sz is not None:                   # stolen from the opponent's pile
                    self.zones[sz] -= n
                    if self.zones[sz] < self.field.zone_capacity:
                        self.zone_kind[sz] = None
                    if self.zones[sz] <= 0:
                        self.pile_pose.pop(sz, None)
                r.faces[a.face] += n
                a.attached, a.count, a.target = True, n, name
                a.duration = a.engage + n * float(self.tool.get('pick_time_per_box', 0.2))
                a.phase = 'lifting'
                self._event(f'{r.name} picked {n} from {self.field.label(name)} on face {a.face}')
            elif a.attached and el >= a.duration:
                lbl = a.target if a.target.startswith('floor') else self.field.label(a.target)
                self._finish(r, a, True, a.count, f'picked {a.count} from {lbl}')
        elif a.mode == DROP_FLOOR:
            if el >= a.duration:
                x, y, th = self._face_drop_pose(r, a.face)
                self._floor_n += 1
                name = f'floor{self._floor_n}'
                n = r.faces[a.face]
                self.floor_piles[name] = [x, y, th, n]
                r.faces[a.face] = 0
                self._event(f'{r.name} put {n} on the floor ({name}) from face {a.face} at ({x:.2f}, {y:.2f})')
                self._finish(r, a, True, n, f'dropped {n} on the floor ({name})')
        else:
            if el >= a.duration and self._try_discard(r, a):
                return
            if el >= a.duration:
                zname, slot = self.zone_under_face(r, a.face)
                dk = self.dock_under_face(r, a.face)
                tol_p = float(self.tool.get('dock_tol_m', 0.02))
                tol_a = math.radians(float(self.tool.get('dock_tol_deg', 5.0)))
                docked = dk is not None and dk[2] <= tol_p and dk[3] <= tol_a
                if docked:
                    zname = dk[0]
                    if dk[1] is not None:
                        slot = dk[1]
                elif dk is not None and (dk[2] < 0.15 or (zname is not None and self.field.zone(zname).dock is not None)):
                    # a zone with a dock pose: only the dock counts (Max) - no "close enough" overlap
                    self._finish(r, a, False, 0,
                                 f'{self.field.label(dk[0])} not at the dock pose '
                                 f'({dk[2] * 100:.1f} cm, {math.degrees(dk[3]):.1f} deg off)')
                    return
                if zname is None:
                    self._finish(r, a, False, 0, 'not above a drop zone')
                    return
                zone = self.field.zone(zname)
                label = self.field.label(zname)
                face_vertical = r.vertical[a.face]
                if zone.shape == 'rect' and not face_vertical and zone.dock is None:
                    # A horizontal drop must be centred on the zone and square
                    # to it (tolerances in sim.yaml tool.drop_*), not merely
                    # overlap it.
                    px, py, pth = self._face_drop_pose(r, a.face)
                    c, sn = math.cos(-zone.th), math.sin(-zone.th)
                    dx, dy = px - zone.x, py - zone.y
                    along, across = dx * c - dy * sn, dx * sn + dy * c
                    dth = abs(wrap(pth - zone.th)) % math.pi
                    dth = min(dth, math.pi - dth)
                    t = self.tool
                    if (abs(along) > float(t.get('drop_tol_along_m', 0.06))
                            or abs(across) > float(t.get('drop_tol_across_m', 0.05))
                            or math.degrees(dth) > float(t.get('drop_tol_angle_deg', 10.0))):
                        self._finish(r, a, False, 0,
                                     f'{self.field.label(zname)} drop not centred '
                                     f'(along {along * 100:+.0f} cm, across {across * 100:+.0f} cm, '
                                     f'{math.degrees(dth):.0f} deg)')
                        return
                vcount = self.zone_vertical[zname]
                total = self.zones[zname]
                hcount = total - vcount

                if zone.shape == 'circle':
                    if not face_vertical:
                        self._finish(r, a, False, 0, f'{label} needs flipped (vertical) boxes')
                        return
                    space = 1 - total   # one tower per circle
                    if space <= 0:
                        self._finish(r, a, False, 0, f'{label} is full')
                        return
                    # Exactly the one box whose own fixed slot actually
                    # overlaps this zone - never "however many are held",
                    # two boxes 11cm apart can't both land in one 20cm
                    # circle from a single dock pose.
                    n = 1
                elif face_vertical:
                    # building (or continuing) a door's 2 vertical posts
                    if hcount > 0:
                        self._finish(r, a, False, 0, f'{label} already has horizontal boxes')
                        return
                    if vcount >= 2:
                        self._finish(r, a, False, 0, f'{label} already has its 2 posts - '
                                                      'flip the last box horizontal to finish the door')
                        return
                    if r.door_used:
                        self._finish(r, a, False, 0, 'already built a door this match')
                        return
                    if r.door_zone not in (None, zname):
                        self._finish(r, a, False, 0,
                                     f'already building a door in {self.field.label(r.door_zone)}')
                        return
                    n = min(2 - vcount, r.faces[a.face])   # both posts together when available
                    r.door_zone = zname
                else:
                    # horizontal: a normal stack, or topping off a door's 2 posts
                    if vcount == 1:
                        self._finish(r, a, False, 0, f'{label} has 1 post - needs its second vertical box')
                        return
                    if vcount == 2 and a.face != r.model.flip_face:
                        self._finish(r, a, False, 0, f'{label} lid must come from the flip face (elevated)')
                        return
                    # Before its construction a zone takes at most one stack
                    # (zone.capacity); once built, extras fill up to
                    # field.zone_max_boxes (+1 each).
                    limit = (self.field.zone_max_boxes if self.zone_kind[zname]
                             else zone.capacity)
                    if self.zone_kind[zname] == 'door' and not self.field.door_accepts_extras:
                        limit = total
                    space = limit - total
                    if space <= 0:
                        self._finish(r, a, False, 0, f'{label} is full')
                        return
                    n = 1 if vcount == 2 else min(space, r.faces[a.face])

                # Circle zones always drop vertical (world rule above), so
                # they used to be excluded by `not face_vertical` alongside
                # door posts (which DO want excluding - a door's render uses
                # its own posts-symmetric-about-zone.th formula, not
                # pile_pose, see sim_node.py) - meaning a circle-zone drop's
                # marker/obstacle always rendered at the idealised zone
                # centre instead of where the robot really was. For a
                # vertical drop with a known slot, use that slot's own exact
                # pose - the box lands exactly where it was held, never
                # teleporting to the face/zone centre.
                if zname not in self.pile_pose and (zone.shape == 'circle' or not face_vertical):
                    self.pile_pose[zname] = (self._face_drop_pose_slot(r, a.face, slot)
                                              if slot is not None else self._face_drop_pose(r, a.face))
                self.zones[zname] += n
                if face_vertical:
                    self.zone_vertical[zname] += n
                    if slot is not None and n == 1:
                        r.vertical_slots[a.face][slot] = False
                if self.zone_kind[zname] is None:
                    if zone.shape == 'circle':
                        self.zone_kind[zname] = 'tower'
                    elif vcount == 2 and not face_vertical:
                        r.door_used = True   # door completed: 2 posts + this lid
                        self.zone_kind[zname] = 'door'
                    elif not face_vertical and self.zones[zname] >= zone.capacity:
                        self.zone_kind[zname] = 'wall'
                r.faces[a.face] -= n
                if r.faces[a.face] == 0:
                    r.vertical[a.face] = False
                    r.vertical_slots[a.face] = [False, False, False]
                self._event(f'{r.name} dropped {n} in {label} from face {a.face}')
                self._finish(r, a, True, n, f'dropped {n} in {label}')

    def discard_under_face(self, r: RobotState, f):
        """Closest discard dock of r's team for face f: (zone_name, pos_err_m, yaw_err_rad) or None."""
        a = r.th + r.model.face_angles[f]
        best = None
        for zname, dx_, dy_, dh in self.field.discard_docks.get(r.team, []):
            e, eth = math.hypot(r.x - dx_, r.y - dy_), abs(wrap(a - dh))
            if best is None or e + 0.1 * eth < best[1] + 0.1 * best[2]:
                best = (zname, e, eth)
        return best

    def _try_discard(self, r: RobotState, a):
        """A horizontal drop at a discard dock: the boxes go next to the zone's built wall as extras.
        True if this drop was handled here (accepted or refused)."""
        if r.faces[a.face] <= 0:
            return False
        dk = self.discard_under_face(r, a.face)
        if dk is None:
            return False
        zname, e, eth = dk
        if e > float(self.tool.get('discard_tol_m', 0.06)) or eth > math.radians(float(self.tool.get('discard_tol_deg', 10.0))):
            return False
        label = self.field.label(zname)
        kind = self.zone_kind[zname]
        if kind != 'wall' and not (kind == 'door' and self.field.door_accepts_extras):
            self._finish(r, a, False, 0, f'{label} discard: its wall is not built yet')
            return True
        space = self.field.discard_max_boxes - self.zones[zname]
        if space <= 0:
            self._finish(r, a, False, 0, f'{label} is full (extras)')
            return True
        vert = r.vertical[a.face]
        n = min(space, r.faces[a.face])          # standing boxes: all of them in one action (Max)
        if vert:
            # each standing box lands where it stood on the face (its own slot), as it stood
            k = 0
            for slot in range(3):
                if k >= n or not r.vertical_slots[a.face][slot]:
                    continue
                x, y, th = self._face_drop_pose_slot(r, a.face, slot)
                self.extra_piles[f'extra_{label}_{len(self.extra_piles) + 1}'] = [x, y, th, 1, zname, True]
                k += 1
        else:
            x, y, th = self._face_drop_pose(r, a.face)
            self.extra_piles[f'extra_{label}_{len(self.extra_piles) + 1}'] = [x, y, th, n, zname, False]
        self.zones[zname] += n
        r.faces[a.face] -= n
        if r.faces[a.face] == 0:
            r.vertical[a.face] = False
            r.vertical_slots[a.face] = [False, False, False]
        self._event(f'{r.name} discarded {n}{" standing" if vert else ""} next to {label} from face {a.face} '
                    f'(extras, +1 each)')
        self._finish(r, a, True, n, f'discarded {n} in {label}')
        return True

    def _update_flip_action(self, r: RobotState):
        a = r.flip_action
        if a is None or a.done:
            return
        if not self._flip_box_in_castle(r, a.face):
            self._finish(r, a, False, 0, 'boxes left the castle area mid-flip')
            return
        if self.time - a.t_start >= a.duration:
            r.vertical[a.face] = not r.vertical[a.face]
            if r.vertical[a.face]:
                # Horizontal -> vertical: the n held boxes take up their n
                # fixed slots (left, [centre,] right) and stay there until
                # each is individually dropped - never reshuffled/recentred
                # as neighbours leave.
                n = r.faces[a.face]
                r.vertical_slots[a.face] = [i < n for i in range(3)]
            else:
                r.vertical_slots[a.face] = [False, False, False]
            verb = 'vertical' if r.vertical[a.face] else 'horizontal (elevated)'
            self._event(f'{r.name} flipped {r.faces[a.face]} box(es) {verb} on face {a.face}')
            self._finish(r, a, True, r.faces[a.face], f'flipped {verb}')

    # ---------------------------------------------------------------- physics
    def step(self, dt):
        n = max(1, int(round(dt / self.dt)))
        h = dt / n
        for _ in range(n):
            self._substep(h)
        self._sync_steal()
        self._check_keepout()

    def _sync_steal(self):
        """Steal 'stacks' mirror the horizontal piles in the enemy drop zones (towers/doors are not pickable)."""
        for name, sz in self._steal_zone.items():
            ok = self.zone_kind[sz] in (None, 'wall') and self.zone_vertical[sz] == 0
            self.stacks[name] = self.zones[sz] if ok else 0

    def _check_keepout(self):
        """Our robot's centre inside the opponent's castle is a fault (counted once per entry)."""
        r = self.robots['us']
        inside = (self.field.in_castle(other_team(r.team), r.x, r.y)
                  and self.match_t0 is not None
                  and self.time - self.match_t0 >= self.keepout_from_s)
        if inside and any(self.field.zone(z).team != r.team and math.hypot(r.x - self.field.zone(z).x, r.y - self.field.zone(z).y) < 0.7
                          for n, z in self._steal_zone.items()):
            inside = False                       # on its way to / picking a stealable pile: allowed
        if inside and not self._in_keepout:
            self.castle_faults += 1
            self._event(f'FAULT: us entered the opponent castle at ({r.x:.2f}, {r.y:.2f})')
        self._in_keepout = inside
        # the opponent's start zone is forbidden at all times (counted with the castle faults)
        in_start = self.match_t0 is not None and self.field.in_start_keepout(r.team, r.x, r.y)
        if in_start and not self._in_start_keepout:
            self.castle_faults += 1
            self._event(f'FAULT: us entered the opponent start zone at ({r.x:.2f}, {r.y:.2f})')
        self._in_start_keepout = in_start

    def _opponent_safety_brake(self, tvx, tvy, opp, other):
        """Scale down the component of `opp`'s target world-frame velocity
        that's closing on `other`, down to zero by opp_safety_stop_min_gap_m
        - leaves the away/sideways component untouched so the opponent can
        still strafe or back off. Edge-to-edge gap, not centre-to-centre
        (r_out circumradius of each hexagon), so the margin means the same
        thing it does for motion_controller's own avoidance_min_clearance_m_.
        """
        dx, dy = opp.x - other.x, opp.y - other.y
        dist = math.hypot(dx, dy)
        gap = dist - (opp.model.r_out + other.model.r_out)
        start_at = self.opp_safety_stop_min_gap_m + self.opp_safety_stop_margin_m
        if gap >= start_at or dist < 1e-6:
            return tvx, tvy
        nx, ny = dx / dist, dy / dist  # points from other -> opp (away direction)
        receding = tvx * nx + tvy * ny
        if receding >= 0:
            return tvx, tvy  # already moving away or purely sideways
        scale = 0.0
        if gap > self.opp_safety_stop_min_gap_m:
            scale = (gap - self.opp_safety_stop_min_gap_m) / self.opp_safety_stop_margin_m
        new_receding = receding * scale
        tvx += (new_receding - receding) * nx
        tvy += (new_receding - receding) * ny
        return tvx, tvy

    def _substep(self, h):
        self.time += h
        if self.running and not self.infinite_time and self.match_time >= self.duration:
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
            if r.name == 'opp' and self.opp_safety_stop_enabled:
                tvx, tvy = self._opponent_safety_brake(tvx, tvy, r, self.robots['us'])
            dvx, dvy = tvx - r.vx, tvy - r.vy
            dv = math.hypot(dvx, dvy)
            lim = m.amax * h
            if dv > lim:
                dvx, dvy = dvx * lim / dv, dvy * lim / dv
            r.vx += dvx
            r.vy += dvy
            r.w += max(-m.awmax * h, min(m.awmax * h, cw - r.w))
            if m.wheel_coupling:
                # driving and turning share the wheels' top speed: scale (vx, vy, w) together
                bvx, bvy = c * r.vx + s * r.vy, -s * r.vx + c * r.vy
                wmax_ = max(abs(-math.sin(b) * bvx + math.cos(b) * bvy + m.wheel_base * r.w)
                            for b in m.wheel_angles)
                if wmax_ > m.wheel_vmax > 0:
                    k = m.wheel_vmax / wmax_
                    r.vx, r.vy, r.w = r.vx * k, r.vy * k, r.w * k
            r.x += r.vx * h
            r.y += r.vy * h
            r.th = wrap(r.th + r.w * h)
        self._collide()
        for r in self.robots.values():
            self._update_action(r)
            self._update_flip_action(r)

    def _push_robot(self, r: RobotState, nvec, depth):
        r.x += nvec[0] * depth
        r.y += nvec[1] * depth
        vn = r.vx * nvec[0] + r.vy * nvec[1]
        if vn < 0:
            r.vx -= vn * nvec[0]
            r.vy -= vn * nvec[1]

    def _static_obstacles(self):
        f = self.field
        out = [(self.stack_polys[n], f.stack(n).x, f.stack(n).y, 0.2, n)
               for n, c in self.stacks.items() if c > 0]
        for n, c in self.zones.items():
            if c <= 0:
                continue
            pose = self.pile_pose.get(n)
            if pose is not None:
                out.append((f.pile_poly(f.zone(n), pose), pose[0], pose[1], 0.2, n))
            else:
                out.append((self.pile_polys[n], f.zone(n).x, f.zone(n).y, 0.2, n))
        for n, (x, y, th, c) in self.floor_piles.items():
            out.append((rect_poly(x, y, f.box_length, f.box_width, th), x, y, 0.2, n))
        for n, (x, y, th, c, _z, v) in self.extra_piles.items():
            out.append((rect_poly(x, y, f.box_width if v else f.box_length, f.box_width, th), x, y, 0.2, n))
        return out

    def _collide(self):
        f = self.field
        robots = list(self.robots.values())
        statics = self._static_obstacles()
        for r in robots:
            # held boxes are lifted above the borders: only the robot body is
            # stopped by them, but they do collide with stacks and piles
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
            polys = self.robot_polys(r)
            reach = r.model.r_out + f.box_length + 0.01
            for (spoly, sx, sy, sr, sname) in statics:
                if math.hypot(r.x - sx, r.y - sy) > reach + sr:
                    continue
                for i, poly in enumerate(polys):
                    # a held box (poly index > 0) sits right at the drop
                    # standoff, which is also exactly where its own
                    # in-progress door pile now sits (posts dropped, lid
                    # still held) - don't let that self-overlap fight the
                    # robot's own commanded motion; the bare body (index 0)
                    # still can't drive through the built posts.
                    if i > 0 and (sname == r.door_zone or sname.startswith('extra_')):
                        continue
                    res = sat_mtv(poly, spoly)
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

    # ------------------------------------------------------------------ camera
    def camera_view(self, name):
        """Face-mounted ArUco camera of robot `name`: which stacks/zones fall
        inside its FOV cone right now, and their true box count (0 if empty
        but visible - that's the point: confirms emptiness without having to
        drive up to it). No occlusion modelling, same simplification as the
        lidar not seeing through/around stacks.
        """
        C = self.camera
        r = self.robots[name]
        hfov = math.radians(float(C.get('hfov_deg', 160.0)))
        max_range = float(C.get('max_range', 2.5))
        face = int(C.get('face', 0))
        facing = r.th + r.model.face_angles[face % len(r.model.face_angles)]
        out = {}
        for coll, counts in ((self.field.stacks, self.stacks), (self.field.zones, self.zones)):
            for o in coll:
                dx, dy = o.x - r.x, o.y - r.y
                d = math.hypot(dx, dy)
                if d > max_range:
                    continue
                if abs(wrap(math.atan2(dy, dx) - facing)) > hfov / 2:
                    continue
                out[o.name] = counts[o.name]
        return out

    # ---------------------------------------------------------------- scoring
    def score(self, team):
        f = self.field
        sc = f.scoring
        boxes = 0
        kinds = {'wall': 0, 'tower': 0, 'door': 0}
        for z in f.team_zones(team):
            boxes += self.zones[z.name]
            k = self.zone_kind[z.name]
            if k:
                kinds[k] += 1
        robot = next(r for r in self.robots.values() if r.team == team)
        ez = f.end_zones[team]
        poly = robot.poly
        if poly_inside_poly(poly, ez.poly):
            zone_state = 'full'
        elif any(ez.contains(v) for v in poly):
            zone_state = 'partial'
        else:
            zone_state = 'out'
        total = boxes * sc.get('box_in_zone', 0)
        total += sum(n * sc.get(f'{k}_bonus', 0) for k, n in kinds.items())
        if zone_state == 'full':
            total += sc.get('robot_in_end_zone', 0)
        elif zone_state == 'partial':
            total += sc.get('robot_partly_in_end_zone', 0)
        return {'boxes': boxes, 'walls': kinds['wall'], 'towers': kinds['tower'],
                'doors': kinds['door'], 'end_zone': zone_state,
                'carried': robot.carried(), 'contacts': robot.contacts, 'castle_faults': self.castle_faults if team == self.team_us else 0,
                'total': int(total)}

    def _end_match(self):
        self.finished = True
        for r in self.robots.values():
            if r.action is not None and not r.action.done:
                self._finish(r, r.action, False, r.action.count, 'match over')
            if r.flip_action is not None and not r.flip_action.done:
                self._finish(r, r.flip_action, False, r.flip_action.count, 'match over')
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
            'floor': {k: list(v) for k, v in self.floor_piles.items()},
            'extra': {k: list(v[:4]) + [v[4], v[5]] for k, v in self.extra_piles.items()},
            'pile_poses': {k: list(v) for k, v in self.pile_pose.items()},
        }
