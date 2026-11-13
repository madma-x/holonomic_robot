"""Game-table knowledge shared by the simulator and the strategy (no ROS).

Loads config/field_2027.yaml and derives what both sides need: stack and
zone footprints per team, start/end zones, the robot poses to pick from a
stack and to drop into a zone.
"""

import math
import os
from dataclasses import dataclass, field as dc_field
from typing import List, Optional, Tuple

import numpy as np
import yaml

from .geometry import rect_poly, truncated_triangle_apothem

TEAMS = ('yellow', 'blue')

_DIRS = {'+x': 0.0, '-x': 180.0, '+y': 90.0, '-y': -90.0}


def other_team(team):
    return 'blue' if team == 'yellow' else 'yellow'


def _point_in_polygon(p, poly):
    """Ray-casting point-in-polygon test - any simple polygon, either
    winding order, convex or not (the hand-traced castle outline isn't
    guaranteed convex)."""
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_at_y = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_at_y:
                inside = not inside
    return inside


def default_field_file():
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.join(here, '..', 'config', 'field_2027.yaml')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(get_package_share_directory('table_sim'),
                                          'config', 'field_2027.yaml'))
    except Exception:  # noqa: BLE001 - ament not available (headless use)
        pass
    for c in candidates:
        if os.path.exists(c):
            return os.path.abspath(c)
    raise FileNotFoundError('field_2027.yaml not found')


@dataclass
class Rect:
    name: str
    x: float
    y: float
    lx: float
    ly: float
    th: float = 0.0          # radians

    @property
    def poly(self):
        return rect_poly(self.x, self.y, self.lx, self.ly, self.th)

    def contains(self, p, margin=0.0):
        c, s = math.cos(self.th), math.sin(self.th)
        dx, dy = p[0] - self.x, p[1] - self.y
        u, v = c * dx + s * dy, -s * dx + c * dy
        return abs(u) <= self.lx / 2 + margin and abs(v) <= self.ly / 2 + margin


@dataclass
class StackSpec(Rect):
    approach: List[float] = dc_field(default_factory=list)   # radians, centre -> robot
    side: str = 'center'
    label: str = ''          # P1..P10
    steal_zone: str = ''     # non-empty: this 'stack' is the pile in that (enemy) drop zone; face 0 only


@dataclass
class ZoneSpec(Rect):
    team: str = 'yellow'
    approach: List[float] = dc_field(default_factory=list)
    label: str = ''          # Y1..Y5 / B1..B5 (rect) or YC1..YC4 / BC1..BC4 (circle)
    shape: str = 'rect'      # 'rect' | 'circle'
    capacity: int = 3        # max boxes
    vertical_only: bool = False   # circle zones: only accepts flipped (vertical) boxes
    # Robot dock pose (x, y, heading of the face that drops), from field yaml `docks:` (Max). When set,
    # the zone's own x/y/th are derived from it (where the held box lands) and World only accepts a
    # drop made from this pose (sim.yaml tool.dock_tol_m / dock_tol_deg). None: old zone geometry.
    dock: Optional[tuple] = None

    def contains(self, p, margin=0.0):
        if self.shape == 'circle':
            return math.hypot(p[0] - self.x, p[1] - self.y) <= self.lx / 2 + margin
        return super().contains(p, margin)

    @property
    def poly(self):
        if self.shape == 'circle':
            n, r = 24, self.lx / 2
            return np.array([[self.x + r * math.cos(2 * math.pi * k / n),
                             self.y + r * math.sin(2 * math.pi * k / n)] for k in range(n)])
        return super().poly


@dataclass
class Spot:
    """A robot pose that puts one face flat against a stack (pick) or with
    its held boxes centred on a zone (drop)."""
    kind: str             # pick | drop
    target: str           # stack or zone name
    x: float              # robot centre
    y: float
    face_heading: float   # direction the working face must point
    pre_x: float          # pre-approach point, clear of the obstacle
    pre_y: float


class Field:
    def __init__(self, path=None):
        path = path or default_field_file()
        with open(path) as f:
            cfg = yaml.safe_load(f)
        self.cfg = cfg
        self.length = float(cfg['table']['length_x'])
        self.width = float(cfg['table']['width_y'])
        self.match_duration = float(cfg.get('match_duration', 85.0))
        b = cfg['box']
        self.box_length = float(b['length'])
        self.box_width = float(b['width'])
        self.box_height = float(b['height'])
        self.boxes_per_stack = int(cfg.get('boxes_per_stack', 4))
        self.zone_capacity = int(cfg.get('zone_capacity', 3))
        # Max boxes in a rect zone once its construction is built (extras +1 each).
        # Extra boxes in an already-built zone (+1 each): off by default for now,
        # they made the robot get stuck on zones it had already built.
        self.extra_boxes = bool(cfg.get('extra_boxes', True))
        self.zone_max_boxes = (int(cfg.get('zone_max_boxes', self.zone_capacity))
                               if self.extra_boxes else self.zone_capacity)
        # discards (discard_docks:) always use the zone total of the yaml, extra_boxes or not
        self.discard_max_boxes = int(cfg.get('zone_max_boxes', self.zone_capacity))
        self.door_accepts_extras = bool(cfg.get('door_accepts_extras', True))
        self.scoring = dict(cfg.get('scoring', {}))

        self.stacks: List[StackSpec] = []
        for s in cfg['stacks']:
            th = math.radians(s.get('th', 0.0))
            appr = [math.radians(_DIRS[a]) for a in s['approach']]
            name = ('Y_' if s.get('mirror') else '') + s['name']
            self.stacks.append(StackSpec(name, s['x'], s['y'], self.box_length, self.box_width,
                                         th, appr, self._half(s['x'])))
            if s.get('mirror'):
                mx = self.length - s['x']
                self.stacks.append(StackSpec('B_' + s['name'], mx, s['y'], self.box_length,
                                             self.box_width, math.pi - th,
                                             [math.pi - a for a in appr], self._half(mx)))

        self.zones: List[ZoneSpec] = []
        for z in cfg['zones']:
            th = math.radians(z.get('th', 0.0))
            appr = [math.radians(a) for a in z['approach_deg']]
            self.zones.append(ZoneSpec('Y_' + z['name'], z['x'], z['y'], z['lx'], z['ly'], th,
                                       'yellow', appr, capacity=self.zone_capacity))
            # `blue:` lets a zone's blue twin keep its own independent pose
            # instead of being auto-mirrored from the yellow one (e.g. Max
            # repositioned yellow's drop zones in the reference artifact but
            # wants the opponent's left where they were).
            bz = z.get('blue')
            if bz is not None:
                bth = math.radians(bz.get('th', 0.0))
                bappr = [math.radians(a) for a in bz['approach_deg']]
                self.zones.append(ZoneSpec('B_' + z['name'], bz['x'], bz['y'], z['lx'], z['ly'],
                                           bth, 'blue', bappr, capacity=self.zone_capacity))
            else:
                self.zones.append(ZoneSpec('B_' + z['name'], self.length - z['x'], z['y'], z['lx'],
                                           z['ly'], math.pi - th, 'blue', [math.pi - a for a in appr],
                                           capacity=self.zone_capacity))

        # circle zones: vertical (flipped) boxes only, up to 3 stacked (one
        # drop can place 1, 2 or 3 at once - FaceTool.Goal.count).
        for i, c in enumerate(cfg.get('decor_circles', []), 1):
            r = float(c['r'])
            a = math.radians(float(c.get('approach_deg', 0.0)))
            self.zones.append(ZoneSpec(f'Y_circle_{i}', c['x'], c['y'], 2 * r, 2 * r, 0.0,
                                       'yellow', [a], shape='circle', capacity=3, vertical_only=True))
            bc = c.get('blue')
            if bc is not None:
                ba = math.radians(float(bc.get('approach_deg', 0.0)))
                self.zones.append(ZoneSpec(f'B_circle_{i}', bc['x'], bc['y'], 2 * r, 2 * r, 0.0,
                                           'blue', [ba], shape='circle', capacity=3, vertical_only=True))
            else:
                self.zones.append(ZoneSpec(f'B_circle_{i}', self.length - c['x'], c['y'], 2 * r, 2 * r,
                                           0.0, 'blue', [math.pi - a], shape='circle', capacity=3,
                                           vertical_only=True))

        # short IDs shown on the table
        for i, s in enumerate(sorted(self.stacks, key=lambda s: (round(s.x, 2), -s.y)), 1):
            s.label = f'P{i}'
        for team, prefix in (('yellow', 'Y'), ('blue', 'B')):
            zs = sorted((z for z in self.team_zones(team) if z.shape != 'circle'),
                       key=lambda z: (round(abs(z.x - self.length / 2), 2) * -1, -z.y))
            for i, z in enumerate(zs, 1):
                z.label = f'{prefix}{i}'
            cs = sorted((z for z in self.team_zones(team) if z.shape == 'circle'), key=lambda z: -z.y)
            for i, z in enumerate(cs, 1):
                z.label = f'{prefix}C{i}'

        # Dock poses (Max): the robot pose for each own drop, the only thing that matters - the zone's
        # position is derived from it (box centre = dock + (r_in + box_width/2 + 3 mm) along the face
        # heading; vertical: the centre slot). Yellow only: the opponent's (blue) zones keep their own
        # geometry so the scripted opponents are unaffected.
        docks = cfg.get('docks') or {}
        if docks:
            r_in = truncated_triangle_apothem(float(cfg.get('robot_face_long_m', 0.22)),
                                              float(cfg.get('robot_face_short_m', 0.08)))
            off = r_in + self.box_width / 2 + 0.003
            for lab, d in docks.items():
                for team, prefix in (('yellow', 'Y'),):
                    z = next((z for z in self.team_zones(team) if z.label == prefix + lab[1:]), None)
                    if z is None:
                        continue
                    x, y, h = float(d['x']), float(d['y']), math.radians(float(d['yaw_deg']))
                    z.dock = (x, y, h)
                    z.x, z.y = x + off * math.cos(h), y + off * math.sin(h)
                    z.approach = [h + math.pi]
                    if z.shape == 'rect':
                        z.th = h + math.pi / 2          # the box's long axis lies along the face

        # Flip poses next to a circle zone (Max): {zone label: (x, y, body yaw)} - where the flip face
        # flips a tower load just before that circle's drop (only the flip-face stack has to be inside
        # the castle). Yellow only, like docks.
        self.flip_poses = {'yellow': {}, 'blue': {}}
        for lab, d in (cfg.get('flip_poses') or {}).items():
            self.flip_poses['yellow'][str(lab)] = (float(d['x']), float(d['y']), math.radians(float(d['yaw_deg'])))

        # Discard docks (Max): over-picked horizontal stacks dropped roughly next to an already-built
        # wall, inside the (bigger than one stack) drop zone - extras, +1 per box. Robot pose; yaw_deg
        # = heading of the dropping face, like docks. {team: [(zone name, x, y, face heading)]}
        self.discard_docks = {'yellow': [], 'blue': []}
        for d in cfg.get('discard_docks') or []:
            z = next((z for z in self.team_zones('yellow') if z.label == str(d['zone'])), None)
            if z is not None:
                self.discard_docks['yellow'].append(
                    (z.name, float(d['x']), float(d['y']), math.radians(float(d['yaw_deg']))))

        # Enemy drop zones that can be picked from (horizontal piles, face 0 only): a pick 'stack' that
        # starts empty and mirrors the pile the opponent builds there.
        self.steal_labels = [str(x) for x in cfg.get('steal_zones', [])]
        self.own_repick_inside = [str(x) for x in cfg.get('own_repick_inside', [])]
        for prefix in ('Y', 'B'):
            for lab in self.steal_labels:
                z = next((z for z in self.zones if z.label == prefix + lab), None)
                if z is None:
                    continue
                appr = list(z.approach)
                if lab in [str(x) for x in cfg.get('steal_approach_flip', [])] and prefix == 'B':
                    appr = [a + math.pi for a in appr]       # pick from the opposite side of the zone's own approach
                if lab in self.own_repick_inside and prefix == 'Y':
                    appr = [a + math.pi for a in appr]       # our own wall taken back from inside the castle (door)
                st = StackSpec('S_' + z.label, z.x, z.y, self.box_length, self.box_width, z.th,
                               appr, self._half(z.x))
                st.label, st.steal_zone = 'S_' + z.label, z.name
                self.stacks.append(st)

        sz = cfg['start_zone']
        ez = cfg.get('end_zone', sz)
        self.start_zones = {
            'yellow': Rect('Y_start', sz['x'], sz['y'], sz['lx'], sz['ly']),
            'blue': Rect('B_start', self.length - sz['x'], sz['y'], sz['lx'], sz['ly']),
        }
        self.end_zones = {
            'yellow': Rect('Y_end', ez['x'], ez['y'], ez['lx'], ez['ly']),
            'blue': Rect('B_end', self.length - ez['x'], ez['y'], ez['lx'], ez['ly']),
        }
        h = float(sz.get('heading', 0.0))
        self.start_headings = {'yellow': h, 'blue': math.pi - h}
        self.decor = []
        for z in self.zones:          # drawn where the circle zones really are (dock-derived for ours)
            if z.shape == 'circle':
                self.decor.append((z.x, z.y, z.lx / 2))

        # Castle area: only place the flip mechanism (vertical boxes, door)
        # may be used. Traced by Max over the reference artwork.
        cpts = cfg.get('castle', {}).get('points', [])
        self.castle = {
            'yellow': np.array(cpts, dtype=float) if cpts else np.zeros((0, 2)),
            'blue': np.array([[self.length - x, y] for x, y in cpts], dtype=float)
                    if cpts else np.zeros((0, 2)),
        }

    # ------------------------------------------------------------------ util
    def _half(self, x):
        if abs(x - self.length / 2.0) < 0.05:
            return 'center'
        return 'yellow' if x < self.length / 2.0 else 'blue'

    def label(self, name):
        for o in self.stacks + self.zones:
            if o.name == name:
                return o.label
        return name

    def name_by_label(self):
        """Inverse of label(): P1/Y1/YC1 -> internal stack/zone name.

        Anything consuming the JSON on /sim/score or /strategy/status needs
        this - those are keyed by label, while the renderer and the sim's own
        dicts are keyed by internal name."""
        m = {s.label: s.name for s in self.stacks}
        m.update({z.label: z.name for z in self.zones})
        return m

    def stack(self, name) -> StackSpec:
        return next(s for s in self.stacks if s.name == name)

    def zone(self, name) -> ZoneSpec:
        return next(z for z in self.zones if z.name == name)

    def team_zones(self, team) -> List[ZoneSpec]:
        return [z for z in self.zones if z.team == team]

    def start_pose(self, team) -> Tuple[float, float, float]:
        z = self.start_zones[team]
        return z.x, z.y, self.start_headings[team]

    def keepout_poly(self, team):
        """The OPPONENT's castle polygon, counter-clockwise (the mirrored one is clockwise): no robot
        of `team` may enter it. Empty array if the field has no castle."""
        poly = self.castle.get(other_team(team))
        if poly is None or len(poly) == 0:
            return np.zeros((0, 2))
        area = 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
                         for i in range(len(poly)))
        return poly if area > 0 else poly[::-1].copy()

    def start_keepout_poly(self, team):
        """The OPPONENT's start zone: forbidden for a robot of `team` at all times (Max). Our own
        start zone never is (yellow keeps out of blue's, blue keeps out of yellow's)."""
        return np.asarray(self.start_zones[other_team(team)].poly, float)

    def in_start_keepout(self, team, x, y) -> bool:
        return _point_in_polygon((x, y), self.start_keepout_poly(team))

    def in_castle(self, team, x, y) -> bool:
        """True if (x, y) is inside `team`'s castle area (where flipping is
        allowed). No castle defined in the config -> don't gate (True)."""
        poly = self.castle.get(team)
        if poly is None or len(poly) == 0:
            return True
        return _point_in_polygon((x, y), poly)

    def pile_poly(self, zone: ZoneSpec, pose=None):
        """Footprint of boxes dropped in a zone (centred, along its long
        axis) - at `pose` (x, y, th) if given (where they were actually
        dropped), else the zone's own preset centre/orientation."""
        x, y, th = pose if pose is not None else (zone.x, zone.y, zone.th)
        if zone.shape == 'circle':
            # a vertical (flipped) box stands on its box_width x box_width end
            return rect_poly(x, y, self.box_width, self.box_width, th)
        return rect_poly(x, y, self.box_length, self.box_width, th)

    # --------------------------------------------------------- robot poses
    def _spot(self, kind, name, cx, cy, half_depth, appr, r_in, standoff, pre_dist):
        ax, ay = math.cos(appr), math.sin(appr)      # centre -> robot
        d = half_depth + standoff + r_in
        return Spot(kind, name, cx + ax * d, cy + ay * d, appr + math.pi,
                    cx + ax * (d + pre_dist), cy + ay * (d + pre_dist))

    def pick_spots(self, stack: StackSpec, r_in, standoff=0.012, pre_dist=0.20) -> List[Spot]:
        return [self._spot('pick', stack.name, stack.x, stack.y, self.box_width / 2, a,
                           r_in, standoff, pre_dist) for a in stack.approach]

    def drop_spots(self, zone: ZoneSpec, r_in, gap=0.004, pre_dist=0.20) -> List[Spot]:
        # held boxes stand right in front of the face; centre them on the zone
        return [self._spot('drop', zone.name, zone.x, zone.y, self.box_width / 2, a,
                           r_in, gap, pre_dist) for a in zone.approach]
