"""Game-table knowledge shared by the simulator and the strategy (no ROS).

Loads config/field_2027.yaml and derives what both sides need: stack and
zone footprints per team, start/end zones, the robot poses to pick from a
stack and to drop into a zone.
"""

import math
import os
from dataclasses import dataclass, field as dc_field
from typing import List, Tuple

import yaml

from .geometry import rect_poly

TEAMS = ('yellow', 'blue')

_DIRS = {'+x': 0.0, '-x': 180.0, '+y': 90.0, '-y': -90.0}


def other_team(team):
    return 'blue' if team == 'yellow' else 'yellow'


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


@dataclass
class ZoneSpec(Rect):
    team: str = 'yellow'
    approach: List[float] = dc_field(default_factory=list)
    label: str = ''          # Y1..Y5 / B1..B5


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
                                       'yellow', appr))
            self.zones.append(ZoneSpec('B_' + z['name'], self.length - z['x'], z['y'], z['lx'],
                                       z['ly'], math.pi - th, 'blue', [math.pi - a for a in appr]))

        # short IDs shown on the table
        for i, s in enumerate(sorted(self.stacks, key=lambda s: (round(s.x, 2), -s.y)), 1):
            s.label = f'P{i}'
        for team, prefix in (('yellow', 'Y'), ('blue', 'B')):
            zs = sorted(self.team_zones(team), key=lambda z: (round(abs(z.x - self.length / 2), 2) * -1, -z.y))
            for i, z in enumerate(zs, 1):
                z.label = f'{prefix}{i}'

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
        for c in cfg.get('decor_circles', []):
            self.decor.append((c['x'], c['y'], c['r']))
            self.decor.append((self.length - c['x'], c['y'], c['r']))

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

    def stack(self, name) -> StackSpec:
        return next(s for s in self.stacks if s.name == name)

    def zone(self, name) -> ZoneSpec:
        return next(z for z in self.zones if z.name == name)

    def team_zones(self, team) -> List[ZoneSpec]:
        return [z for z in self.zones if z.team == team]

    def start_pose(self, team) -> Tuple[float, float, float]:
        z = self.start_zones[team]
        return z.x, z.y, self.start_headings[team]

    def pile_poly(self, zone: ZoneSpec):
        """Footprint of boxes dropped in a zone (centred, along its long axis)."""
        return rect_poly(zone.x, zone.y, self.box_length, self.box_width, zone.th)

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
