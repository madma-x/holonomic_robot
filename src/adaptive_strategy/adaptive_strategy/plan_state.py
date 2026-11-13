"""Abstract forward model for the search planner (no ROS, no physics).

`apply(model, state, action)` is pure: returns a new PlanState or None when
the action is illegal. Legality and scoring mirror table_sim/world.py
(test/test_plan_state.py checks them against World). Drop-order rules
(Y1->Y3 ...) are NOT here: pass `guard(state, action) -> bool` (Phase 2).

Actions are tuples:
  ('pick', stack_idx, face)   ('drop', zone_idx, face)
  ('flip', face)              ('home',)
  ('flip', face, 'dock')      flip while docking for the NEXT drop (a rect zone whose dock keeps the
                              flip-face stack inside the castle, e.g. un-flip at Y2/Y4 before dropping)
  ('flip', face, 'at', zone_idx)  flip at that circle zone's flip pose (field yaml flip_poses:, YC1/YC4);
                              the very next action must be the flip-face drop into that zone
  ('discard', zone_idx, face) drop face's horizontal stack roughly at the zone's discard dock (field
                              yaml discard_docks:), next to its built wall: extras, +1 per box - frees a
                              face of over-picked boxes so the door (Y5 last, flip face only) stays possible
  ('regrip', face)            put face's horizontal stack on the floor right where the robot is,
                              back out, turn, pick it up again with the flip face (towers)
"""
from dataclasses import dataclass, replace
from typing import Optional, Tuple

import math

import numpy as np

from table_sim.field import _point_in_polygon
from table_sim.flipmap import FlipMap
from table_sim.geometry import closest_point_on_poly, rect_poly, truncated_triangle_apothem

from .travel import TravelMatrix

NONE, WALL, TOWER, DOOR = 0, 1, 2, 3
_KIND_KEY = {WALL: 'wall_bonus', TOWER: 'tower_bonus', DOOR: 'door_bonus'}


@dataclass(frozen=True)
class PlanState:
    t: float                       # match time elapsed
    node: str                      # 'home' | 'S:<stack>' | 'Z:<zone>'
    faces: Tuple[int, ...]         # boxes per face
    vert: Tuple[bool, ...]         # face flipped vertical?
    stacks: Tuple[int, ...]        # believed boxes per stack
    zbox: Tuple[int, ...]          # boxes per own zone
    zvert: Tuple[int, ...]         # of those, vertical (door posts / towers)
    zkind: Tuple[int, ...]         # NONE/WALL/TOWER/DOOR
    door_used: bool = False
    door_zone: int = -1
    dropped: Tuple[int, ...] = ()  # zone indices, in drop order
    picked: Tuple[int, ...] = ()   # stack indices, in pick order
    last: str = ''                 # kind of the previous action ('pick', 'drop', ...)
    parked: bool = False
    dock_f: int = -1               # face used at the last pick/drop (the robot is still docked on it)
    risk_cost: float = 0.0         # access risk paid for picks near the forbidden opponent castle (negative = early bonus)
    flip_debt: bool = False        # flipped outside the castle: the very next action must be a castle drop that completes it
    flip_for: int = -1             # flipped at this circle zone's flip pose: the next action must be the drop there
    discarded: Tuple[int, ...] = ()  # zones we discarded extras next to (their discard pile blocks nearby circle docks)
    stolen: Tuple[bool, ...] = ()  # per face: the load came from an enemy drop-zone pile
    slack: float = 0.0             # extra finish margin this plan must keep (a tower built from a stolen
                                   # stack before the door: running late there costs the door)
    bonus: float = 0.0             # denial value REALISED: boxes the opponent would have used that we delivered
    fbonus: Tuple[float, ...] = () # per-box denial value of the stack held on each face (credited when dropped)
    head: Optional[float] = None   # robot body heading after the last pick / drop (None: unknown, no turn cost)
    regrip_flip: bool = False      # regripped onto the flip face: it must be flipped before it drops lying down
                                   # (a wall drops from any face - a regrip only pays for a flip; Max)

    def key(self):
        """Full-state key for dominance pruning (time excluded)."""
        return (self.node, self.faces, self.vert, self.stacks, self.zbox, self.zvert,
                self.zkind, self.door_used, self.door_zone, self.dropped, self.picked,
                self.parked, self.dock_f, self.flip_debt, self.flip_for, self.discarded, self.stolen,
                self.slack, round(self.bonus), round(self.risk_cost), self.regrip_flip)


class PlanModel:
    """Static data shared by all states of one planning problem."""

    def tt(self, a, b):
        """Realistic Nav2 travel time a -> b (grid estimate x travel_factor), + cross_time when the leg
        goes from the top half of the table to the bottom one or back (castle / centre stacks in the
        way; Max: the back-and-forth was costing seconds the grid estimate did not show)."""
        t = self.travel.time(a, b) * self.travel_factor
        if self.cross_time > 0:
            ya, yb = self.travel.nodes[a][1], self.travel.nodes[b][1]
            mid, band = self.field.width / 2, 0.15
            if (ya - mid > band and mid - yb > band) or (mid - ya > band and yb - mid > band):
                t += self.cross_time
        return t

    def __init__(self, field, sim_cfg, team, dock_poses_file=None, stack_counts=None,
                 dock_faces=None):
        robot, tool = sim_cfg['robot'], sim_cfg['tool']
        self.field, self.team = field, team
        self.duration = field.match_duration
        self.capacity = int(robot.get('boxes_per_face', 3))
        self.flip_face = int(robot.get('flip_face', 0))
        self.n_faces = len(robot.get('face_angles_deg', [0, 120, 240]))
        self.engage = float(tool.get('pick_engage_time', 0.4))
        self.per_box = float(tool.get('pick_time_per_box', 0.2))
        self.drop_time = float(tool.get('drop_time', 0.6))
        self.flip_time = float(tool.get('flip_time', 0.5))
        # Real execution vs the bare model (calibrated on the 2026-11-09 RViz match): Nav2 legs take
        # travel_factor x the grid estimate, and every pick / drop costs a fixed docking overhead
        # (back out, approach point, dock in, stop) - a re-dock in the same place is NOT free.
        # home_reserve: every plan state must still reach home with this much to spare.
        self.travel_factor = 1.3
        self.pick_overhead = 1.2
        # measured actual / planned per action kind (strategy.yaml plan_pick/drop_time_factor)
        self.pick_factor = 1.0
        self.drop_factor = 1.0
        self.steal_tower_slack = 3.0
        self.cross_time = 1.0             # s added to a top <-> bottom leg (see tt)
        self.door_earliest = 60.0         # no Y5 drop before this while a table stack is still believed full      # s of extra plan margin once a stolen stack went into a tower
        self.drop_overhead = 1.5
        self.home_reserve = 3.0
        self.scoring = field.scoring
        self.stacks = list(field.stacks)
        self.zones = list(field.team_zones(team))
        self.stack_idx = {s.name: i for i, s in enumerate(self.stacks)}
        self.zone_idx = {z.name: i for i, z in enumerate(self.zones)}
        self.zone_label = [z.label for z in self.zones]
        r_in = truncated_triangle_apothem(float(robot['face_long_m']), float(robot['face_short_m']))
        self.travel = TravelMatrix(field, team, robot, r_in, dock_poses_file)
        self.in_castle = {n: field.in_castle(team, *xy) for n, xy in self.travel.nodes.items()}
        self.stack_nodes = ['S:' + s.name for s in self.stacks]
        # Crowded picks: a stack whose approach point / dock sits close to ANOTHER pick stack (P3/P7
        # next to P5, P4/P8 next to P6) docks slower while that neighbour still has boxes (Nav2's
        # carrying inflation keeps the approach point in high cost). {si: (neighbour si, ...)}
        self.crowded_pick_s = 1.0
        self.crowd_r = 0.30
        self.crowd = {}
        for si, st in enumerate(self.stacks):
            if st.steal_zone:
                continue
            sp = field.pick_spots(st, r_in)[0]
            nb = []
            for sj, o in enumerate(self.stacks):
                if sj == si or o.steal_zone:
                    continue
                d = min(closest_point_on_poly(np.array(p), np.asarray(o.poly))[1]
                        for p in ((sp.x, sp.y), (sp.pre_x, sp.pre_y)))
                if d < self.crowd_r:
                    nb.append(sj)
            if nb:
                self.crowd[si] = tuple(nb)
        self._steal_names = {s.name for s in self.stacks if s.steal_zone}
        # Drop-and-repick: a horizontal pile dropped at the door zone (Y5) can be picked back up with the
        # flip face (face 0) so the door can be built from it. {stack index: zone index}
        # (Towers use the in-place floor regrip instead: ('regrip', f), see apply().)
        self.regrip = {i: self.zone_idx[st.steal_zone] for i, st in enumerate(self.stacks)
                       if st.steal_zone and st.steal_zone in self.zone_idx and st.label.endswith('5')}
        # Last resort for the door (Max): our own Y3 / Y4 wall taken back with the flip face, from inside
        # the castle (field own_repick_inside). {stack index: zone index}
        self.own_repick = {i: self.zone_idx[st.steal_zone] for i, st in enumerate(self.stacks)
                           if st.steal_zone in self.zone_idx
                           and st.label[len('S_') + 1:] in getattr(field, 'own_repick_inside', [])}
        self._near_castle = None
        self.zone_nodes = ['Z:' + z.name for z in self.zones]
        self._stack_counts = stack_counts
        self.face_angles = [math.radians(a) for a in robot.get('face_angles_deg', [0, 120, 240])]
        self.r_in = r_in
        self._flip_ok = {}
        # Flip rule = World's: the flip-face stack fully inside our castle (robot centre may be outside).
        # A flip is planned at a node if such a pose is within flip_reach; it costs flip_extra_time.
        self.flipmap = FlipMap(field, team, r_in, self.face_angles[self.flip_face])
        # Flips only in two places (Max): during a castle drop with face 0 pointing into the castle
        # (flip_at_dock: Y1 face 2, Y2 face 1, Y5), or in the flip area around the Y5 flip spot.
        self.flip_area_r = 0.30          # flip area radius around flip_spot (m)
        self.flip_turn_time = 0.3
        # Coming from outside the castle: flip at the Y5 dock area (most room). Node 'F:flip'.
        y5 = next((z for z in self.zones if z.shape == 'rect' and z.label.endswith('5')), None)
        y5xy = self.travel.nodes['Z:' + y5.name] if y5 is not None else (0.5, 1.0)
        self.flip_spot = self.flipmap.nearest(*y5xy)[1] or (y5xy[0], y5xy[1], 0.0)
        self.travel.set_node('F:flip', self.flip_spot[:2])
        # + the flip poses by YC1 / YC4 (field yaml flip_poses:, Max): {zone idx: travel node}. A flip
        # there is only for that circle's drop (action ('flip', f, 'at', zi)).
        # discard docks (field yaml discard_docks:, Max): {zone idx: travel node}. A discard pile lies on
        # the diagonal between two circles: the robot docked at those circles would stand on it, so their
        # towers must come first (discard_blocks: {discard zone idx: {circle zone idx}}).
        self.discard_node, self.discard_blocks = {}, {}
        off = r_in + field.box_width / 2 + 0.003
        for zname, dx_, dy_, h_ in field.discard_docks.get(team, []):
            zi = next((i for i, z in enumerate(self.zones) if z.name == zname), None)
            if zi is not None:
                self.discard_node[zi] = 'D:' + self.zone_label[zi]
                self.travel.set_node(self.discard_node[zi], (dx_, dy_))
                cx, cy = dx_ + off * math.cos(h_), dy_ + off * math.sin(h_)
                self.discard_blocks[zi] = {i for i, z in enumerate(self.zones)
                                           if z.shape == 'circle' and math.hypot(z.x - cx, z.y - cy) < 0.35}
        self.zone_flip_node = {}
        for lab, pose in field.flip_poses.get(team, {}).items():
            if lab in self.zone_label:
                self.zone_flip_node[self.zone_label.index(lab)] = 'F:' + lab
                self.travel.set_node('F:' + lab, pose[:2])
        self._dock_flip = {}
        # Turning (Max): every pick / drop leg costs turn_s_per_rad x the body's heading change, even when
        # the turn fits inside the drive (turning eats wheel speed on a 3-omni-wheel robot and costs
        # precision). face_dir: node -> world direction the working face points there; the body heading
        # for face f is face_dir - face_angles[f], so faces get compared by how much they turn.
        self.turn_s_per_rad = 0.25 / (math.pi / 2)
        self.face_dir = {}
        for si, st in enumerate(self.stacks):
            try:
                self.face_dir[self.stack_nodes[si]] = field.pick_spots(st, r_in)[0].face_heading
            except (IndexError, AttributeError):
                pass
        for zi, z in enumerate(self.zones):
            try:
                self.face_dir[self.zone_nodes[zi]] = (z.dock[2] if z.dock is not None
                                                      else field.drop_spots(z, r_in)[0].face_heading)
            except (IndexError, AttributeError):
                pass
        for zname, dx_, dy_, h_ in field.discard_docks.get(team, []):
            zi = self.zone_idx.get(zname)
            if zi in self.discard_node:
                self.face_dir[self.discard_node[zi]] = h_
        self.flip_pose_head = {zi: field.flip_poses[team][self.zone_label[zi]][2]
                               for zi in self.zone_flip_node} if self.zone_flip_node else {}
        self.deny_w = 1.0         # points per box taken that the opponent would have used (set by the brain)
        self.deny_horizon = 50.0  # ...i.e. it would have emptied that stack before this match time
        self.scen = None          # ScenarioSet: stacks the opponent probably emptied are not pickable
        self.scen_p = 0.85        # ...when P(taken by our arrival) >= this (risk below that is priced by the
                                  #    scenario re-ranking: a failed pick loses time and points there)
        # Stacks right next to the opponent's (forbidden) castle: easy and safe early, awkward later
        # (its piles and robot crowd the approach). risk = w * (smoothstep((t - t0)/(t1 - t0)) - 0.5)
        # points per pick: a bonus before t0, a cost after t1.
        self.castle_w, self.castle_t0, self.castle_t1, self.castle_near = 6.0, 8.0, 45.0, 0.45
        # Stacks next to the opponent castle (P9, P10) may only be picked before the castle becomes
        # forbidden (keepout_from_s): after that their pick pose is out of bounds (Max).
        self.keepout_from = 20.0
        # Contact risk: picking where the opponent is (or will be) is penalised so the plan picks elsewhere.
        self.contact_w, self.contact_r, self.contact_now_s = 6.0, 0.9, 4.0
        self.opp_now = None       # (t, x, y, vx, vy) of the opponent when it is tracked
        self.opening, self.opening_bonus = (), 0.0     # preferred first picks (stack indices, in order)
        self.unseen, self.unseen_cost = set(), 3.0   # enemy piles assumed (not seen): picking one may find it empty
        self.deny_tau = 15.0      # denial fades with how long after our arrival the opponent would have taken it
        # zone idx -> the face the calibrated dock pose drops with (None: any face)
        self.dock_face = {self.zone_idx[n]: f for n, f in (dock_faces or {}).items()}
        # In-place floor regrip ('regrip', f): drop face f's stack on the floor where the robot stands,
        # back out, turn, repick it with the flip face. Allowed anywhere with room to back out and turn
        # (regrip_clear from walls / stacks / zones, regrip_castle_margin from both castles). Costs
        # regrip_time and regrip_risk points (the opponent may hit the pile), so the search only uses
        # it when it pays.
        self.regrip_on = True
        self.regrip_time = 2.5            # back out + turn + step back in (s), on top of drop + pick times
        self.regrip_risk = 2.0            # points
        self.regrip_clear = 0.30          # robot centre -> wall / stack / zone (m)
        self.regrip_castle_margin = 0.30  # robot centre -> either castle (m)
        # Towers are a long trip (Max): go there with the other faces full. Points charged per tower
        # drop per box missing on the non-flip faces, while the table still has a stack to pick.
        self.tower_partial_cost = 1.0
        self._regrip_ok = {}

    def flip_detour(self, node):
        """Distance (m) from `node` to the Y5 flip spot."""
        x, y = self.travel.nodes[node]
        return math.hypot(x - self.flip_spot[0], y - self.flip_spot[1])

    def flip_ok(self, node, dock_f=None):
        """Is `node` inside the flip area (around the Y5 flip spot)?"""
        return self.flip_detour(node) <= self.flip_area_r

    def flip_at_dock(self, node, dock_f):
        """Docked at zone `node` dropping with face dock_f (not the flip face): is the flip-face stack
        inside the castle in that pose (+-5 deg)? Then the flip runs during the drop, for free
        (Y1 with face 2, Y2 with face 1, Y5 with any)."""
        if dock_f < 0 or dock_f == self.flip_face or not node.startswith('Z:'):
            return False
        k = (node, dock_f)
        v = self._dock_flip.get(k)
        if v is None:
            x, y = self.travel.nodes[node]
            z = self.zones[self.zone_nodes.index(node)]
            th = math.atan2(z.y - y, z.x - x) - self.face_angles[dock_f]
            v = all(self.flipmap.ok_at(x, y, th + d) for d in (-0.09, 0.0, 0.09))
            self._dock_flip[k] = v
        return v

    def flip_at_next_dock(self, node, f):
        """Docking at rect zone `node` with face f (any face, incl. the flip face): is the flip-face
        stack inside the castle (+-5 deg)? Then a flip can run while docking there."""
        if not node.startswith('Z:'):
            return False
        z = self.zones[self.zone_nodes.index(node)]
        if z.shape != 'rect':
            return False
        k = ('any', node, f)
        v = self._dock_flip.get(k)
        if v is None:
            x, y = self.travel.nodes[node]
            th = math.atan2(z.y - y, z.x - x) - self.face_angles[f]
            v = all(self.flipmap.ok_at(x, y, th + d) for d in (-0.09, 0.0, 0.09))
            self._dock_flip[k] = v
        return v

    def flip_extra_time(self, node):
        """Turning (and the short detour, there and back) to flip near `node`."""
        return self.flip_turn_time + 2.0 * self.flip_detour(node) / self.travel.vmax

    def regrip_ok(self, node, s):
        """Room for an in-place floor regrip at `node`'s position, given the state's stack / pile counts?
        Empty stacks and empty own zones don't block (nothing to hit there)."""
        xy = self.travel.nodes.get(node)
        if xy is None:
            return False
        k = (round(xy[0], 2), round(xy[1], 2))
        info = self._regrip_ok.get(k)
        if info is None:
            info = self._regrip_free(xy)
            self._regrip_ok[k] = info
        ok, near_stacks, near_zones = info
        return (ok and not any(s.stacks[i] > 0 for i in near_stacks)
                and not any(s.zbox[i] > 0 for i in near_zones))

    def _regrip_free(self, xy):
        """(static_ok, stacks too close, own zones too close) for a regrip at xy. Static: walls,
        both castles, opponent zones (unknown piles)."""
        import numpy as np
        f, R = self.field, self.regrip_clear
        x, y = xy
        p = np.array([x, y])

        def near(poly, r):
            return _point_in_polygon((x, y), poly) or closest_point_on_poly(p, np.asarray(poly))[1] < r

        if x < R or y < R or x > f.length - R or y > f.width - R:
            return False, (), ()
        if any(near(c, self.regrip_castle_margin) for c in f.castle.values() if c is not None and len(c)):
            return False, (), ()
        if any(near(z.poly, R) for z in f.zones if z.name not in self.zone_idx):
            return False, (), ()
        stacks = tuple(i for i, st in enumerate(self.stacks) if not st.steal_zone and near(st.poly, R))
        zones = tuple(i for i, z in enumerate(self.zones) if near(z.poly, R))
        return True, stacks, zones

    def near_opp_castle(self, si):
        if self._near_castle is None:
            ko = self.field.keepout_poly(self.team)
            self._near_castle = []
            for node in self.stack_nodes:
                x, y = self.travel.nodes[node]
                if node[2:] in self._steal_names:      # stealing from the castle is deliberate, not a risk
                    self._near_castle.append(False)
                    continue
                d = min(_seg_dist((x, y), ko[i], ko[(i + 1) % len(ko)]) for i in range(len(ko))) if len(ko) else 9.0
                self._near_castle.append(d <= self.castle_near)
        return self._near_castle[si]

    def opening_gain(self, s, si):
        """Points for following the chosen opening: pick k must be opening[k] and the earlier picks must match."""
        k = len(s.picked)
        if k < len(self.opening) and si == self.opening[k] and tuple(s.picked) == tuple(self.opening[:k]):
            return self.opening_bonus
        return 0.0

    def contact_risk(self, si, t):
        """Points lost to a likely meeting with the opponent at stack si at time t: its tracked position
        (projected a few seconds ahead) blended into the scenario mixture."""
        if self.contact_w <= 0:
            return 0.0
        x, y = self.travel.nodes[self.stack_nodes[si]]
        R = self.contact_r
        p = 0.0
        if self.scen is not None:
            for sc in self.scen.scenarios:
                px, py = sc.pos_at(t)
                p += sc.prob * max(0.0, 1.0 - math.hypot(px - x, py - y) / R)
        if self.opp_now is not None:
            t0, ox, oy, vx, vy = self.opp_now
            dt = max(0.0, t - t0)
            w = max(0.0, 1.0 - dt / self.contact_now_s)
            if w > 0:
                px, py = ox + vx * min(dt, 2.0), oy + vy * min(dt, 2.0)
                p = w * max(0.0, 1.0 - math.hypot(px - x, py - y) / R) + (1 - w) * p
        return self.contact_w * p

    def castle_risk(self, si, t):
        if not self.near_opp_castle(si):
            return 0.0
        u = min(1.0, max(0.0, (t - self.castle_t0) / (self.castle_t1 - self.castle_t0)))
        return self.castle_w * (u * u * (3 - 2 * u) - 0.5)

    def initial_state(self, t=0.0, node='home'):
        n = self.n_faces
        counts = self._stack_counts or [self.field.boxes_per_stack] * len(self.stacks)
        z = len(self.zones)
        return PlanState(t, node, (0,) * n, (False,) * n, tuple(counts),
                         (0,) * z, (0,) * z, (0,) * z,
                         head=self.field.start_headings.get(self.team) if node == 'home' else None)

    def head_at(self, node, f):
        """Body heading at `node` working with face f (None: unknown node)."""
        d = self.face_dir.get(node)
        return None if d is None else d - self.face_angles[f]

    def turn_time(self, h0, h1):
        if h0 is None or h1 is None or self.turn_s_per_rad <= 0:
            return 0.0
        return self.turn_s_per_rad * abs((h1 - h0 + math.pi) % (2 * math.pi) - math.pi)


def _seg_dist(p, a, b):
    ax, ay, bx, by = float(a[0]), float(a[1]), float(b[0]), float(b[1])
    dx, dy = bx - ax, by - ay
    u = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - (ax + u * dx), p[1] - (ay + u * dy))


def _denial(m, si, t):
    """Per-box value of taking stack si at time t: for each scenario that would empty it after we
    arrive (and within the horizon), prob * exp(-(take - t) / tau). A stack the opponent is about to
    use is worth denying; one it would only reach much later (or never) is worth little."""
    if m.scen is None or m.deny_w <= 0:
        return 0.0
    return m.deny_w * sum(sc.prob * math.exp(-(sc.take[si] - t) / m.deny_tau)
                          for sc in m.scen.scenarios if t < sc.take[si] <= m.deny_horizon)


def _set(tup, i, v):
    return tup[:i] + (v,) + tup[i + 1:]


def apply(m: PlanModel, s: PlanState, action, guard=None, check_home=True,
          check_scen=True) -> Optional[PlanState]:
    """Pure transition. None if illegal (rules, capacity, or can't get home in time).
    The cheap legality checks run first, the rules (guard) only on what passes them: ~90% of the
    tried actions are plainly illegal, and the guard was ~40% of the planning time."""
    ns = _apply(m, s, action, check_home, check_scen)
    if ns is None or (guard is not None and not guard(s, action)):
        return None
    return ns


def _apply(m: PlanModel, s: PlanState, action, check_home=True, check_scen=True) -> Optional[PlanState]:
    if s.parked:
        return None
    kind = action[0]
    if s.flip_debt and kind != 'drop':
        return None
    if s.flip_for >= 0 and not (kind == 'drop' and action[1] == s.flip_for and action[2] == m.flip_face):
        return None                  # flipped at a circle's flip pose: that circle's drop comes next
    if kind == 'home':
        t = s.t + m.tt(s.node, 'home')
        if t > m.duration:
            return None
        return replace(s, t=t, node='home', parked=True, last='home', dock_f=-1)

    if kind == 'flip':
        f = action[1]
        if f != m.flip_face or s.faces[f] == 0:
            return None
        if len(action) > 3 and action[2] == 'at':
            # flip at circle zone zi's own flip pose (YC1/YC4), right before its drop
            zi = action[3]
            node = m.zone_flip_node.get(zi)
            if node is None or s.flip_debt or s.vert[f] or s.zbox[zi] > 0:
                return None
            ns = replace(s, t=s.t + m.tt(s.node, node) + m.flip_time, node=node, vert=_set(s.vert, f, True),
                         last='flip', flip_debt=False, flip_for=zi, dock_f=-1, regrip_flip=False,
                         head=m.flip_pose_head.get(zi, s.head))
            return ns if _can_finish(m, ns, check_home) else None
        if len(action) > 2 and action[2] == 'dock':
            # flip while docking for the next drop: settled (and timed) by _drop
            if s.flip_debt or not _unflip_ok(m, s, f):
                return None
            ns = replace(s, vert=_set(s.vert, f, not s.vert[f]), last='flip', flip_debt=True, regrip_flip=False)
            return ns if _can_finish(m, ns, check_home) else None
        if not _unflip_ok(m, s, f):
            return None
        # Where the flip happens (World: the flip-face stack must be inside our castle):
        #  - just dropped at a castle dock with another face, face 0 pointing into the castle
        #    (Y1 face 2, Y2 face 1, Y5): during that drop, free;
        #  - already in the flip area around the Y5 flip spot: a short move / turn;
        #  - otherwise: drive to the Y5 flip spot ('F:flip') and flip there.
        if s.last == 'drop' and m.flip_at_dock(s.node, s.dock_f):
            t, node = s.t, s.node
        elif m.flip_ok(s.node):
            t, node = s.t + m.flip_time + m.flip_extra_time(s.node), s.node
        else:
            t, node = s.t + m.tt(s.node, 'F:flip') + m.flip_time, 'F:flip'
        ns = replace(s, t=t, node=node, vert=_set(s.vert, f, not s.vert[f]), last='flip', flip_debt=False,
                     dock_f=s.dock_f if node == s.node else -1, regrip_flip=False)
        return ns if _can_finish(m, ns, check_home) else None

    if kind == 'discard':
        zi, f = action[1], action[2]
        node = m.discard_node.get(zi)
        if node is None or s.faces[f] == 0 or s.zkind[zi] != WALL:
            return None
        if s.regrip_flip and f == m.flip_face and not s.vert[f]:
            return None
        # Only at the end (Max): every pick stack gone (whoever took it) and the towers next to this
        # discard spot already built - a discard is for over-picked leftovers, not a way to score early
        if any(c > 0 for c, st in zip(s.stacks, m.stacks) if not st.steal_zone):
            return None
        if any(s.zbox[c] == 0 for c in m.discard_blocks.get(zi, ())):
            return None
        ff = m.flip_face
        if not s.door_used and s.door_zone < 0:
            # Never throw away what could still be the door (Max): a face 1 / 2 stack only once the
            # door's stack is already on the flip face (steal first, discard after: an empty enemy
            # pile then still leaves this stack for the door, via the floor regrip)...
            if f != ff and s.faces[ff] < 3:
                return None
            # ...and never the flip face's own leftovers: the door's replacement stack has to be ON the
            # robot before anything is thrown away (Max: seen full is not enough, it must be taken), and
            # the flip face must be empty to take it
            if f == ff:
                return None
        n = min(m.field.discard_max_boxes - s.zbox[zi], s.faces[f])   # standing boxes: all in one action
        if n <= 0:
            return None
        h1 = m.head_at(node, f)
        ns = replace(s, t=s.t + (m.tt(s.node, node) + m.drop_overhead + m.drop_time) * m.drop_factor
                     + m.turn_time(s.head, h1), node=node, head=h1,
                     faces=_set(s.faces, f, s.faces[f] - n), zbox=_set(s.zbox, zi, s.zbox[zi] + n),
                     vert=_set(s.vert, f, s.vert[f] and s.faces[f] > n),
                     last='discard', dock_f=-1, discarded=s.discarded + (zi,),
                     bonus=s.bonus + (s.fbonus[f] if s.fbonus else 0.0) * n)
        return ns if _can_finish(m, ns, check_home) else None

    if kind == 'regrip':
        f, ff = action[1], m.flip_face
        if (not m.regrip_on or f == ff or s.faces[f] == 0 or s.vert[f] or s.faces[ff] > 0 or s.vert[ff]
                or not m.regrip_ok(s.node, s)):
            return None
        if any(c > 0 for c, st in zip(s.stacks, m.stacks) if not st.steal_zone):
            # a table stack can still be picked straight onto the (empty) flip face: same boxes for the
            # towers / door, no floor drop (Max: P6 regripped at 4 s while P8 was next anyway)
            return None
        n = s.faces[f]
        fb = s.fbonus or (0.0,) * m.n_faces
        ns = replace(s, t=s.t + m.drop_time + m.regrip_time + m.engage + n * m.per_box,
                     faces=_set(_set(s.faces, f, 0), ff, n), fbonus=_set(_set(fb, f, 0.0), ff, fb[f]),
                     last='regrip', dock_f=ff, risk_cost=s.risk_cost + m.regrip_risk, regrip_flip=True,
                     head=None if s.head is None else s.head + m.face_angles[f] - m.face_angles[ff],
                     stolen=_set(s.stolen or (False,) * m.n_faces, ff, False))
        return ns if _can_finish(m, ns, check_home) else None

    if kind == 'pick':
        si, f = action[1], action[2]
        free = m.capacity - s.faces[f]
        # held boxes block the stack, so a pick needs an empty face
        if (s.vert[f] or s.faces[f] > 0 or free <= 0
                or (s.stacks[si] <= 0 and si not in m.regrip and si not in m.own_repick)):
            return None
        sz = m.stacks[si].steal_zone
        zo = m.own_repick.get(si)
        if zo is not None:
            return _own_repick(m, s, si, f, zo, check_home)
        zr = m.regrip.get(si)
        if zr is not None:                       # repick our own horizontal pile from the flip face
            if f != m.flip_face or s.zvert[zr] > 0 or s.zkind[zr] == DOOR:
                return None
            # only worth it right after dropping from ANOTHER face (the stack must end up on the flip face)
            if s.last != 'drop' or not s.dropped or s.dropped[-1] != zr or s.dock_f == m.flip_face:
                return None
            n = min(free, s.zbox[zr])
            if n <= 0 or s.faces[f] > 0:
                return None
            node = m.stack_nodes[si]
            h1 = m.head_at(node, f)
            arrive = s.t + m.tt(s.node, node) + m.pick_overhead * m.pick_factor + m.turn_time(s.head, h1)
            left = s.zbox[zr] - n
            d = list(s.dropped)
            if zr in d:
                d.reverse(); d.remove(zr); d.reverse()
            ns = replace(s, t=arrive + m.engage + n * m.per_box, node=node, faces=_set(s.faces, f, n),
                         zbox=_set(s.zbox, zr, left),
                         zkind=_set(s.zkind, zr, WALL if left >= m.zones[zr].capacity else NONE),
                         dropped=tuple(d), last='pick', dock_f=f, head=h1,
                         fbonus=_set(s.fbonus or (0.0,) * m.n_faces, f, 0.0),
                         stolen=_set(s.stolen or (False,) * m.n_faces, f, False))
            return ns if _can_finish(m, ns, check_home) else None
        if sz and (f != 0 or m.field.zone(sz).team == m.team):   # enemy drop-zone piles: face 0 only
            return None
        n = min(free, s.stacks[si])
        node = m.stack_nodes[si]
        h1 = m.head_at(node, f)
        arrive = (s.t + m.tt(s.node, node) + m.pick_overhead * m.pick_factor   # factor: docking only, not travel
                  + m.turn_time(s.head, h1))
        if not sz and m.near_opp_castle(si) and arrive + m.engage + n * m.per_box > m.keepout_from:
            return None                          # P9/P10 only while the opponent castle is not yet forbidden
        if check_scen and m.scen is not None and m.scen.p_taken(si, arrive + m.engage) >= m.scen_p:
            return None
        t = arrive + m.engage + n * m.per_box
        if any(s.stacks[sj] > 0 for sj in m.crowd.get(si, ())):
            t += m.crowded_pick_s
        ns = replace(s, t=t, node=node, faces=_set(s.faces, f, s.faces[f] + n),
                     stacks=_set(s.stacks, si, s.stacks[si] - n),
                     picked=s.picked + (si,), last='pick', dock_f=f, head=h1,
                     fbonus=_set(s.fbonus or (0.0,) * m.n_faces, f, _denial(m, si, arrive + m.engage)),
                     stolen=_set(s.stolen or (False,) * m.n_faces, f, bool(sz)),
                     risk_cost=s.risk_cost + m.castle_risk(si, arrive + m.engage)
                     + m.contact_risk(si, arrive + m.engage) + (m.unseen_cost if si in m.unseen else 0.0)
                     - m.opening_gain(s, si))
        return ns if _can_finish(m, ns, check_home) else None

    if kind == 'drop':
        return _drop(m, s, action[1], action[2], check_home)
    raise ValueError(action)


def _unflip_ok(m, s, f):
    """Standing boxes are laid back down only for the door lintel (posts already in), or when no
    standing use is left: no free circle for a tower, and no door posts possible from this face.
    Otherwise a flip gets undone for a wall (Max: P6 flipped at 13 s, flipped back at 19 s for Y1;
    then one tower and the 2 leftovers un-flipped into Y2 instead of 3 towers)."""
    if not s.vert[f]:
        return True
    if s.door_zone >= 0 and not s.door_used:
        return True                                       # the lintel
    blocked = set()
    for d in s.discarded:
        blocked |= m.discard_blocks.get(d, set())
    if any(z.shape == 'circle' and s.zbox[i] == 0 and i not in blocked for i, z in enumerate(m.zones)):
        return False                                      # a tower is still possible
    return s.door_used or s.faces[f] < m.capacity         # a full standing stack still makes the door posts


def _own_repick(m, s, si, f, zi, check_home):
    """Take our own Y3 / Y4 wall back with the flip face for the door (Max): 3 + 20 for the door against
    3 + 5 for the wall. Only as a last resort: no table stack left, no enemy pile to steal, no 3 boxes
    held (or parked in Y5) for the door, door not started."""
    if f != m.flip_face or s.faces[f] > 0 or s.vert[f] or s.door_used or s.door_zone >= 0:
        return None
    if s.zkind[zi] != WALL or s.zvert[zi] > 0 or s.zbox[zi] < m.capacity:
        return None
    if any(c > 0 for c, st in zip(s.stacks, m.stacks) if not st.steal_zone):
        return None                                       # table stacks first
    if any(c > 0 for c, st in zip(s.stacks, m.stacks) if st.steal_zone and st.steal_zone not in m.zone_idx):
        return None                                       # an enemy pile to steal first
    if any(n >= m.capacity for n in s.faces) or any(s.zbox[z] >= m.capacity for z in m.regrip.values()):
        return None                                       # 3 boxes for the door already held / parked in Y5
    n = m.capacity
    node = m.stack_nodes[si]
    h1 = m.head_at(node, f)
    arrive = s.t + m.tt(s.node, node) + m.pick_overhead * m.pick_factor + m.turn_time(s.head, h1)
    ns = replace(s, t=arrive + m.engage + n * m.per_box, node=node, faces=_set(s.faces, f, n),
                 zbox=_set(s.zbox, zi, s.zbox[zi] - n), zkind=_set(s.zkind, zi, NONE),
                 last='pick', dock_f=f, head=h1,
                 fbonus=_set(s.fbonus or (0.0,) * m.n_faces, f, 0.0),
                 stolen=_set(s.stolen or (False,) * m.n_faces, f, False))
    return ns if _can_finish(m, ns, check_home) else None


def _drop(m, s, zi, f, check_home):
    if s.faces[f] == 0:
        return None
    if s.regrip_flip and f == m.flip_face and not s.vert[f]:
        return None            # regripped for a flip: laying it down as it is would waste the regrip
    if s.flip_debt and not m.flip_at_next_dock(m.zone_nodes[zi], f):
        return None            # the flip can't run while docking here
    zone = m.zones[zi]
    total, vcount = s.zbox[zi], s.zvert[zi]
    hcount = total - vcount
    fv = s.vert[f]
    kind, door_used, door_zone = s.zkind[zi], s.door_used, s.door_zone
    if (zone.label.endswith('5') and zone.shape == 'rect' and not s.door_used and s.t < m.door_earliest
            and any(c > 0 for c, st in zip(s.stacks, m.stacks) if not st.steal_zone)):
        return None              # the door ends our match: not early while table stacks are still there (Max)
    if zone.shape == 'circle':
        if not fv or total >= 1:
            return None
        if any(zi in m.discard_blocks.get(d, ()) for d in s.discarded):
            return None          # a discard pile already lies where the robot docks for this tower
        n = 1
    elif fv:                                  # door posts
        if hcount > 0 or vcount >= 2 or door_used or door_zone not in (-1, zi):
            return None
        n = min(2 - vcount, s.faces[f])
        door_zone = zi
    else:
        if vcount == 1 or (vcount == 2 and f != m.flip_face):   # lid must be the elevated flip-face box
            return None
        limit = m.field.zone_max_boxes if kind else zone.capacity
        if kind == DOOR and not m.field.door_accepts_extras:
            limit = total
        space = limit - total
        if space <= 0:
            return None
        n = 1 if vcount == 2 else min(space, s.faces[f])
    new_total = total + n
    if kind == NONE:
        if zone.shape == 'circle':
            kind = TOWER
        elif vcount == 2 and not fv:
            kind, door_used = DOOR, True
        elif not fv and new_total >= zone.capacity:
            kind = WALL
    left = s.faces[f] - n
    node = m.zone_nodes[zi]
    h1 = m.head_at(node, f)
    ns = replace(
        s, t=s.t + (m.tt(s.node, node) + m.drop_overhead + m.drop_time) * m.drop_factor
        + (m.flip_time if s.flip_debt else 0.0) + m.turn_time(s.head, h1), node=node, head=h1,
        faces=_set(s.faces, f, left), vert=_set(s.vert, f, fv and left > 0),
        zbox=_set(s.zbox, zi, new_total), zvert=_set(s.zvert, zi, vcount + n if fv else vcount),
        zkind=_set(s.zkind, zi, kind), door_used=door_used, door_zone=door_zone,
        dropped=s.dropped + (zi,), last='drop', dock_f=f, flip_debt=False, flip_for=-1,
        bonus=s.bonus + (s.fbonus[f] if s.fbonus else 0.0) * n,
        risk_cost=s.risk_cost + _tower_partial(m, s, zone, f),
        slack=(max(s.slack, m.steal_tower_slack)
               if zone.shape == 'circle' and s.stolen and s.stolen[f] and not s.door_used else s.slack))
    return ns if _can_finish(m, ns, check_home) else None


def _tower_partial(m, s, zone, f):
    """Cost of a tower drop made while a non-flip face still has room and boxes are left to pick."""
    if zone.shape != 'circle' or m.tower_partial_cost <= 0:
        return 0.0
    missing = sum(m.capacity - s.faces[g] for g in range(len(s.faces)) if g != f)
    if missing <= 0 or not any(c > 0 for c, st in zip(s.stacks, m.stacks) if not st.steal_zone):
        return 0.0
    return m.tower_partial_cost * missing


def _can_finish(m, s, check_home):
    return (not check_home) or s.t + m.tt(s.node, 'home') + m.home_reserve + s.slack <= m.duration


def legal_actions(m: PlanModel, s: PlanState, guard=None):
    """(action, next_state) pairs. Faces in identical (count, vertical) state
    are interchangeable, so only the first of each kind is generated."""
    out = []
    if s.parked:
        return out
    groups = {}
    for f in range(m.n_faces):
        k = ((s.faces[f], s.vert[f]) if f != m.flip_face and f not in m.dock_face.values()
             else (f, s.faces[f], s.vert[f]))
        groups.setdefault(k, []).append(f)

    def best(fs, node):
        # equivalent faces: the one whose dock heading is closest to the current heading (Max: turn less)
        if len(fs) == 1 or s.head is None:
            return fs[0]
        return min(fs, key=lambda g: m.turn_time(s.head, m.head_at(node, g)))

    for fs in groups.values():
        f = fs[0]
        for si in range(len(m.stacks)):
            if (s.stacks[si] > 0 or (si in m.regrip and s.zbox[m.regrip[si]] > 0)
                    or (si in m.own_repick and s.zbox[m.own_repick[si]] > 0)):
                a = ('pick', si, best(fs, m.stack_nodes[si]))
                ns = apply(m, s, a, guard)
                if ns:
                    out.append((a, ns))
        for zi in range(len(m.zones)):
            if m.dock_face.get(zi, f) != f:
                continue
            a = ('drop', zi, best(fs, m.zone_nodes[zi]))
            ns = apply(m, s, a, guard)
            if ns:
                out.append((a, ns))
        for zi in m.discard_node:
            a = ('discard', zi, best(fs, m.discard_node[zi]))
            ns = apply(m, s, a, guard)
            if ns:
                out.append((a, ns))
        for a in (('flip', f), ('flip', f, 'dock')) + tuple(('flip', f, 'at', zi) for zi in m.zone_flip_node):
            ns = apply(m, s, a, guard)
            if ns:
                out.append((a, ns))
        if f != m.flip_face:
            a = ('regrip', f)
            ns = apply(m, s, a, guard)
            if ns:
                out.append((a, ns))
    a = ('home',)
    ns = apply(m, s, a, guard)
    if ns:
        out.append((a, ns))
    return out


def points(m: PlanModel, s: PlanState, include_park=True):
    """Score of the state; boxes still carried count 0, park needs `parked`."""
    sc = m.scoring
    total = sum(s.zbox) * sc.get('box_in_zone', 0)
    for k in s.zkind:
        if k:
            total += sc.get(_KIND_KEY[k], 0)
    if include_park and s.parked:
        total += sc.get('robot_in_end_zone', 0)
    return total
