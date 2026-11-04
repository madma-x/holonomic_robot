"""StrategyCore: the sensor-realistic match-strategy brain.

Implements the table_sim.headless --brain contract: step(obs, dt) -> Command,
where this module defines Observation and Command (see table_sim/headless.py's
docstring for the exact field list). Never reads ground truth - only what
Observation carries (own pose, lidar ranges, camera detections, last tool
result, match clock) and what the robot has itself commanded/observed,
exactly like a real ROS node limited to /odom, /scan, /camera/detections,
/face_tool results and /game/time_remaining would be.

Uses all 3 faces (9 boxes) before switching from picking to dropping, so one
"collecting" run covers up to 3 stacks instead of a pick/drop/pick/drop
shuttle - `self._mode` hysteresis ('collecting' <-> 'delivering') is what
prevents immediately re-picking into a face freed by the first drop of a
multi-box delivery run.

Navigation is split in two: the coarse leg to a pre-approach point (or home)
is exposed via Command.goal_pose for a Nav2-driving node to handle (real
costmap-based planning/avoidance); the final, precise approach right up to
the stack/zone (tight tolerance, exact heading needed for the tool to reach)
stays under this class's own direct kinematic control (Command.vx/vy/wz,
goal_pose=None) - Nav2's own goal tolerance (table_sim's
nav2_params_sim.yaml: 0.05m / any heading) is too loose for that last bit.
headless.py has no Nav2 at all, so it always uses vx/vy/wz and ignores
goal_pose - this class's own GridPlanner-based follower (used to compute
vx/vy/wz regardless of stage) is what makes that path work standalone.

v1 does not use the flip/door mechanic (vertical boxes, circle zones) - a
scoring optimization on a working baseline, natural next step.
"""

import json
import math
import os

from table_sim.field import Spot
from table_sim.geometry import rect_poly
from table_sim.gridplan import GridPlanner

from .belief import StackBelief
from .opponent_tracker import OpponentTracker


class Observation:
    __slots__ = ('t', 'active', 'time_left', 'pose', 'ranges', 'angle_min',
                 'angle_inc', 'lidar_yaw', 'cam', 'tool_busy', 'result',
                 'flip_busy', 'flip_result')

    def __init__(self, t, active, time_left, pose, ranges, angle_min, angle_inc,
                 lidar_yaw, cam, tool_busy, result, flip_busy=False, flip_result=None):
        self.t, self.active, self.time_left = t, active, time_left
        self.pose, self.ranges = pose, ranges
        self.angle_min, self.angle_inc, self.lidar_yaw = angle_min, angle_inc, lidar_yaw
        self.cam, self.tool_busy, self.result = cam, tool_busy, result
        # FLIP runs in its own concurrent slot (world.py) so it can overlap a
        # PICK/DROP on another face - tool_busy/result above are main-slot
        # only, these are the flip slot's own busy flag and (face, success,
        # count, message) result tuple.
        self.flip_busy, self.flip_result = flip_busy, flip_result


class Command:
    __slots__ = ('vx', 'vy', 'wz', 'cancel', 'action', 'goal_pose', 'flip_action')

    def __init__(self, vx=0.0, vy=0.0, wz=0.0, cancel=False, action=None, goal_pose=None,
                 flip_action=None):
        self.vx, self.vy, self.wz = vx, vy, wz
        self.cancel, self.action = cancel, action
        self.goal_pose = goal_pose   # (x, y, th) for a Nav2-driving node, or None = use vx/vy/wz
        self.flip_action = flip_action   # (face,) to request a concurrent FLIP, or None


class Debug:
    def __init__(self):
        self.state = 'init'
        self.path = []
        self.opp = None
        self.belief = {}
        self.log = []

    def say(self, msg):
        self.log.append(msg)
        del self.log[:-20]


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _eta(p, q, speed):
    return math.hypot(q[0] - p[0], q[1] - p[1]) / max(speed, 0.05)


class StrategyCore:
    def __init__(self, field, team, params=None):
        p = dict(params or {})
        self._params = p
        self.field = field
        self.team = team
        self.side = float(p.get('robot_side', 0.34))
        self.r_in = self.side / (2.0 * math.sqrt(3.0))
        self.face_angles = [math.radians(a) for a in p.get('face_angles_deg', [0, 120, 240])]
        self.capacity = int(p.get('boxes_per_face', 3))
        self.vmax = float(p.get('max_lin_vel', 1.0))
        self.amax = float(p.get('max_lin_acc', 1.5))
        self.wmax = float(p.get('max_ang_vel', 4.0))
        self.wacc = float(p.get('max_ang_acc', 8.0))
        # Mirrors motion_controller's own heading_blend_distance (see
        # pure_pursuit_controller.cpp): face the direction of travel while
        # still far out, blending to the exact required heading (tth) over
        # this last stretch - the SAME formula _goto uses whether it's
        # computing the coarse Nav2 leg or the final direct-controlled
        # approach, so if Nav2's own blend already got us facing tth by the
        # time it hands off, this controller's heading error starts near
        # zero instead of snapping a fresh P-controller onto whatever
        # heading the coarse leg happened to leave us at.
        self.dock_heading_blend_dist = float(p.get('dock_heading_blend_distance_m', 0.3))
        self.go_home_margin = float(p.get('go_home_margin_s', 5.0))
        # Opponent avoidance only applies to the final, direct-controlled
        # pick/drop approach (_goto's plan=False case) - the coarse legs
        # (Command.goal_pose set) are driven by Nav2, whose own local
        # costmap already reacts to the opponent live via /scan, so
        # duplicating that here would be redundant. Graduated: slow down
        # starting at avoid_slow_dist, hard stop at avoid_stop_dist.
        self.avoid_slow_dist = float(p.get('avoid_slow_dist_m', 0.7))
        self.avoid_stop_dist = float(p.get('avoid_stop_dist_m', 0.35))
        # Separate from the two above: those slow/stop us reactively while
        # *driving*. This instead keeps us from ever *committing* to a
        # coarse-leg goal (Command.goal_pose, Nav2-driven) that sits inside
        # the opponent's own footprint - Nav2's local costmap only inflates
        # the opponent live once it's within lidar range and close to the
        # robot's current rolling window, so a goal placed right on top of
        # its last-known position can still be accepted and then repeatedly
        # fail/replan instead of being rejected outright. Combined
        # circumradius of two robots (2 * side/sqrt(3)) plus margin.
        self.opponent_goal_clearance = float(
            p.get('opponent_goal_clearance_m', 2.0 * self.side / math.sqrt(3.0) + 0.1))
        self.pos_tol = float(p.get('pos_tol_m', 0.02))
        self.yaw_tol = float(p.get('yaw_tol_rad', 0.05))
        # Looser POSITION tolerance for stages handed to Nav2 (Command.goal_pose
        # set, 'approach'/'home'): must stay >= Nav2's own goal_checker
        # xy_goal_tolerance (table_sim's nav2_params_sim.yaml: 0.05m) or Nav2
        # reports "reached" while this class's own arrival check never fires -
        # nothing then drives the last few cm, and the stuck-timeout abandons
        # the target forever. Yaw is no longer loosened here: Nav2's own
        # yaw_goal_tolerance is now a real 10deg (motion_controller actually
        # enforces it - see pure_pursuit_controller.cpp's terminal yaw stage),
        # so the coarse leg now arrives at the exact heading this was sent,
        # same as the direct-controlled stages.
        self.nav_pos_tol = float(p.get('nav_pos_tol_m', 0.08))
        self.nav_yaw_tol = float(p.get('nav_yaw_tol_rad', 0.175))
        # world.py's zone_under_face only requires the held box's footprint
        # to overlap the zone AT ALL (SAT test, not full containment) -
        # position can stay loose, letting the robot drop from a wider range
        # of final positions instead of one fixed exact spot. Heading is
        # different: Max wants a dropped pile's own placed angle within
        # +-10deg of the zone's own th (not just "touching it at any angle")
        # - _spot_for's curated approach_deg is exactly zone.th+-90 so this
        # is tight by construction, not a margin to search within.
        self.drop_pos_tol = float(p.get('drop_pos_tol_m', 0.1))
        self.drop_yaw_tol = float(p.get('drop_yaw_tol_rad', math.radians(10)))
        self.backout_dist = float(p.get('backout_dist_m', 0.06))
        # Below this confidence, a pick target is "risky" enough to spend a
        # moment turning the face-0 camera toward it before committing to
        # the final approach - confirms it's still there (saving the rest
        # of the trip if not) instead of finding out only after arriving.
        self.verify_confidence = float(p.get('verify_confidence_threshold', 0.7))
        self.verify_timeout = float(p.get('verify_timeout_s', 3.0))

        self.blocked_timeout = float(p.get('blocked_timeout_s', 2.5))
        self.planner = GridPlanner(field.length, field.width, res=0.04)
        self.pose = (0.0, 0.0, 0.0)

        # Circle (V) zones only take flipped boxes, and world.py only lets
        # face `flip_face` (the one with the flip mechanism) flip at all -
        # matches table_sim/config/sim.yaml's tool.flip_face.
        self.flip_face = int(p.get('flip_face', 0))
        self.use_vertical_zones = bool(p.get('use_vertical_zones', True))

        # Exact drop dock poses Max tuned by hand in the table-reference
        # artifact's dock-pose editor (dock_poses.json: {label: {x,y,yaw_deg}},
        # circle zones as <label>_L/_C/_R - only _C is used here, since
        # world.py doesn't yet model which of the 3 vertical boxes a left/
        # right offset would actually pick up). Keyed by zone label (Y1..Y5,
        # YC1..YC4) -> an override Spot with that exact pose and the face
        # that reproduces it (same face-from-yaw inference the artifact
        # itself uses), bypassing the geometric drop_spots()/_best_face
        # computation entirely for these - Max: "use the docking position on
        # artifact."
        self._dock_override = self._load_dock_overrides(p.get('dock_poses_file'))

        self.reset()

    # Which of the 3 fixed slots (see world.py: 0=left, 1=centre, 2=right,
    # each box_width apart) a circle zone's drop should target, shifting the
    # robot off the artifact's centre dock pose by that many box-widths so
    # that slot's box - not necessarily the centre one - physically lands
    # on the zone. Defaults to centre for every circle zone; override here
    # per label once a different slot is confirmed to be the right one.
    _CIRCLE_SLOT = {}

    def _load_dock_overrides(self, path):
        if not path or not os.path.exists(path):
            return {}
        with open(path) as f:
            data = json.load(f)
        poses = data.get('poses', {})
        bw = self.field.box_width
        overrides = {}
        for key, pose in poses.items():
            if '_' in key:
                base, pos = key.rsplit('_', 1)
                if pos != 'C':   # only the artifact's centre pose is used - see _CIRCLE_SLOT
                    continue
                label = base
            else:
                label = key
            zone = next((z for z in self.field.team_zones(self.team) if z.label == label), None)
            if zone is None:
                continue
            x, y = float(pose['x']), float(pose['y'])
            yaw = math.radians(float(pose['yaw_deg']))
            # Circle zones only ever receive flipped boxes off flip_face
            # (world.py hard rule) - the working face there is always
            # flip_face, never inferred from yaw.
            face = self.flip_face if zone.shape == 'circle' else self._dock_face(x, y, yaw, zone.x, zone.y)
            if zone.shape == 'circle':
                slot = self._CIRCLE_SLOT.get(label, 1)
                if slot != 1:
                    # Shift the robot off-centre by the same box-width offset
                    # world.py's held_poly_slot uses, in the OPPOSITE
                    # direction, so that slot (not the centre one) ends up
                    # sitting where the centre pose aimed.
                    tangent_x, tangent_y = -math.sin(yaw), math.cos(yaw)
                    shift = (1 - slot) * bw
                    x, y = x + tangent_x * shift, y + tangent_y * shift
            face_heading = _wrap(yaw + self.face_angles[face])
            overrides[label] = (Spot('drop', zone.name, x, y, face_heading, x, y), face)
        return overrides

    def _dock_face(self, x, y, yaw, zx, zy):
        to_zone = math.atan2(zy - y, zx - x)
        best, best_d = 0, 1e9
        for f, fa in enumerate(self.face_angles):
            d = abs(_wrap(yaw + fa - to_zone))
            if d < best_d:
                best_d, best = d, f
        return best

    def reset(self):
        """Full state reset - belief, tracker, target, everything mutable.
        Call this whenever the match/world resets (e.g. on /sim/reset):
        nothing resets StrategyCore on its own, so without this call its
        belief (what we think we've dropped, where the opponent was, ...)
        just carries over stale from whatever match ran before."""
        self.belief = StackBelief(self.field, self.team, self._params)
        self.tracker = OpponentTracker()
        self.debug = Debug()

        self._static_key = None

        self.faces = [0] * len(self.face_angles)   # our own belief of what we're carrying, per face
        self.vertical = [False] * len(self.face_angles)  # our own belief of each face's flip state
        self._mode = 'collecting'       # 'collecting' (pick up to all faces) | 'delivering' (drop them all)
        self.target_kind = None         # 'pick' | 'drop' | 'home' | 'backout' | None
        self.target_obj = None          # StackSpec | ZoneSpec | None
        self.spot = None
        self._backout_target = None     # (x, y, th) while target_kind == 'backout'
        self._forced_face = None        # set by _decide: this pick must use this exact face
        self.face = 0                   # which of our faces this target uses
        self.stage = 'approach'         # 'approach' -> ['verify'] -> 'final'
        self.path = []
        self.v = (0.0, 0.0)
        self.wz = 0.0
        self._last_pose = None
        self._vel_est = (0.0, 0.0, 0.0)  # (vx_world, vy_world, wz) from consecutive poses
        self._prev_xy = None
        self.blocked_t = 0.0
        self._skip_until = {}           # stack name -> t when it's worth retrying
        self._drop_retry_offset = {}    # zone/stack name -> how many curated sides to rotate past
        self._verify_t0 = None

    # --------------------------------------------------------------- belief
    def occupied(self, t):
        """Stacks/zones we currently believe are occupied - used both for
        our own path planning below and (by strategy_node) to feed a Nav2
        costmap from belief instead of ground truth. Includes the
        opponent's own zones once camera has actually seen a box in one
        (collision/disqualification avoidance, not a pick/drop target)."""
        zc = self.belief.zone_counts()
        occ_zones = [z for z in self.field.team_zones(self.team) if zc.get(z.name, 0) > 0]
        occ_stacks = [s for s in self.field.stacks if self.belief.believed_count(s, t) >= 0.5]
        occ_zones += self.belief.enemy_occupied_zones()
        return occ_stacks, occ_zones

    def zone_obstacle_pose(self, z):
        """(x, y, th) to use for z's obstacle - our own real drop pose
        (self-knowledge) once we've actually dropped there, else the
        zone's idealized preset centre (nothing dropped there yet by us,
        or it's an opponent zone we only know about via camera)."""
        return self.belief.zone_pose(z.name) or (z.x, z.y, z.th)

    def _rebuild_statics(self, t):
        occ_stacks, occ_zones = self.occupied(t)
        key = tuple(sorted(s.name for s in occ_stacks) + sorted(z.name for z in occ_zones))
        if key == self._static_key:
            return
        zone_polys = []
        for z in occ_zones:
            zx, zy, zth = self.zone_obstacle_pose(z)
            # A flipped box standing on end (circle zones only ever hold
            # vertical boxes) foots box_width x box_width, not the
            # lying-flat box_length x box_width footprint - same distinction
            # world.py's held_poly/pile rendering already makes.
            bl = self.field.box_width if z.shape == 'circle' else self.field.box_length
            zone_polys.append(rect_poly(zx, zy, bl, self.field.box_width, zth))
        polys = [s.poly for s in occ_stacks] + zone_polys
        self.planner.set_static(key, polys, self.r_in + 0.02)
        self._static_key = key

    # -------------------------------------------------------------- decide
    def _best_face(self, face_heading, want, avoid=()):
        """Which of our faces to use: nearest in heading to minimise turning,
        preferring to top up an already-partly-loaded face over an empty one
        when picking, and emptying the fullest face first when dropping -
        same tie-break ScriptedOpponent._best_face uses. `d` is the actual
        rotation needed from our CURRENT heading (not just an absolute
        offset between face_heading and the face's body-frame angle, which
        ignored where we're currently facing and so didn't reliably pick
        the face needing the least turning). `avoid`: faces to only use if
        nothing else qualifies (flip_face while it's reserved for a pending
        circle-zone drop - see _circle_zones_pending)."""
        th = self.pose[2]
        best, best_d = None, 1e9
        for f, a in enumerate(self.face_angles):
            n = self.faces[f]
            if want == 'free' and (n >= self.capacity or self.vertical[f]):
                # A vertical face can't be picked onto at all (world.py:
                # "drop all boxes first") even with boxes still left to
                # drop - not just "full", genuinely unusable for picking
                # until it's back to empty/horizontal.
                continue
            if want == 'loaded' and n == 0:
                continue
            d = abs(_wrap(face_heading - a - th))
            if want == 'free':
                d += 0.5 * (n == 0)
            if want == 'loaded':
                d -= 0.5 * n
            if f in avoid:
                d += 100.0
            if d < best_d:
                best, best_d = f, d
        return best

    def _circle_zones_pending(self):
        """True if some circle (V) zone of ours still has capacity left -
        while true, flip_face's load should be reserved for it rather than
        spent on a generic rect (H) drop (see the 'loaded' face-selection
        call sites)."""
        if not self.use_vertical_zones:
            return False
        counts = self.belief.zone_counts()
        # Circles hold up to 3 now (world.py) but the current choreography
        # only wants 1 each before moving on (belief.py's best_drop uses the
        # same target=1) - match that here so flip_face isn't reserved any
        # longer than it actually needs to be.
        return any(z.shape == 'circle' and not self.belief.is_excluded(z) and counts.get(z.name, 0) < 1
                   for z in self.field.team_zones(self.team))

    def _opponent_blocks(self, gx, gy):
        """True if a believed-current opponent position sits too close to
        (gx, gy) to safely commit to it as a coarse-leg Nav2 goal. Only
        trusts a *fresh* sighting (tracker.opp.visible) - a stale, decayed
        estimate (sigma grown large since last seen) is as likely to be
        wrong as right, and blacklisting every stack/zone near wherever the
        opponent was last glimpsed would starve the strategy for no benefit
        since Nav2's own live costmap already handles the stale case fine."""
        opp = self.tracker.opp
        if not opp.visible:
            return False
        return math.hypot(gx - opp.x, gy - opp.y) < self.opponent_goal_clearance

    def _decide(self, obs):
        t, time_left = obs.t, obs.time_left
        hx, hy, _ = self.field.start_pose(self.team)
        eta_home = _eta(self.pose[:2], (hx, hy), self.vmax) + self.go_home_margin
        carried = sum(self.faces)
        # A vertical face can't be picked onto regardless of count (world.py:
        # "drop all boxes first") - only count faces that are actually
        # pickable, not just under capacity.
        free_face = any(n < self.capacity and not v for n, v in zip(self.faces, self.vertical))
        self._forced_face = None

        if time_left < eta_home:
            return 'home', None

        skip = {name for name, until in self._skip_until.items() if until > t}

        if carried == 0:
            self._mode = 'collecting'
        elif not free_face:
            self._mode = 'delivering'
        elif self._mode == 'delivering' and time_left >= eta_home + 10.0:
            # A face freed up mid-delivery (e.g. just dropped Y1/Y3) while
            # we're still carrying elsewhere (flip_face mid-feeding circles).
            # Max's choreography picks P1/P3 right here, before finishing the
            # rest of the drops - interleave collecting in whenever the next
            # fixed-order pick is still available, instead of draining
            # everything held before picking again.
            if self.belief.next_pick(exclude=skip) is not None:
                self._mode = 'collecting'

        if self._mode == 'collecting' and time_left >= eta_home + 10.0:
            s = self.belief.next_pick(exclude=skip)
            if s is not None:
                # If this stack is pinned to feed a circle zone (P1 -> YC1,
                # P2 -> YC4), land it on flip_face from the start - picking
                # it onto whichever face is heading-optimal right now (the
                # generic rule) can strand it somewhere that can never be
                # flipped, with nothing left at the stack to pick again later.
                if self.belief.is_pinned(s) and self.faces[self.flip_face] < self.capacity:
                    self._forced_face = self.flip_face
                return 'pick', s
            self._mode = 'delivering'   # nothing left in the pick order - deliver what we already have

        if carried > 0:
            # Strict order: best_drop always returns the DUE zone now, never
            # skips a blacklisted one to a later zone (see belief.py). So
            # there's exactly one drop candidate to consider each tick, not
            # a "try this, else try anything else" ladder.
            z = self.belief.best_drop(include_vertical=self.use_vertical_zones)
            if z is None:
                return 'home', None
            if z.shape == 'circle' and self.faces[self.flip_face] == 0:
                # Can't flip an empty face - Max: specific zones are
                # paired with the stack that should feed them (P1 -> YC1,
                # P2 -> YC4): go get that one on flip_face specifically.
                # Guarded by believed_count so a stack already emptied
                # (e.g. picked onto the wrong face before this zone's pin
                # existed, or just unlucky timing) doesn't get retried
                # forever - falls through to 'wait' instead, holding for
                # the due zone rather than advancing to a different one.
                req = self.belief.required_pick(z)
                if (req is not None and req.name not in skip
                        and self.belief.believed_count(req, t) >= 0.5):
                    self._forced_face = self.flip_face
                    return 'pick', req
                return 'wait', None
            return 'drop', z
        return 'home', None

    def _face_drop_pose(self, face):
        """Where boxes just released from `face` actually end up, in our
        own self-knowledge (current pose + which face) - not ground truth
        about anything else. Mirrors table_sim/world.py's own
        _face_drop_pose/held_poly formula (face center = robot centre +
        r_in along the face direction, box centre = that + half box width
        further out), so the believed drop pose matches where world.py
        itself will render/collide the real pile."""
        x, y, th = self.pose
        face_dir = th + self.face_angles[face]
        bw = self.field.box_width
        standoff = self.r_in + bw / 2 + 0.003
        px = x + math.cos(face_dir) * standoff
        py = y + math.sin(face_dir) * standoff
        pth = face_dir + math.pi / 2
        return (px, py, pth)

    def _spot_for(self, kind, obj):
        # Both pick and drop need a genuinely square approach now: pick
        # because world.py's stack_in_reach requires close, square alignment
        # (pick_reach/pick_max_angle_deg/pick_max_lateral); drop because a
        # dropped pile must land within +-10deg of the zone's own placed
        # angle (Max: "same angle as the zone is placed plus or minus 10
        # degrees") - field.drop_spots' approach_deg is exactly zone.th+-90,
        # which is what makes the held box's own placed angle come out equal
        # to zone.th (see _face_drop_pose/world.py's pile_pose), not just an
        # arbitrary closest-point side. Circle zones additionally need the
        # curated side because they sit right at the castle's narrow waist -
        # a free-angle approach isn't guaranteed to land the final spot
        # back inside the castle the way the curated one was placed to.
        spots = (self.field.pick_spots(obj, self.r_in) if kind == 'pick'
                 else self.field.drop_spots(obj, self.r_in))
        # If repeated stuck-abandons on this target mean the closest option
        # keeps being unreachable (almost always an opponent camped on it),
        # rotate through the other curated sides (if the zone/stack has more
        # than one) instead of recomputing the same one every retry.
        offset = self._drop_retry_offset.get(obj.name, 0)
        spots = spots[offset:] + spots[:offset] if offset else spots
        return min(spots, key=lambda sp: _eta(self.pose[:2], (sp.x, sp.y), self.vmax))

    # ------------------------------------------------------------- driving
    def _goto(self, tx, ty, tth, dt, plan=True, pos_tol=None, yaw_tol=None):
        x, y, th = self.pose
        d = math.hypot(tx - x, ty - y)
        eth = _wrap(tth - th)
        pos_tol = self.pos_tol if pos_tol is None else pos_tol
        yaw_tol = self.yaw_tol if yaw_tol is None else yaw_tol
        if d < pos_tol and abs(eth) < yaw_tol:
            self.v = (0.0, 0.0)
            self.wz = 0.0
            self.blocked_t = 0.0
            self._prev_xy = (x, y, eth)
            return True, 0.0, 0.0, 0.0
        # stuck detection: not yet arrived and not actually getting anywhere
        # - whether that's a path fallback pushing into an obstacle, or just
        # sitting still waiting on opponent-avoidance to clear (Command.vx/
        # vy/wz forced to 0 below) - either way, after blocked_timeout the
        # caller abandons this target and tries something else rather than
        # waiting forever. Deliberately not gated on "were we trying to
        # move fast" (self.v) - a long avoidance wait has self.v decay
        # toward 0 too, which would otherwise look like "not blocked". Also
        # not just position: the heading blend above can legitimately
        # reach pos_tol before yaw_tol (position settles while still
        # rotating the last stretch to tth) - progress on EITHER axis counts,
        # or a real rotation-in-progress gets misread as stuck.
        if self._prev_xy is not None:
            px, py, peth = self._prev_xy
            moved = math.hypot(x - px, y - py) > 0.01
            turning = abs(peth) - abs(eth) > 0.01
            self.blocked_t = 0.0 if (moved or turning) else self.blocked_t + dt
        else:
            self.blocked_t = 0.0
        self._prev_xy = (x, y, eth)
        if not plan:
            self.path = [(tx, ty)]
        elif not self.path or self.path[-1] != (tx, ty):
            wps, _ = self.planner.plan((x, y), (tx, ty))
            self.path = wps if wps else [(tx, ty)]
        while len(self.path) > 1 and math.hypot(self.path[0][0] - x, self.path[0][1] - y) < 0.12:
            self.path.pop(0)
        wx, wy = self.path[0]
        dx, dy = wx - x, wy - y
        dd = math.hypot(dx, dy)
        v = min(self.vmax, math.sqrt(2 * self.amax * 0.8 * d), 3.0 * d + 0.02)
        vx_w, vy_w = (dx / dd * v, dy / dd * v) if dd > 1e-6 else (0.0, 0.0)

        # Heading target: blend from "facing the direction of travel" to
        # the exact required tth as we close in (see dock_heading_blend_dist
        # above) - using remaining distance to the real target (d), not the
        # possibly-nearer waypoint, so the blend always completes by tth.
        # Only for the coarse leg (plan=True, Nav2's real job live - this
        # path only runs for real in headless, which has no Nav2 at all):
        # the direct-controlled docking stage (plan=False) is reached only
        # after that coarse leg (and Nav2's own heading_blend_distance)
        # already got us close to tth, so it should just hold tth exactly,
        # not re-derive a travel direction - atan2(dy, dx) is numerically
        # unstable once d is down near pos_tol (bearing to a point you're
        # almost standing on is noise, not signal), and blending even a
        # small weight of that noise into the target is exactly what was
        # making heading_err - and so wz - swing wildly instead of settling.
        if plan and self.dock_heading_blend_dist > 1e-6:
            travel_heading = math.atan2(dy, dx) if dd > 1e-6 else tth
            blend = max(0.0, min(1.0, 1.0 - d / self.dock_heading_blend_dist))
            target_heading = travel_heading + blend * _wrap(tth - travel_heading)
        else:
            target_heading = tth
        heading_err = _wrap(target_heading - th)

        # Opponent avoidance: only on the direct-controlled final approach
        # (plan=False) - the coarse legs go through Nav2, whose own local
        # costmap already reacts to the opponent live via /scan. Graduated:
        # full speed outside avoid_slow_dist, linearly slower down to a hard
        # stop at avoid_stop_dist, and only when actually closing on it
        # (crossing paths the other way isn't a collision risk).
        if not plan:
            opp = self.tracker.opp
            if opp.visible:
                od = math.hypot(opp.x - x, opp.y - y)
                closing = (vx_w * (opp.x - x) + vy_w * (opp.y - y)) > 0.3 * v * max(od, 1e-6)
                if closing and od < self.avoid_slow_dist:
                    scale = 0.0 if od <= self.avoid_stop_dist else (
                        (od - self.avoid_stop_dist) / (self.avoid_slow_dist - self.avoid_stop_dist))
                    vx_w *= scale
                    vy_w *= scale

        ax, ay = vx_w - self.v[0], vy_w - self.v[1]
        a = math.hypot(ax, ay)
        lim = self.amax * dt
        if a > lim and a > 1e-9:
            ax, ay = ax * lim / a, ay * lim / a
        self.v = (self.v[0] + ax, self.v[1] + ay)
        c, s = math.cos(th), math.sin(th)
        vx_b = c * self.v[0] + s * self.v[1]
        vy_b = -s * self.v[0] + c * self.v[1]
        # Same stopping-distance shape as v above, not a flat proportional
        # gain: a flat gain (e.g. 4.0*heading_err) combined with a separate
        # rate limit is exactly what causes "lag, then overshoot" - the
        # target only starts shrinking once heading_err is already small,
        # but the rate-limited actual wz can't always decelerate fast
        # enough in time to stop it from sliding past zero. Computing the
        # target FROM the accel limit (like v's sqrt(2*amax*0.8*d)) makes
        # it decelerate in time by construction, the same way translation
        # already does.
        # The sqrt term alone still overshot in practice: it's the max
        # speed that *would* stop exactly at zero error decelerating at
        # wacc starting now, but self.wz tracks it with that same wacc, one
        # tick behind - so self.wz is always a bit higher than the current
        # w_target, and decelerating at only wacc from a bit too high
        # slides past zero. v above never had this problem because of its
        # extra `3.0*d+0.02` cap, which is more conservative than the pure
        # stopping-distance term for small d - add the equivalent here.
        wsign = 1.0 if heading_err >= 0 else -1.0
        w_target = wsign * min(self.wmax, math.sqrt(2 * self.wacc * 0.6 * abs(heading_err)),
                                1.5 * abs(heading_err) + 0.05)
        wa = w_target - self.wz
        wlim = self.wacc * dt
        if abs(wa) > wlim:
            wa = wlim if wa > 0 else -wlim
        self.wz += wa
        return False, vx_b, vy_b, self.wz

    # ---------------------------------------------------------------- step
    def step(self, obs, dt):
        cmd = self._step_inner(obs, dt)
        cmd.flip_action = self._decide_flip(obs)
        return cmd

    def _decide_flip(self, obs):
        """Request the flip as soon as flip_face is loaded and we're inside
        the castle - independent of the main target, so it can run while
        driving toward (or already parked at) something else entirely. The
        concurrent world.py slot rejects a redundant request while one is
        already in flight, and the authoritative in-castle check there is on
        the boxes' own footprint for the whole duration, not this coarse
        robot-centre check - this is just the trigger."""
        if not obs.active or obs.flip_busy:
            return None
        if self.faces[self.flip_face] <= 0 or self.vertical[self.flip_face]:
            return None
        if not self.field.in_castle(self.team, self.pose[0], self.pose[1]):
            return None
        return (self.flip_face,)

    def _step_inner(self, obs, dt):
        self.pose = obs.pose
        self.debug.path = []

        # Real robot velocity, from consecutive poses - not the same thing
        # as self.v/self.wz, which during a Nav2-driven 'approach' leg are
        # just this class's own unused internal simulation (strategy_node
        # ignores vx/vy/wz whenever Command.goal_pose is set, so self.v
        # never tracked what the robot actually did). Used to seed the
        # docking controller's ramp at the Nav2->direct-control handoff
        # with the robot's real motion instead of an assumed standstill -
        # a big, fictional velocity-to-zero jump right at the handoff is
        # exactly what reads as a jerky stop-and-go.
        if self._last_pose is not None and dt > 1e-6:
            lx, ly, lth = self._last_pose
            x, y, th = self.pose
            self._vel_est = ((x - lx) / dt, (y - ly) / dt, _wrap(th - lth) / dt)
        self._last_pose = self.pose

        self.tracker.update(obs, dt)
        self.debug.opp = self.tracker.opp.as_dict() if self.tracker.opp.seen_ever else None

        self.belief.on_camera(obs.cam, obs.t)
        if obs.result is not None:
            face, mode, success, count, message = obs.result
            backout = False
            if mode == 0 and self.target_kind == 'pick' and self.target_obj is not None:
                self.belief.on_pick_result(self.target_obj.name, success, count, obs.t)
                if success:
                    self.faces[face] += count
                self.debug.say(f'pick {self.target_obj.label}: {message}')
            elif mode == 1 and self.target_kind == 'drop' and self.target_obj is not None:
                pose = self._face_drop_pose(face) if success else None
                self.belief.on_drop_result(self.target_obj.name, success, count, pose)
                if success:
                    self.faces[face] -= count
                    if self.faces[face] == 0:
                        self.vertical[face] = False   # mirrors world.py's own reset
                    self._drop_retry_offset.pop(self.target_obj.name, None)
                self.debug.say(f'drop {self.target_obj.label}: {message}')
                backout = success
            if backout:
                # Step straight back off the pile we just placed before
                # planning the next goal - avoids Nav2 immediately trying to
                # route through/along boxes still right at our face. Away
                # from the face that actually dropped (th + face_angles
                # [face]), not the robot's base/face-0 heading - backing
                # out along -th when a side face (1 or 2) just dropped
                # would step sideways into the pile instead of away from it.
                x, y, th = self.pose
                face_dir = th + self.face_angles[face]
                self._backout_target = (x - self.backout_dist * math.cos(face_dir),
                                        y - self.backout_dist * math.sin(face_dir), th)
                self.target_kind, self.target_obj, self.spot = 'backout', None, None
            else:
                self.target_kind, self.target_obj, self.spot = None, None, None
            self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None

        if obs.flip_result is not None:
            # Independent of the main target (see _decide_flip) - just update
            # belief; if we're parked at a circle zone waiting on this, the
            # next arrival check below will see the face vertical now and
            # issue the real DROP.
            flip_face, flip_success, _count, flip_message = obs.flip_result
            if flip_success:
                self.vertical[flip_face] = not self.vertical[flip_face]
            self.debug.say(f'flip face {flip_face}: {flip_message}')

        self.debug.belief = {s.name: round(self.belief.believed_count(s, obs.t), 1)
                             for s in self.field.stacks}

        if not obs.active:
            self.debug.state = 'waiting'
            return Command()

        if obs.tool_busy:
            self.debug.state = self.target_kind or 'busy'
            self.v = (0.0, 0.0)
            self.wz = 0.0
            return Command()

        self._rebuild_statics(obs.t)

        if self.target_kind is not None and self.blocked_t > self.blocked_timeout:
            label = self.target_obj.label if self.target_obj is not None else 'home'
            self.debug.say(f'stuck heading to {label}, abandoning')
            if self.target_kind == 'pick':
                self._skip_until[self.target_obj.name] = obs.t + 15.0
            elif self.target_kind == 'drop':
                # Short blacklist, not pick's 15s: an opponent camping on
                # one zone shouldn't take it off the table for long, and
                # with only 5 own zones (vs. 10 stacks) we need to come
                # back to it sooner once the approach is actually clear.
                self._skip_until[self.target_obj.name] = obs.t + 3.0
                # Also rotate to the next curated approach side (see
                # _spot_for) - a zone/stack with only one side just retries
                # that same one, but any with more than one stops re-aiming
                # at whichever got blocked every retry.
                name = self.target_obj.name
                self._drop_retry_offset[name] = self._drop_retry_offset.get(name, 0) + 1
            self.target_kind, self.target_obj, self.spot = None, None, None
            self.stage, self.path, self.blocked_t = 'approach', [], 0.0

        if self.target_kind is None:
            kind, obj = self._decide(obs)
            if kind == 'home':
                self.target_kind, self.target_obj, self.spot = 'home', None, None
            elif kind == 'wait':
                pass  # target_kind stays None - hold position, retry next tick
            elif kind == 'drop' and obj.label in self._dock_override:
                # Max's hand-tuned dock pose from the artifact, exact x/y/yaw
                # and the face that reproduces it - bypasses drop_spots()/
                # _best_face entirely for this zone.
                spot, face = self._dock_override[obj.label]
                if self.faces[face] == 0:
                    self.debug.say(f'{obj.label} needs boxes on face {face}, deferring')
                    self._skip_until[obj.name] = obs.t + 1.5
                    self.target_kind, self.target_obj, self.spot = None, None, None
                else:
                    self.target_kind, self.target_obj, self.spot, self.face = kind, obj, spot, face
            else:
                spot = self._spot_for(kind, obj)
                # Only for picks: we have 10 stacks and can always try a
                # different one instead. Our own drop zones are a scarce,
                # non-substitutable resource (only 5, clustered close
                # together) - refusing to even attempt a contested one would
                # let an opponent camping nearby deny all our scoring for
                # free, which is strictly worse than just contesting it (the
                # final-approach stage's own avoid_slow_dist/avoid_stop_dist
                # already keeps that attempt collision-safe).
                if kind == 'pick' and self._opponent_blocks(spot.pre_x, spot.pre_y):
                    # Don't hand Nav2 a coarse-leg goal sitting inside the
                    # opponent's own footprint - blacklist briefly (it moves)
                    # and reconsider next tick rather than commit to it.
                    self.debug.say(f'{obj.label} goal blocked by opponent, deferring')
                    self._skip_until[obj.name] = obs.t + 1.5
                    self.target_kind, self.target_obj, self.spot = None, None, None
                elif kind == 'drop' and obj.shape == 'circle':
                    # Only flip_face can flip at all (world.py), and one
                    # flip covers the whole face at once - so reuse whatever
                    # that face is already carrying rather than picking by
                    # heading like a normal drop; if it's empty right now
                    # (everything we're carrying landed on the other faces),
                    # defer this target rather than drive home carrying boxes.
                    face = self.flip_face if self.faces[self.flip_face] > 0 else None
                    if face is None:
                        self.debug.say(f'{obj.label} needs boxes on face {self.flip_face} to flip, deferring')
                        self._skip_until[obj.name] = obs.t + 1.5
                        self.target_kind, self.target_obj, self.spot = None, None, None
                    else:
                        self.target_kind, self.target_obj, self.spot, self.face = kind, obj, spot, face
                elif kind == 'pick' and self._forced_face is not None:
                    # _decide pinned this pick to flip_face specifically (it
                    # exists to load a circle-zone drop that's waiting on
                    # it) - use it even if it's not the heading-optimal
                    # choice, that's the whole point of pinning it.
                    face = self._forced_face
                    self.target_kind, self.target_obj, self.spot, self.face = kind, obj, spot, face
                else:
                    avoid = {self.flip_face} if (kind == 'drop' and self._circle_zones_pending()) else ()
                    face = self._best_face(spot.face_heading, 'free' if kind == 'pick' else 'loaded',
                                           avoid=avoid)
                    if face is None:
                        self.target_kind, self.target_obj, self.spot = 'home', None, None
                    else:
                        self.target_kind, self.target_obj, self.spot, self.face = kind, obj, spot, face
            # fresh start: don't let blocked-time accumulated against the
            # previous target (or just sitting idle before this decision)
            # count against the new one.
            self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None

        if self.target_kind is None:
            return Command()

        self.debug.state = self.target_kind

        if self.target_kind == 'home':
            hx, hy, hth = self.field.start_pose(self.team)
            arrived, vx, vy, wz = self._goto(hx, hy, hth, dt,
                                             pos_tol=self.nav_pos_tol, yaw_tol=self.nav_yaw_tol)
            self.debug.path = list(self.path)
            if arrived:
                self.debug.state = 'parked'
                return Command()
            return Command(vx, vy, wz, goal_pose=(hx, hy, hth))

        if self.target_kind == 'backout':
            bx, by, bth = self._backout_target
            arrived, vx, vy, wz = self._goto(bx, by, bth, dt, plan=False)
            self.debug.path = list(self.path)
            if arrived:
                self.target_kind, self.target_obj, self.spot = None, None, None
                self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None
                return Command()
            return Command(vx, vy, wz)

        sp = self.spot
        heading = _wrap(sp.face_heading - self.face_angles[self.face])
        if self.stage == 'approach':
            arrived, vx, vy, wz = self._goto(sp.pre_x, sp.pre_y, heading, dt,
                                             pos_tol=self.nav_pos_tol, yaw_tol=self.nav_yaw_tol)
            self.debug.path = list(self.path)
            if not arrived:
                return Command(vx, vy, wz, goal_pose=(sp.pre_x, sp.pre_y, heading))
            # Face is decided once, at target selection, and never revisited
            # here. That used to need a re-pick on arrival because Nav2's own
            # yaw tolerance was effectively infinite (3.14 rad), so the
            # coarse leg could leave the robot facing anything and a stale
            # face choice meant a big spin right at the stack/zone. Now that
            # motion_controller actually enforces the requested goal yaw
            # (pure_pursuit_controller.cpp's terminal yaw stage,
            # goal_yaw_tolerance 10deg), the heading computed above is
            # reliably what Nav2 delivers - reselecting here would just
            # re-introduce the problem by changing the required heading
            # after Nav2 already hit the one that was asked for.
            # Blend straight into the next leg this same tick instead of an
            # extra all-stop tick - Nav2 was driving during 'approach', so
            # this class's own internal velocity ramp (self.v) was never
            # real; seed it from the robot's actual estimated velocity
            # instead of an assumed standstill, so the docking controller's
            # own ramp continues smoothly from whatever Nav2 really left us
            # doing rather than snapping to a fictional zero.
            risky = (self.target_kind == 'pick'
                     and self.belief.confidence(self.target_obj, obs.t) < self.verify_confidence)
            if risky:
                self.stage, self.blocked_t, self._prev_xy = 'verify', 0.0, None
                self._verify_t0 = obs.t
            else:
                self.stage, self.path, self.blocked_t, self._prev_xy = 'final', [], 0.0, None
            self.v = self._vel_est[:2]
            self.wz = self._vel_est[2]

        if self.stage == 'verify':
            # Low-confidence pick target: turn the face-0 camera toward it
            # and wait for a fresh reading before committing to the final
            # approach - confirms it's still there, or saves the rest of
            # the trip (and the pick attempt) if the opponent already took
            # it. Holds position, only rotates.
            tx, ty = self.target_obj.x, self.target_obj.y
            x, y, th = self.pose
            eth = _wrap(math.atan2(ty - y, tx - x) - th)
            seen = obs.cam is not None and self.target_obj.name in obs.cam
            timed_out = (obs.t - self._verify_t0) > self.verify_timeout
            if not (seen or timed_out):
                self.debug.state = 'verify'
                return Command(0.0, 0.0, max(-self.wmax, min(self.wmax, 4.0 * eth)))
            if seen and self.belief.believed_count(self.target_obj, obs.t) < 0.5:
                self.debug.say(f'verified {self.target_obj.label} empty, skipping')
                self._skip_until[self.target_obj.name] = obs.t + 15.0
                self.target_kind, self.target_obj, self.spot = None, None, None
                self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None
                return Command()
            self.stage, self.path, self.blocked_t, self._prev_xy = 'final', [], 0.0, None
            self.v = self._vel_est[:2]
            self.wz = self._vel_est[2]

        if self.target_kind == 'drop' and self.target_obj.shape == 'circle':
            # Unlike a rect zone's generous "any overlap" footprint, a
            # circle zone is only r=0.10 across - the loose drop_pos_tol
            # that's fine for a 0.4x0.2m rect can land the held (now
            # box_width x box_width, post-flip) box short of actually
            # overlapping it. Dock as precisely as a pick instead.
            arrived, vx, vy, wz = self._goto(sp.x, sp.y, heading, dt, plan=False)
        elif self.target_kind == 'drop':
            arrived, vx, vy, wz = self._goto(sp.x, sp.y, heading, dt, plan=False,
                                             pos_tol=self.drop_pos_tol, yaw_tol=self.drop_yaw_tol)
        else:
            arrived, vx, vy, wz = self._goto(sp.x, sp.y, heading, dt, plan=False)
        self.debug.path = list(self.path)
        if arrived:
            if self.target_kind == 'drop' and self.target_obj.shape == 'circle' \
                    and not self.vertical[self.face]:
                # Not vertical yet - the flip for this face is requested
                # independently every step (_decide_flip, concurrent world.py
                # slot), including while we were still driving here. Just
                # hold position; once obs.flip_result reports it done this
                # branch stops matching and the real DROP goes out below.
                self.debug.state = 'awaiting flip'
                return Command(0.0, 0.0, 0.0)
            count = 0
            if self.target_kind == 'pick':
                mode = 0
            else:
                mode = 1
                if self.target_kind == 'drop' and self.target_obj.shape == 'circle':
                    # One box per circle visit by default - keeps the YC1/YC2/YC3
                    # drop_sequence order filling each zone in turn. A circle now
                    # has room for up to 3 (field.py), so this is a policy choice,
                    # not a capacity limit - raise it if a target wants more at once.
                    count = 1
            return Command(0.0, 0.0, 0.0, action=(self.face, mode, count))
        return Command(vx, vy, wz)
