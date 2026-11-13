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
import re

from table_sim.field import Spot, StackSpec
from table_sim.flipmap import FlipMap
from table_sim.geometry import (polygon_from_truncated_triangle, rect_poly, sat_mtv,
                                truncated_triangle_apothem, truncated_triangle_circumradius)
from table_sim.gridplan import GridPlanner

from .belief import StackBelief
from .opponent_tracker import OpponentTracker


class Observation:
    __slots__ = ('t', 'active', 'time_left', 'pose', 'ranges', 'angle_min',
                 'angle_inc', 'lidar_yaw', 'cam', 'tool_busy', 'result',
                 'flip_busy', 'flip_result', 'opp_xy', 'evac')

    def __init__(self, t, active, time_left, pose, ranges, angle_min, angle_inc,
                 lidar_yaw, cam, tool_busy, result, flip_busy=False, flip_result=None, opp_xy=None, evac=None):
        self.opp_xy = opp_xy      # (x, y) of the opponent from an external beacon, or None (lidar only)
        # (x, y): the robot centre is in a cell the global planner treats as blocked (inside a pile's /
        # the border's footprint circle while carrying) and this is the nearest free point; else None
        self.evac = evac
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
        self.log_total = 0   # monotonic count of every say() ever, even once log's tail is trimmed

    def say(self, msg):
        self.log.append(msg)
        del self.log[:-20]
        self.log_total += 1


def gap_fillers(polys, max_gap, width=0.04):
    """Thin rects (cx, cy, th, length, width) closing every gap narrower than max_gap between two of
    `polys`. Nav2's planner only keeps the bare robot's inscribed radius clear (0.11 m), so it routed
    through e.g. the 0.25 m Y2-Y4 gap the robot (0.31 m across its corners, more loaded) cannot pass."""
    import numpy as np
    from table_sim.geometry import closest_point_on_poly
    out = []
    P = [np.asarray(p, float) for p in polys]
    for i in range(len(P)):
        for j in range(i + 1, len(P)):
            best = None
            for A, B, swap in ((P[i], P[j], False), (P[j], P[i], True)):
                for a in A:
                    q, d = closest_point_on_poly(a, B)
                    if best is None or d < best[2]:
                        best = (q, a, d) if swap else (a, q, d)
            pa, pb, d = best
            if 1e-6 < d < max_gap:
                c = (pa + pb) / 2
                out.append((float(c[0]), float(c[1]), math.atan2(pb[1] - pa[1], pb[0] - pa[0]), d + 0.02, width))
    return out


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
        # Hexagon (3 long face edges + 3 short corner edges, see
        # table_sim/world.py's RobotModel/geometry.polygon_from_truncated_
        # triangle) - NOT a bare equilateral triangle. r_in (apothem) is
        # what every pick/drop standoff and the opponent-clearance circle
        # below are built from, so a stale triangle r_in here (now fixed)
        # used to put every one of those ~1cm off from the real hexagon.
        self.face_long_m = float(p.get('face_long_m', 0.22))
        self.face_short_m = float(p.get('face_short_m', 0.08))
        self.r_in = truncated_triangle_apothem(self.face_long_m, self.face_short_m)
        self.r_out = truncated_triangle_circumradius(self.face_long_m, self.face_short_m)
        self.face_angles = [math.radians(a) for a in p.get('face_angles_deg', [0, 120, 240])]
        self.capacity = int(p.get('boxes_per_face', 3))
        self.vmax = float(p.get('max_lin_vel', 1.0))
        self.amax = float(p.get('max_lin_acc', 1.5))
        self.wmax = float(p.get('max_ang_vel', 4.0))
        self.wacc = float(p.get('max_ang_acc', 8.0))
        self.dock_w_gain = float(p.get('dock_w_gain', 1.5))
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
        # circumradius of two robots plus margin.
        self.opponent_goal_clearance = float(
            p.get('opponent_goal_clearance_m', 2.0 * self.r_out + 0.1))
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
        # Docking handover (Max): close to the approach point, stop relying on Nav2 / the global
        # costmap (it inflates the piles we dock against and blocks the last bit) - switch to the
        # direct docking stage once within dock_handover_m and not making progress for
        # dock_handover_stall_s (or within nav_pos_tol, as before).
        self.dock_handover = float(p.get('dock_handover_m', 0.25))
        self.dock_handover_stall = float(p.get('dock_handover_stall_s', 0.5))
        # approach point (pre-pose) kept this far from the table edges: shortened toward the dock
        self.pre_wall_clear = float(p.get('pre_pose_wall_clear_m', 0.22))
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
        # picks: the world is lenient (reach 5 cm, lateral 12 cm, 25 deg) - don't spend time on precision
        # per face (Max): face 0 (flip face) 3 cm / 4 deg, faces 1 and 2 tighter, 2 cm / 2 deg
        self.pick_pos_tol = float(p.get('pick_pos_tol_m', 0.03))
        self.pick_yaw_tol = float(p.get('pick_yaw_tol_rad', math.radians(4)))
        self.pick_pos_tol_side = float(p.get('pick_pos_tol_side_m', 0.02))
        self.pick_yaw_tol_side = float(p.get('pick_yaw_tol_side_rad', math.radians(2)))
        # start the pick/drop only once the robot has (nearly) stopped: releasing while still rolling
        # made it coast 2-3 cm past the dock (Y2 refused)
        self.release_v = float(p.get('release_v_mps', 0.05))
        self.backout_dist = float(p.get('backout_dist_m', 0.06))
        self.backout_handover = float(p.get('backout_handover_m', 0.03))   # this close to the back-out end: go on
        # after a vertical (circle-zone) drop the remaining standing boxes span 3 box widths across
        # the face: step further back before turning, so they don't sweep the tower just placed
        self.backout_vertical = float(p.get('backout_vertical_m', 0.15))
        self.backout_loaded = float(p.get('backout_loaded_m', 0.20))      # after a drop with boxes still held
        self.backout_pick = float(p.get('backout_pick_m', 0.13))          # after a pick (0 = no back-out)...
        self.backout_pick_border = float(p.get('backout_pick_border_m', 0.25))   # ...of a stack this close to the border
        # stop at the Nav2 -> direct-docking handover (no carried-over side speed) for at most this long
        self.dock_settle_s = float(p.get('dock_settle_s', 0.3))
        self.dock_settle_v = float(p.get('dock_settle_v', 0.03))
        # In-place floor regrip (PlannerBrain's ('regrip', f)): step back this far off the floor pile
        # before turning - the hull's corners (circumradius) reach further than its faces (apothem).
        self.regrip_backout = float(p.get('regrip_backout_m', 0.08))
        self.regrip_max_tries = int(p.get('regrip_max_tries', 3))
        self.await_flip_timeout = float(p.get('await_flip_timeout_s', 3.0))
        # Below this confidence, a pick target is "risky" enough to spend a
        # moment turning the face-0 camera toward it before committing to
        # the final approach - confirms it's still there (saving the rest
        # of the trip if not) instead of finding out only after arriving.
        self.verify_confidence = float(p.get('verify_confidence_threshold', 0.7))
        self.verify_timeout = float(p.get('verify_timeout_s', 3.0))

        self.blocked_timeout = float(p.get('blocked_timeout_s', 2.5))
        # Separate from blocked_timeout above (which Max set to effectively
        # infinite so a due pick/drop is never abandoned) - a backout is
        # just a cosmetic step-back after a successful drop, not a
        # strategic target, so it must NEVER be allowed to hang: _goto's
        # opponent-avoidance (see selectSafeVelocity-equivalent block
        # below, "Opponent avoidance: only on the direct-controlled final
        # approach") applies to backout too (plan=False), and the
        # opponent lingering nearby used to be able to zero its velocity
        # and freeze it there indefinitely ("stuck at backout, no
        # apparent reason").
        self.backout_timeout = float(p.get('backout_timeout_s', 2.0))
        self.planner = GridPlanner(field.length, field.width, res=0.04)
        self.pose = (0.0, 0.0, 0.0)

        # Circle (V) zones only take flipped boxes, and world.py only lets
        # face `flip_face` (the one with the flip mechanism) flip at all -
        # matches table_sim/config/sim.yaml's tool.flip_face.
        self.flip_face = int(p.get('flip_face', 0))
        # World's flip rule: the flip-face stack fully inside our castle (the robot centre may be outside)
        self.flipmap = FlipMap(field, team, self.r_in, self.face_angles[self.flip_face])
        self._flipgo_want, self._flipgo_pose = True, None
        self.use_vertical_zones = bool(p.get('use_vertical_zones', True))
        # Door (2 vertical posts + horizontal lid) on this own-team zone label.
        self.door_label = p.get('door_zone')
        self.door_min_time = float(p.get('door_min_time_s', 30.0))
        self.final_idx = str(p.get('final_zone_index', 5))
        # a pick is only worth starting if its boxes can still be delivered
        self.pick_slack = float(p.get('pick_min_slack_s', 22.0))

        # Exact drop dock poses Max tuned by hand in the table-reference
        # artifact's dock-pose editor (dock_poses.json: {label: {x, y,
        # yaw_deg, approach_deg?}}, circle zones as <label>_L/_C/_R - only
        # _C is loaded here, the centre-slot dock pose; see _dock_spot for
        # how one of the other 2 fixed slots is targeted instead).  Keyed
        # by zone label (Y1..Y5, YC1..YC4) -> the raw calibrated pose/face,
        # bypassing the geometric drop_spots()/_best_face computation
        # entirely for these - Max: "use the docking position on artifact."
        # The actual Spot is built on demand by _dock_spot (not baked here)
        # since a circle zone's dock pose depends on which of the 3
        # vertical slots is about to be dropped - see _next_slot.
        self._dock_override = self._load_dock_overrides(p.get('dock_poses_file'))
        # Flip spot for a stack picked outside the castle: the Y5 dock area (most room) -
        # same pose as PlanModel.flip_spot.
        y5 = next((z for z in field.team_zones(team) if z.shape == 'rect' and z.label.endswith('5')), None)
        ov = self._dock_override.get(y5.label) if y5 is not None else None
        xy = (ov['x'], ov['y']) if ov else ((y5.x, y5.y) if y5 is not None else (0.5, 1.0))
        self.flip_spot = self.flipmap.nearest(*xy)[1] or (xy[0], xy[1], 0.0)
        self.flip_area_r = float(p.get('flip_area_r_m', 0.30))
        # Flip poses by a circle zone (field yaml flip_poses:, Max): a tower load for that circle flips
        # there, right before its drop. {zone label: (x, y, body yaw)}
        self.zone_flip_poses = dict(field.flip_poses.get(team, {}))
        self.zone_flip_tol = (float(p.get('zone_flip_pos_tol_m', 0.05)),
                              math.radians(float(p.get('zone_flip_yaw_tol_deg', 8.0))))
        # gaps narrower than this between castle piles / towers / discards are closed for path planning
        self.gap_block = float(p.get('gap_block_m', 0.33))
        # flip in the flip area only when already within this of the next drop's dock heading
        self.flip_heading_tol = math.radians(float(p.get('flip_heading_tol_deg', 30.0)))

        self.reset()

    def _load_dock_overrides(self, path):
        overrides = {}
        # Dock poses from the field yaml (`docks:`, Max): the reference. yaw there is the heading of
        # the dropping face; stored here as the body yaw with face 0 dropping (the brain re-derives
        # the body heading for whichever face it uses). Approach straight from behind.
        for zone in self.field.team_zones(self.team):
            if zone.dock is None:
                continue
            x, y, h = zone.dock
            face = self.flip_face if zone.shape == 'circle' else 0
            overrides[zone.label] = dict(x=x, y=y, yaw=_wrap(h - self.face_angles[face]),
                                         approach=_wrap(h + math.pi), face=face,
                                         zone=zone.name, shape=zone.shape)
        if not path or not os.path.exists(path):
            return overrides
        with open(path) as f:
            data = json.load(f)
        poses = data.get('poses', {})
        for key, pose in poses.items():
            if '_' in key:
                base, pos = key.rsplit('_', 1)
                if pos != 'C':   # only the artifact's centre pose is loaded - _dock_spot shifts off it
                    continue
                label = base
            else:
                label = key
            zone = next((z for z in self.field.team_zones(self.team) if z.label == label), None)
            if zone is None or label in overrides:      # the field's dock pose wins
                continue
            x, y = float(pose['x']), float(pose['y'])
            yaw = math.radians(float(pose['yaw_deg']))
            approach = math.radians(float(pose['approach_deg'])) if 'approach_deg' in pose else None
            # Circle zones only ever receive flipped boxes off flip_face
            # (world.py hard rule) - the working face there is always
            # flip_face, never inferred from yaw.
            face = self.flip_face if zone.shape == 'circle' else self._dock_face(x, y, yaw, zone.x, zone.y)
            overrides[label] = dict(x=x, y=y, yaw=yaw, approach=approach,
                                    face=face, zone=zone.name, shape=zone.shape)
        return overrides

    def _next_slot(self, face):
        """Lowest-index vertical slot still believed occupied on `face`, or
        None if none are (e.g. a non-circle drop, or nothing flipped) -
        FIFO matches the load order (_decide_flip/obs.flip_result sets
        slots 0..n-1 for n boxes, same as world.py), and world.py's own
        zone_under_face resolves a drop to the lowest-index occupied slot
        whose footprint actually overlaps the zone - so as long as the dock
        pose below puts ONLY that slot over the zone, this stays in sync
        with which box really lands, with no ground-truth feedback needed."""
        for s, occupied in enumerate(self.vertical_slots[face]):
            if occupied:
                return s
        return None

    def _clear_slot(self, face, label):
        """Which occupied vertical slot to drop at circle zone `label`: the lowest one whose shifted
        dock keeps the robot and everything else it carries clear of the piles we already built
        (e.g. YC1 slot 0 pushes the face's other standing boxes into the Y3 pile). Falls back to
        the lowest occupied slot."""
        occ = [i for i, o in enumerate(self.vertical_slots[face]) if o]
        if len(occ) <= 1:
            return occ[0] if occ else None
        zc = self.belief.zone_counts()
        piles = [self.field.pile_poly(z, self.zone_obstacle_pose(z))
                 for z in self.field.team_zones(self.team)
                 if zc.get(z.name, 0) > 0 and z.label != label]
        if not piles:
            return occ[0]
        bw, bl = self.field.box_width, self.field.box_length
        off = self.r_in + bw / 2 + 0.003
        body0 = polygon_from_truncated_triangle(self.face_long_m, self.face_short_m, self.face_angles)
        for cand in occ:
            sp, _ = self._dock_spot(label, cand)
            th = _wrap(sp.face_heading - self.face_angles[face])
            c, sn = math.cos(th), math.sin(th)
            polys = [[(sp.x + c * px - sn * py, sp.y + sn * px + c * py) for px, py in body0]]
            for f, n in enumerate(self.faces):
                if n <= 0:
                    continue
                a = th + self.face_angles[f]
                nx, ny = math.cos(a), math.sin(a)
                cx, cy = sp.x + nx * off, sp.y + ny * off
                if f == face and self.vertical[f]:
                    for k in occ:
                        if k != cand:
                            px_, py_ = cx - ny * (k - 1) * bw, cy + nx * (k - 1) * bw
                            polys.append(rect_poly(px_, py_, bw, bw, a + math.pi / 2))
                else:
                    polys.append(rect_poly(cx, cy, 3 * bw if self.vertical[f] else bl, bw, a + math.pi / 2))
            hit = False
            for pg in polys:
                for pl in piles:
                    m = sat_mtv(pg, pl)
                    if m is not None and m[1] > 0.005:
                        hit = True
                        break
                if hit:
                    break
            if not hit:
                return cand
        return occ[0]

    def _dock_spot(self, label, slot):
        """Build the Spot for `label`'s calibrated dock pose, shifted for
        `slot` if it's a circle zone (None/non-circle: centre, unshifted).

        Shift direction: world.py's slot 0/2 naming ("left"/"right") is
        from an observer facing the robot's face, which is the robot's own
        RIGHT/LEFT respectively - confirmed against the real mechanism:
        dropping slot 0 (first box loaded, physically on the robot's own
        right) needs the dock pose shifted to the robot's own left, and
        slot 2 (last box, robot's own left) needs it shifted to the
        robot's own right. `tangent` below is the robot's own left
        direction (feed yaw=0: tangent=(0,1)=+y=left, ROS REP103), so
        `shift = (1 - slot) * bw` along it gives +bw (left) for slot 0 and
        -bw (right) for slot 2, centre (slot 1) unshifted."""
        d = self._dock_override[label]
        x, y, yaw, face = d['x'], d['y'], d['yaw'], d['face']
        if d['shape'] == 'circle' and slot is not None and slot != 1:
            bw = self.field.box_width
            tangent_x, tangent_y = -math.sin(yaw), math.cos(yaw)
            shift = (1 - slot) * bw
            x, y = x + tangent_x * shift, y + tangent_y * shift
        corr = self._dock_corr.get(label)
        if corr:
            zth = self.field.zone(d['zone']).th
            c, sn = math.cos(zth), math.sin(zth)
            x, y = x + corr[0] * c - corr[1] * sn, y + corr[0] * sn + corr[1] * c
        face_heading = _wrap(yaw + self.face_angles[face])
        if d['approach'] is not None:
            ax, ay = math.cos(d['approach']), math.sin(d['approach'])
            pre_d = 0.20
            while pre_d > 0.0:                  # keep the approach point off the table edges (e.g. YC4)
                px_, py_ = x + ax * pre_d, y + ay * pre_d
                c = self.pre_wall_clear
                if c <= px_ <= self.field.length - c and c <= py_ <= self.field.width - c:
                    break
                pre_d -= 0.02
            pre_d = max(pre_d, 0.0)
            pre_x, pre_y = x + ax * pre_d, y + ay * pre_d
        else:
            pre_x, pre_y = x, y
        return Spot('drop', d['zone'], x, y, face_heading, pre_x, pre_y), face

    def _discard_spot(self, zone_name):
        """Spot at `zone_name`'s discard dock (field yaml discard_docks:), approached from behind."""
        d = next((d for d in self.field.discard_docks.get(self.team, []) if d[0] == zone_name), None)
        if d is None:
            return None
        _, x, y, h = d
        return Spot('drop', zone_name, x, y, h, x - 0.20 * math.cos(h), y - 0.20 * math.sin(h))

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
        self._door_phase = 'none'
        self._door_zone_label = None
        self._dock_corr = {}            # zone label -> [along, across] metres added to the dock (self-calibration)
        self._flip_block_until = 0.0
        self._detouring, self._detour_hold = False, 0.0
        self._flip_pose = None          # FlipMap pose the flip detour drives to (None: not chosen yet)
        self._await_key, self._await_t0 = None, 0.0   # (target, face) we are docked waiting a flip for
        self._keepout_from = float(self._params.get('keepout_from_s', 20.0))   # castle is only forbidden after this
        self._keepout_on = bool(self._params.get('keepout', True))   # plan around the forbidden opponent castle
        self._drop_face = None          # face the planner wants for the next drop (None: brain decides)
        self._discard = False           # this drop is a discard (planner): dock at the zone's discard dock
        self.discard_piles = []         # (x, y, th) of stacks we discarded next to a built wall: obstacles
        self._last_kind, self._last_face = '', -1     # last completed pick/drop and the face it used
        self.dropped_log, self.picked_log = [], []   # zone / stack names, in order

        self.faces = [0] * len(self.face_angles)   # our own belief of what we're carrying, per face
        self.vertical = [False] * len(self.face_angles)  # our own belief of each face's flip state
        # Our own belief of which of the 3 fixed vertical slots (see
        # _CIRCLE_SLOT) are still occupied per face - set on flip (see
        # obs.flip_result handling), cleared slot-by-slot as each circle
        # drop succeeds (see obs.result handling). Mirrors world.py's own
        # RobotState.vertical_slots, kept independently since we never read
        # ground truth - only our own flip/drop results.
        self.vertical_slots = [[False, False, False] for _ in self.face_angles]
        self._target_slot = None        # which vertical slot the current drop targets, or None
        self._mode = 'collecting'       # 'collecting' (pick up to all faces) | 'delivering' (drop them all)
        self._door_phase = 'none'       # none -> posts -> done | abandoned
        self._door_zone_label = None    # zone the door is being built in (fixed by config or first posts)
        self.target_kind = None         # 'pick' | 'drop' | 'home' | 'backout' | None
        self.target_obj = None          # StackSpec | ZoneSpec | None
        self.spot = None
        self._backout_target = None     # (x, y, th) while target_kind == 'backout'
        self._settle_until = -1.0       # final docking waits for the robot to stop until this time
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
        self._now = 0.0
        self._spot_side = 0             # index (in field.pick_spots / drop_spots order) of the approach side _spot_for chose
        self._floor_pile = None         # (x, y, th, n): our stack put on the floor by a regrip, to repick with flip_face
        self._regrip_held = False       # the flip face holds a regripped stack not flipped yet (PlanState.regrip_flip)
        self._evacuating = False        # stepping out of a blocked global-costmap cell (Observation.evac)
        self._floor_tries = 0

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
        # Inflate by the held boxes too once loaded, or paths hug piles the
        # boxes then wedge on.
        loaded = sum(self.faces) > 0
        stealing = getattr(self.target_obj, 'steal_zone', '') if self.target_kind == 'pick' else ''
        inside_ko = self.field.in_castle('blue' if self.team == 'yellow' else 'yellow', self.pose[0], self.pose[1])
        # picking an enemy drop-zone pile (or leaving the castle afterwards) is allowed inside the keep-out
        ko_on = self._keepout_on and t >= self._keepout_from and not stealing and not inside_ko
        fp = self._floor_pile
        key = (tuple(sorted(s.name for s in occ_stacks) + sorted(z.name for z in occ_zones)), loaded, ko_on,
               tuple(round(v, 3) for v in fp[:3]) if fp else None, len(self.discard_piles))
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
        # castle objects (piles, towers, our discards): close the gaps the robot cannot pass (Max: Nav2
        # tried to path between Y2 and Y4)
        castle = zone_polys + [rect_poly(dx_, dy_, self.field.box_length, self.field.box_width, dth)
                               for (dx_, dy_, dth) in self.discard_piles]
        zone_polys = zone_polys + [rect_poly(cx, cy, ln, wd, th)
                                   for (cx, cy, th, ln, wd) in gap_fillers(castle, self.gap_block)]
        polys = [s.poly for s in occ_stacks] + zone_polys
        if fp:
            polys.append(rect_poly(fp[0], fp[1], self.field.box_length, self.field.box_width, fp[2]))
        for (dx_, dy_, dth) in self.discard_piles:
            polys.append(rect_poly(dx_, dy_, self.field.box_length, self.field.box_width, dth))
        ko = self.field.keepout_poly(self.team)      # opponent castle: never plan through it
        keep = [ko] if (len(ko) and ko_on) else []
        # the opponent's start zone: forbidden at all times (unless we are somehow in it: get out)
        if not self.field.in_start_keepout(self.team, self.pose[0], self.pose[1]):
            keep.append(self.field.start_keepout_poly(self.team))
        self.planner.set_static(key, polys, self.r_in + 0.02 + (self.field.box_width if loaded else 0.0),
                                keepout=keep)
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
        elif self._mode == 'delivering' and time_left >= eta_home + self.pick_slack:
            # A face freed up mid-delivery (e.g. just dropped Y1) while
            # we're still carrying elsewhere (flip_face mid-feeding circles).
            # Max's choreography picks P1/P3 back in once the due drop
            # reaches the first circle zone (YC1), NOT right after Y1 -
            # Y1 -> Y3 is a strictly-sequential pair that must finish
            # dropping before picking resumes (see belief.due_order/
            # first_interleave_order). still_sequential guards that: a due
            # zone of None (nothing left to drop) doesn't block it either.
            due = self.belief.best_drop(include_vertical=self.use_vertical_zones)
            still_sequential = due is not None and self.belief.due_order(due) < self.belief.first_interleave_order()
            if not still_sequential and self.belief.next_pick(exclude=skip, opp=self.tracker.opp) is not None:
                self._mode = 'collecting'

        if self._door_phase == 'posts':
            self._mode = 'delivering'   # finish the lid before picking anything

        if self._mode == 'collecting' and time_left >= eta_home + self.pick_slack:
            s = self.belief.next_pick(exclude=skip, opp=self.tracker.opp)
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
            if z.shape == 'rect' and z.label[1:] == self.final_idx:
                extra = self._extra_drop_zone(z)
                if extra is not None:
                    return 'drop', extra
            if self._door_label_of(z) and self._door_phase == 'none':
                ff = self.flip_face
                if self.faces[ff] != 3:
                    # Door needs a full fresh stack on flip_face.
                    nxt = (self.belief.next_pick(exclude=skip, opp=self.tracker.opp)
                           if (self.faces[ff] == 0 and time_left >= self.door_min_time) else None)
                    if nxt is not None:
                        self._forced_face = ff
                        return 'pick', nxt
                    self._door_phase = 'abandoned'
                    self.debug.say('door abandoned: no full stack on flip face')
                elif time_left < self.door_min_time:
                    self._door_phase = 'abandoned'
            # The whole trip (dock, drop, back home) must fit, with slack for
            # detours/flip - straight-line ETAs are optimistic, hence the 1.5.
            ov = self._dock_override.get(z.label)
            sx, sy = (ov['x'], ov['y']) if ov else (z.x, z.y)
            need = (1.5 * (_eta(self.pose[:2], (sx, sy), self.vmax) + _eta((sx, sy), (hx, hy), self.vmax))
                    + self.go_home_margin + 3.0 + (8.0 if z.shape == 'circle' else 0.0))
            if time_left < need:
                return 'home', None
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

    def _face_drop_pose_slot(self, face, slot):
        """Like _face_drop_pose, but for a vertical drop into a specific
        one of the 3 fixed slots - mirrors world.py's own _slot_point/
        _face_drop_pose_slot (same tangent, same (slot - 1) * bw offset,
        note: NOT the opposite-sign shift _dock_spot uses for where the
        ROBOT should stand - this is where the BOX itself actually ends
        up)."""
        x, y, th = self.pose
        face_dir = th + self.face_angles[face]
        bw = self.field.box_width
        standoff = self.r_in + bw / 2 + 0.003
        tangent_x, tangent_y = -math.sin(face_dir), math.cos(face_dir)
        shift = (slot - 1) * bw
        px = x + math.cos(face_dir) * standoff + tangent_x * shift
        py = y + math.sin(face_dir) * standoff + tangent_y * shift
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
        sz = getattr(obj, 'steal_zone', '') if kind == 'pick' else ''
        pose = self.belief.zone_pose(sz) if sz else None
        if pose is not None:      # repick of our own pile: dock on where it REALLY landed, not the zone's preset centre
            spots = [self.field._spot('pick', obj.name, pose[0], pose[1], self.field.box_width / 2, a,
                                      self.r_in, 0.012, 0.20) for a in obj.approach]
        # If repeated stuck-abandons on this target mean the closest option
        # keeps being unreachable (almost always an opponent camped on it),
        # rotate through the other curated sides (if the zone/stack has more
        # than one) instead of recomputing the same one every retry.
        offset = self._drop_retry_offset.get(obj.name, 0)
        order = list(range(len(spots)))
        if offset:
            spots = spots[offset:] + spots[:offset]
            order = order[offset:] + order[:offset]
        cands = list(zip(order, spots))
        # a side the brain knows is blocked (PlannerBrain: opponent at that side's pre-pose, stuck
        # there, impossible with what we carry) is only a last resort: try the other sides first
        ok = [(i, sp) for i, sp in cands if not self._side_blocked(kind, obj, i, sp)]
        i, sp = min(ok or cands, key=lambda c: _eta(self.pose[:2], (c[1].x, c[1].y), self.vmax))
        self._spot_side = i
        return sp

    def _side_blocked(self, kind, obj, side, spot):
        """Hook: is this approach side of `obj` known to be blocked right now? (rule brain: never)"""
        return False

    def _pick_goal_blocked(self, obj, spot):
        """Hook: defer a pick because of the opponent near its pre-pose? The rule brain does (old broad
        check); PlannerBrain does not (its watchdog applies the tight 'opponent is taking it' rule)."""
        return self._opponent_blocks(spot.pre_x, spot.pre_y)

    def _stuck_pick(self, obs):
        self._skip_until[self.target_obj.name] = obs.t + 15.0

    def _stuck_drop(self, obs):
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

    def _stuck_other(self, obs):
        """Hook for target kinds the base class does not know (PlannerBrain: 'look')."""

    def _pick_failed(self, obj, message):
        """Hook: a pick of `obj` was refused (e.g. 'no stack in reach')."""

    def _look_step(self, obs, dt):
        """Hook: PlannerBrain's scouting target."""
        self.target_kind, self.target_obj, self.spot = None, None, None
        return Command()

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
                                self.dock_w_gain * abs(heading_err) + 0.05)
        wa = w_target - self.wz
        wlim = self.wacc * dt
        if abs(wa) > wlim:
            wa = wlim if wa > 0 else -wlim
        self.wz += wa
        return False, vx_b, vy_b, self.wz

    # ---------------------------------------------------------------- step
    def step(self, obs, dt):
        cmd = self._step_inner(obs, dt)
        ev = getattr(obs, 'evac', None)
        if cmd.goal_pose is not None and ev is not None:
            # About to hand Nav2 a leg from a cell its planner treats as blocked: it would never plan
            # (Max: stuck against the border / between the castle and the wall). Step out first, direct.
            if not self._evacuating:
                self.debug.say('evacuate: robot centre inside a blocked cell, stepping to (%.2f, %.2f)' % ev)
            self._evacuating = True
            _a, vx, vy, wz = self._goto(ev[0], ev[1], self.pose[2], dt, plan=False,
                                        pos_tol=0.01, yaw_tol=math.pi)
            cmd = Command(vx, vy, wz)
        else:
            self._evacuating = False
        cmd.flip_action = self._decide_flip(obs)
        return cmd

    def _extra_drop_zone(self, final_zone):
        """Before the last drop (final_zone) the robot must enter with only the
        final dock's face loaded, and that face should hold a full wall (3).
        Return an already-built own rect zone that can take the surplus boxes
        as +1 extras, or None."""
        d = self._dock_override.get(final_zone.label)
        fz = d['face'] if d else None
        surplus = [f for f, n in enumerate(self.faces)
                   if n > 0 and not self.vertical[f] and (f != fz or n < 3)]
        if not surplus and sum(n for f, n in enumerate(self.faces) if f != fz) == 0:
            return None
        counts, cap = self.belief.zone_counts(), self.field.zone_max_boxes
        best, best_d = None, 1e9
        for z in self.field.team_zones(self.team):
            ov = self._dock_override.get(z.label)
            if (z.shape != 'rect' or z is final_zone or ov is None or ov['face'] not in surplus
                    or self.belief.is_excluded(z)):
                continue
            c = counts.get(z.name, 0)
            if (c < z.capacity or c >= cap or (self._door_label_of(z) and (
                    self._door_phase != 'done' or not self.field.door_accepts_extras))):
                continue
            dist = math.hypot(z.x - self.pose[0], z.y - self.pose[1])
            if dist < best_d:
                best, best_d = z, dist
        return best

    def _note_off_centre(self, zone, message):
        """World refused a drop as 'not centred (along +3 cm, across -5 cm, ..)': shift
        this zone's dock by the opposite amount (zone frame) so the retry is centred."""
        m = re.search(r'along ([+-]?\d+) cm, across ([+-]?\d+) cm', message or '')
        if not m or zone is None:
            return
        da, dc = -int(m.group(1)) / 100.0, -int(m.group(2)) / 100.0
        # 1 cm of slack beyond the rounding keeps us clear of the tolerance edge
        corr = self._dock_corr.setdefault(zone.label, [0.0, 0.0])
        corr[0] += da
        corr[1] += dc
        self.debug.say(f'{zone.label} dock shifted {da * 100:+.0f}/{dc * 100:+.0f} cm (zone frame), '
                       f'total {corr[0] * 100:+.0f}/{corr[1] * 100:+.0f}')

    def _door_label_of(self, zone):
        lbl = self.door_label or self._door_zone_label
        return zone is not None and lbl is not None and zone.label == lbl

    def _door_due(self):
        """The door zone is the next drop and we still mean to build the door."""
        if self._door_phase not in ('none', 'posts') or self.door_label is None:
            return False
        z = self.belief.best_drop(include_vertical=self.use_vertical_zones)
        return self._door_label_of(z)

    def _wants_vertical(self):
        """Should flip_face's horizontal load be flipped vertical now? Only for
        a circle-zone tower or the door posts - otherwise a flipped face would
        turn a plain rect drop into accidental door posts."""
        if self._circle_zones_pending():
            return True
        return (self._door_phase == 'none' and self._door_due()
                and self.faces[self.flip_face] == 3)

    def _door_posts_now(self, n):
        """Target is the door zone, no post down yet, and a full stack to post."""
        return (self.target_obj is not None and self._door_label_of(self.target_obj)
                and self._door_phase == 'none' and n == 3)

    def _zone_flip_pose(self):
        """The configured flip pose of the circle zone this flip-face drop is for (YC1/YC4), else None."""
        obj = self.target_obj
        if self.target_kind != 'drop' or obj is None or self.face != self.flip_face:
            return None
        return self.zone_flip_poses.get(getattr(obj, 'label', None))

    def _observe(self, obs):
        """Hook, every step: extra belief updates from what is observed (PlannerBrain)."""

    def _while_busy(self, obs):
        """Hook: a pick / drop is running (PlannerBrain: plan the next step now)."""

    def _pre_match(self, obs):
        """Hook, every step before the match starts (PlannerBrain: plan the opening now)."""

    def _flip_target_heading(self):
        """Body yaw that points the flip face at the dock of the drop this flip is for (None: no
        calibrated dock / not a flip-face drop)."""
        obj = self.target_obj
        if self.target_kind != 'drop' or obj is None or self.face != self.flip_face:
            return None
        d = self._dock_override.get(getattr(obj, 'label', None))
        if d is None:
            return None
        h = d['yaw'] + self.face_angles[d['face']]          # heading of the dropping face at the dock
        return _wrap(h - self.face_angles[self.flip_face])

    def _expects_vertical(self, n):
        """Does the drop target want the flip-face load vertical (door posts)?
        The rule brain: only a full stack at its door zone. PlannerBrain overrides."""
        return self._door_posts_now(n)

    def _wants_horizontal(self):
        """A vertical load on flip_face headed for a rect zone: un-flip it (door
        lid, a plain drop such as Y5, or the leftovers after a tower), except
        a full stack that is about to become the door's posts."""
        if (self.target_kind != 'drop' or self.target_obj is None or self.target_obj.shape != 'rect'
                or self.face != self.flip_face or self._discard):
            return False                # another face / a discard (standing boxes go down as they are)
        return not self._expects_vertical(self.faces[self.flip_face])

    def _flip_allowed_here(self):
        """Flip only (Max): docking for a rect drop with ANOTHER face while the flip-face stack is
        inside the castle (Y1 face 2, Y2 face 1, Y5), or in the flip area around the Y5 flip spot.
        Always also World's rule: the flip-face stack fully inside the castle."""
        if not self.flipmap.ok_at(*self.pose):
            return False
        zp = self._zone_flip_pose()
        if zp is not None:
            # a YC1/YC4 tower load flips at that circle's own flip pose, nowhere else
            return (math.hypot(self.pose[0] - zp[0], self.pose[1] - zp[1]) <= self.zone_flip_tol[0]
                    and abs(_wrap(self.pose[2] - zp[2])) <= self.zone_flip_tol[1])
        fx, fy, _ = self.flip_spot
        if math.hypot(self.pose[0] - fx, self.pose[1] - fy) <= self.flip_area_r:
            # Only already facing the drop this flip is for (Max): the robot turns on the way in and
            # arrives at Y5 facing 0 deg, instead of flipping at any heading and then spinning the
            # loaded faces between the built castle piles.
            want = self._flip_target_heading()
            return want is None or abs(_wrap(self.pose[2] - want)) <= self.flip_heading_tol
        return (self.target_kind == 'drop' and self.target_obj is not None
                and self.target_obj.shape == 'rect' and self.stage == 'final')

    def _await_flip(self, obs):
        """Docked, waiting for the flip this drop needs. Never forever: after await_flip_timeout
        the drop is abandoned (skipped briefly) so the brain can replan."""
        key = (self.target_obj.name, self.face)
        if self._await_key != key:
            self._await_key, self._await_t0 = key, obs.t
        if obs.t - self._await_t0 > self.await_flip_timeout and not obs.flip_busy:
            self.debug.say(f'{self.target_obj.label}: flip never happened, abandoning this drop')
            self._skip_until[self.target_obj.name] = obs.t + 3.0
            self.target_kind, self.target_obj, self.spot = None, None, None
            self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None
            self._await_key = None
            return Command()
        self.debug.state = 'awaiting flip'
        return Command(0.0, 0.0, 0.0)

    def _floor_target(self):
        """Pick target + spot for our floor pile: square to its long side, on the side we are on."""
        x, y, th, _n = self._floor_pile
        obj = StackSpec('floor', x, y, self.field.box_length, self.field.box_width, th, [], 'center')
        obj.label = 'floor'
        to_robot = math.atan2(self.pose[1] - y, self.pose[0] - x)
        appr = min((th + math.pi / 2, th - math.pi / 2), key=lambda a: abs(_wrap(a - to_robot)))
        obj.approach = [appr]
        spot = self.field._spot('pick', 'floor', x, y, self.field.box_width / 2, appr, self.r_in, 0.012, 0.10)
        return obj, spot

    def _floor_failed(self, why):
        self._floor_tries += 1
        if self._floor_tries >= self.regrip_max_tries:
            self.debug.say(f'regrip: giving up on the floor pile ({why}), {self._floor_pile[3]} boxes left there')
            self._floor_pile, self._floor_tries = None, 0

    def _hold_for_flip(self, obs):
        """Stop and wait for a flip before driving to the target? The rule brain never does."""
        return False

    def _watch_target(self, obs):
        """Every tick with a target: drop it (return True) if it is no longer worth going to.
        The rule brain never does (PlannerBrain overrides)."""
        return False

    def _decide_flip(self, obs):
        """Flip request for flip_face, independent of the main target so it can
        run while driving. Flip vertical only when needed (_wants_vertical);
        un-flip a lone vertical box headed for a rect zone. In-castle here is
        just the trigger - world.py re-checks the boxes' footprint."""
        if not obs.active or obs.flip_busy or obs.t < self._flip_block_until:
            return None
        ff, n = self.flip_face, self.faces[self.flip_face]
        if n <= 0 or not self._flip_allowed_here():
            return None
        if not self.vertical[ff]:
            return (ff,) if self._wants_vertical() else None
        if self._wants_horizontal():
            return (ff,)
        return None

    def _step_inner(self, obs, dt):
        self.pose = obs.pose
        self._now = obs.t
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
            backout, was_vertical = False, False
            if mode == 0 and self.target_kind == 'pick' and self.target_obj is not None \
                    and self.target_obj.name == 'floor':
                if success:
                    self.faces[face] += count
                    self._floor_pile, self._floor_tries = None, 0
                    self._last_kind, self._last_face = 'pick', face
                    self._regrip_held = True
                    backout = self.backout_pick > 0
                else:
                    self._floor_failed('pick failed: ' + message)
                self.debug.say(f'regrip pick: {message}')
            elif mode == 3 and self.target_kind == 'regrip':
                if success:
                    x, y, th = self._face_drop_pose(face)
                    self._floor_pile, self._floor_tries = (x, y, th, count), 0
                    self.faces[face] -= count
                    backout = True
                self.debug.say(f'regrip drop on face {face}: {message}')
            elif mode == 0 and self.target_kind == 'pick' and self.target_obj is not None:
                self.belief.on_pick_result(self.target_obj.name, success, count, obs.t)
                if success:
                    sz = getattr(self.target_obj, 'steal_zone', '')
                    if sz and self.field.zone(sz).team == self.team:     # repicked our own pile: the zone is empty again
                        self.belief.on_zone_emptied(sz, count)
                        if sz in self.dropped_log:
                            self.dropped_log.reverse(); self.dropped_log.remove(sz); self.dropped_log.reverse()
                    else:
                        self.picked_log.append(self.target_obj.name)
                    self.faces[face] += count
                    self._last_kind, self._last_face = 'pick', face
                    # only at the edge stacks (P1-P4, P7-P10): the robot ends up next to the border there.
                    # P5 / P6 sit mid-table, a back-out is wasted time (Max)
                    o = self.target_obj
                    edge = min(o.x, self.field.length - o.x, o.y, self.field.width - o.y)
                    backout = self.backout_pick > 0 and edge <= self.backout_pick_border
                else:
                    self._pick_failed(self.target_obj, message)
                self.debug.say(f'pick {self.target_obj.label}: {message}')
            elif mode == 1 and self.target_kind == 'drop' and self.target_obj is not None:
                was_vertical = self.vertical[face]
                if success and self.target_obj.shape == 'rect':
                    if was_vertical and self._door_phase == 'none':
                        self._door_phase = 'posts'
                        self._door_zone_label = self.target_obj.label
                    elif (not was_vertical and self._door_phase == 'posts'
                          and self._door_label_of(self.target_obj)):
                        self._door_phase = 'done'
                if not success:
                    self._note_off_centre(self.target_obj, message)
                    pose = None
                elif self.target_obj.shape == 'circle' and self._target_slot is not None:
                    # world.py's own zone_under_face/held_poly_slot never
                    # teleports a vertical drop to the face/zone centre -
                    # match that here too, or the costmap obstacle (and the
                    # belief this feeds into Nav2) sits in the middle of
                    # the zone while the real box is a slot-width to one
                    # side, same mismatch Max flagged against the marker.
                    pose = self._face_drop_pose_slot(face, self._target_slot)
                else:
                    pose = self._face_drop_pose(face)
                discard = self._discard
                self._discard = False
                if discard:
                    # extras next to a built wall: the zone count grows, its pile pose and the drop
                    # order stay as they are; the discarded stack is an obstacle of its own
                    self.belief.on_drop_result(self.target_obj.name, success, count, None)
                    if success and pose is not None:
                        self.discard_piles.append(tuple(pose))
                        self._static_key = None
                else:
                    self.belief.on_drop_result(self.target_obj.name, success, count, pose)
                if success and not discard:
                    self.dropped_log.append(self.target_obj.name)
                if success:
                    self._last_kind, self._last_face = 'drop', face
                    self.faces[face] -= count
                    if self.faces[face] == 0:
                        self.vertical[face] = False   # mirrors world.py's own reset
                        self.vertical_slots[face] = [False, False, False]
                    elif self._target_slot is not None:
                        # Only the targeted slot - never "whichever", since
                        # the dock pose we sent was shifted so only that
                        # slot's footprint overlapped the zone (see
                        # _dock_spot/_next_slot).
                        self.vertical_slots[face][self._target_slot] = False
                    self._drop_retry_offset.pop(self.target_obj.name, None)
                self.debug.say(f'drop {self.target_obj.label}: {message}')
                backout = success
                self._target_slot = None
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
                if self.target_kind == 'regrip':
                    bd = self.regrip_backout
                elif mode == 0:
                    # after a pick (Max: robot stuck after P8): the robot now carries, so the global
                    # costmap footprint is the loaded circle; at an edge stack the robot centre is 0.22 m
                    # from the table border, inside it - step back from the wall before Nav2 plans
                    bd = self.backout_pick
                elif mode == 1 and was_vertical:
                    bd = self.backout_vertical
                else:
                    bd = self.backout_dist
                if mode == 1 and sum(self.faces) > 0:
                    # still carrying: the global costmap footprint is the loaded circle, the robot centre
                    # must leave it around the pile just dropped or Nav2 can't plan from here (Max: evacuate)
                    bd = max(bd, self.backout_loaded)
                self._backout_target = (x - bd * math.cos(face_dir), y - bd * math.sin(face_dir), th)
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
                if self.vertical[flip_face]:
                    # Horizontal -> vertical: the n held boxes take their n
                    # fixed slots (same order/rule as world.py's own
                    # _update_flip_action).
                    n = self.faces[flip_face]
                    self.vertical_slots[flip_face] = [i < n for i in range(3)]
                else:
                    self.vertical_slots[flip_face] = [False, False, False]
            if not flip_success:
                self._flip_block_until = obs.t + 1.0     # don't hammer a refused flip every tick
            self.debug.say(f'flip face {flip_face}: {flip_message}')

        self.debug.belief = {s.name: round(self.belief.believed_count(s, obs.t), 1)
                             for s in self.field.stacks}

        self._observe(obs)
        if not obs.active:
            self._pre_match(obs)
            self.debug.state = 'waiting'
            return Command()

        if obs.tool_busy:
            self.debug.state = self.target_kind or 'busy'
            self.v = (0.0, 0.0)
            self.wz = 0.0
            self._while_busy(obs)
            return Command()

        self._rebuild_statics(obs.t)

        if self.target_kind is not None and self.blocked_t > self.blocked_timeout:
            label = self.target_obj.label if self.target_obj is not None else 'home'
            self.debug.say(f'stuck heading to {label}, abandoning')
            if self.target_kind == 'pick' and self.target_obj.name == 'floor':
                self._floor_failed('stuck')
            elif self.target_kind == 'pick':
                self._stuck_pick(obs)
            elif self.target_kind == 'drop':
                self._stuck_drop(obs)
            else:
                self._stuck_other(obs)
            self.target_kind, self.target_obj, self.spot = None, None, None
            self.stage, self.path, self.blocked_t = 'approach', [], 0.0

        if (self.target_kind == 'backout' and not obs.flip_busy and self._backout_target is not None
                and math.hypot(self.pose[0] - self._backout_target[0],
                               self.pose[1] - self._backout_target[1]) <= self.backout_handover):
            # close enough: hand over to the next target in this same step, still moving (Max: no
            # stop between the back-out and the Nav2 leg)
            self.target_kind, self.target_obj, self.spot = None, None, None
            self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None

        if self.target_kind is None and self._floor_pile is not None:
            # our regrip pile is on the floor: picking it back up with flip_face comes first
            ff = self.flip_face
            if self.faces[ff] > 0 or self.vertical[ff]:
                self._floor_failed('flip face no longer free')
            else:
                obj, spot = self._floor_target()
                self.target_kind, self.target_obj, self.spot, self.face = 'pick', obj, spot, ff
                self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None

        if self.target_kind is None:
            kind, obj = self._decide(obs)
            if kind == 'home':
                self.target_kind, self.target_obj, self.spot = 'home', None, None
            elif kind == 'wait':
                pass  # target_kind stays None - hold position, retry next tick
            elif kind == 'look':
                # obj = the scouting viewpoint (PlannerBrain: drive there, look at the stacks, replan)
                self.target_kind, self.target_obj, self.spot = 'look', None, None
            elif kind == 'flip':
                # obj = the robot pose to flip from (PlannerBrain: the plan starts with a flip)
                self.target_kind, self.target_obj, self.spot = 'flipgo', None, None
                self._flipgo_pose, self._detour_hold = obj, 0.0
                self._flipgo_want = not self.vertical[self.flip_face]
            elif kind == 'regrip':
                # obj = the face whose stack goes on the floor right here (see _regrip_step)
                self.target_kind, self.target_obj, self.spot, self.face = 'regrip', None, None, obj
            elif kind == 'drop' and obj.label in self._dock_override:
                # Max's hand-tuned dock pose from the artifact, exact x/y/yaw
                # and the face that reproduces it - bypasses drop_spots()/
                # _best_face entirely for this zone. For a circle zone, the
                # dock pose itself depends on which of the 3 vertical slots
                # is about to be dropped - started here at the centre/None
                # slot (nothing may be flipped yet), kept in sync with the
                # live belief every tick below (see "live circle-zone slot").
                # The dock pose fixes position + approach; the face is the
                # strategy's call (planner passes _drop_face), the body heading
                # follows as dock heading - that face's angle (see `heading` below).
                face = self._drop_face if self._drop_face is not None else self._dock_override[obj.label]['face']
                if self._drop_face is None and obj.shape == 'rect' and self.faces[face] == 0:
                    # rect dock: any loaded horizontal face can drop there (the body turns to suit it)
                    face = next((f for f in range(len(self.faces)) if self.faces[f] and not self.vertical[f]), face)
                if self.faces[face] == 0:
                    self.debug.say(f'{obj.label} needs boxes on face {face}, deferring')
                    self._skip_until[obj.name] = obs.t + 1.5
                    self.target_kind, self.target_obj, self.spot = None, None, None
                else:
                    self._target_slot = None
                    self._spot_side = 0
                    spot = self._discard_spot(obj.name) if self._discard else None
                    if spot is None:
                        spot, _ = self._dock_spot(obj.label, None)
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
                if kind == 'pick' and self._pick_goal_blocked(obj, spot):
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
                    face = (self._drop_face if (kind == 'drop' and self._drop_face is not None)
                            else self._best_face(spot.face_heading, 'free' if kind == 'pick' else 'loaded',
                                                 avoid=avoid))
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
        if self._watch_target(obs):          # PlannerBrain: target dropped (opponent took it, replan, ...)
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

        if self.target_kind == 'look':
            return self._look_step(obs, dt)

        # Flip detour: a tower/door-posts load that still has to be flipped can
        # only flip with the boxes fully inside the castle for the whole flip
        # (the robot centre itself may be outside). Stack inside right now:
        # stop and flip here. Otherwise go to the nearest pose that puts it
        # inside, on the way to the dock (FlipMap), and HOLD there until it is
        # vertical (a short give-up keeps a refused flip from hanging us).
        ff = self.flip_face
        need_flip = (self.target_kind == 'drop' and self.target_obj is not None and self.face == ff
                     and not self.vertical[ff] and self.faces[ff] > 0
                     and (self.target_obj.shape == 'circle' or self._expects_vertical(self.faces[ff])))
        if not need_flip:
            self._detouring, self._detour_hold, self._flip_pose = False, 0.0, None
        elif self._detour_hold < 4.0:
            if self._flip_allowed_here():
                self._detour_hold += dt
                self.debug.state = 'flip hold'
                self.v, self.wz = (0.0, 0.0), 0.0
                return Command(0.0, 0.0, 0.0)
            if self._flip_pose is None:
                # the flip pose by the Y5 flip spot (most room) that already faces this drop's dock,
                # so the turn happens on the way there, not next to the piles after the flip
                want = self._flip_target_heading()
                self._flip_pose = (self._zone_flip_pose()
                                   or (self.flipmap.nearest(self.flip_spot[0], self.flip_spot[1], heading=want)[1]
                                       if want is not None else None)
                                   or self.flip_spot)
            if self._flip_pose is not None:
                self._detouring = True
                fx, fy, fth = self._flip_pose
                near = math.hypot(self.pose[0] - fx, self.pose[1] - fy) < 0.15
                _arrived, vx, vy, wz = self._goto(fx, fy, fth, dt, plan=not near,
                                                  pos_tol=0.02, yaw_tol=math.radians(5))
                self.debug.path = list(self.path)
                self.debug.state = 'flip detour'
                if _arrived:
                    self._detour_hold += dt        # arrived but still not ok (shouldn't happen): don't hang
                    return Command(0.0, 0.0, 0.0)
                return Command(vx, vy, wz, goal_pose=None if near else (fx, fy, fth))

        if self.target_kind == 'backout':
            if obs.flip_busy:
                # a flip started at the dock is still running: moving now can take the flipping
                # stack out of the castle mid-flip (world aborts it) - hold until it's done
                self.blocked_t, self._prev_xy = 0.0, None
                self.debug.state = 'backout: waiting for flip'
                return Command()
            bx, by, bth = self._backout_target
            arrived, vx, vy, wz = self._goto(bx, by, bth, dt, plan=False)
            self.debug.path = list(self.path)
            if arrived or self.blocked_t > self.backout_timeout:
                if not arrived:
                    self.debug.say('backout stuck (opponent blocking?), giving up on it')
                self.target_kind, self.target_obj, self.spot = None, None, None
                self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None
                return Command()
            return Command(vx, vy, wz)

        if self.target_kind == 'flipgo':
            ff = self.flip_face
            done = self.vertical[ff] == self._flipgo_want or self.faces[ff] == 0
            if done or self._detour_hold > 4.0:
                if not done:
                    self.debug.say('flip at the flip pose never happened, replanning')
                self.target_kind, self.target_obj, self.spot, self._flipgo_pose = None, None, None, None
                self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None
                return Command()
            if self._flip_allowed_here():
                self._detour_hold += dt
                self.debug.state = 'flip hold'
                self.v, self.wz = (0.0, 0.0), 0.0
                return Command(0.0, 0.0, 0.0)
            fx, fy, fth = self._flipgo_pose
            near = math.hypot(self.pose[0] - fx, self.pose[1] - fy) < 0.15
            arrived, vx, vy, wz = self._goto(fx, fy, fth, dt, plan=not near,
                                             pos_tol=0.02, yaw_tol=math.radians(5))
            self.debug.path = list(self.path)
            self.debug.state = 'flip detour'
            if arrived:
                self._detour_hold += dt
                return Command(0.0, 0.0, 0.0)
            return Command(vx, vy, wz, goal_pose=None if near else (fx, fy, fth))

        if self.target_kind == 'regrip':
            # put this face's stack on the floor where we stand: stop, then DROP_FLOOR (mode 3)
            self.debug.state = 'regrip'
            vx, vy, wz = self._vel_est
            if math.hypot(vx, vy) > 0.03 or abs(wz) > 0.1:
                self.v, self.wz = (0.0, 0.0), 0.0
                return Command(0.0, 0.0, 0.0)
            return Command(0.0, 0.0, 0.0, action=(self.face, 3, 0))

        # A flip that has to happen HERE before we drive off (PlannerBrain: a plan
        # that starts with a flip) - hold still so the boxes don't leave the
        # castle mid-flip.
        if self.target_kind in ('pick', 'drop') and self.stage == 'approach' and self._hold_for_flip(obs):
            self.debug.state = 'flip hold'
            self.v, self.wz = (0.0, 0.0), 0.0
            return Command(0.0, 0.0, 0.0)

        # Live circle-zone slot: a drop target is locked in (face/target_obj)
        # well before the flip that decides which slot it'll actually be -
        # flip can land anywhere between target selection and arrival (see
        # _decide_flip). Re-resolve every tick so the dock pose - not just
        # the initial centre-pose guess - tracks which box is really about
        # to drop (_next_slot is None, i.e. centre, until vertical[face]
        # is set, matching the "nothing flipped yet" case safely).
        if (self.target_kind == 'drop' and self.target_obj is not None
                and self.target_obj.label in self._dock_override
                and self._dock_override[self.target_obj.label]['shape'] == 'circle'):
            live_slot = (self._clear_slot(self.face, self.target_obj.label)
                         if self.vertical[self.face] else None)
            if live_slot != self._target_slot:
                self._target_slot = live_slot
                self.spot, _ = self._dock_spot(self.target_obj.label, live_slot)

        sp = self.spot
        heading = _wrap(sp.face_heading - self.face_angles[self.face])
        if self.stage == 'approach':
            arrived, vx, vy, wz = self._goto(sp.pre_x, sp.pre_y, heading, dt,
                                             pos_tol=self.nav_pos_tol, yaw_tol=self.nav_yaw_tol)
            self.debug.path = list(self.path)
            if not arrived:
                d_pre = math.hypot(sp.pre_x - self.pose[0], sp.pre_y - self.pose[1])
                if d_pre < self.dock_handover and self.blocked_t >= self.dock_handover_stall:
                    arrived = True              # Nav2 / the costmap won't take us closer: dock directly
                    self.debug.say(f'{getattr(self.target_obj, "label", "")}: approach point not reached '
                                   f'({d_pre * 100:.0f} cm), docking directly')
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
            risky = (self.target_kind == 'pick' and self.target_obj.name != 'floor'
                     and self.belief.confidence(self.target_obj, obs.t) < self.verify_confidence)
            if risky:
                self.stage, self.blocked_t, self._prev_xy = 'verify', 0.0, None
                self._verify_t0 = obs.t
            else:
                self.stage, self.path, self.blocked_t, self._prev_xy = 'final', [], 0.0, None
            # Stop at the handover instead of carrying Nav2's speed into the docking stage (the
            # carried side velocity is what made it curve / overshoot): docking ramps up from 0.
            if self.dock_settle_s > 0:
                self.v, self.wz = (0.0, 0.0), 0.0
                self._settle_until = obs.t + self.dock_settle_s
            else:
                # smooth handover (Max): the docking controller continues from the robot's real
                # motion, no stop; it corrects the remaining heading while closing in
                self.v = self._vel_est[:2]
                self.wz = self._vel_est[2]

        if self.stage == 'final' and obs.t < self._settle_until:
            vx_, vy_, wz_ = self._vel_est
            if math.hypot(vx_, vy_) > self.dock_settle_v or abs(wz_) > 3 * self.dock_settle_v:
                self.debug.state = 'dock: settling'
                return Command()
            self._settle_until = -1.0

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
            # (dock poses: the world checks the dock pose for circles too, same tolerance as rects)
            arrived, vx, vy, wz = self._goto(sp.x, sp.y, heading, dt, plan=False,
                                             pos_tol=self.drop_pos_tol, yaw_tol=self.drop_yaw_tol)
        elif self.target_kind == 'drop':
            arrived, vx, vy, wz = self._goto(sp.x, sp.y, heading, dt, plan=False,
                                             pos_tol=self.drop_pos_tol, yaw_tol=self.drop_yaw_tol)
        else:
            side = self.face != 0
            arrived, vx, vy, wz = self._goto(sp.x, sp.y, heading, dt, plan=False,
                                             pos_tol=self.pick_pos_tol_side if side else self.pick_pos_tol,
                                             yaw_tol=self.pick_yaw_tol_side if side else self.pick_yaw_tol)
        self.debug.path = list(self.path)
        if arrived and self.target_kind == 'drop' and math.hypot(*self._vel_est[:2]) > self.release_v:
            # (drops only: a pick is done as soon as the robot is at the dock - grippers close on the way in)
            self.debug.state = f'{self.target_kind}: stopping'
            return Command(0.0, 0.0, 0.0)       # in tolerance but still rolling: brake first
        if arrived:
            if self.target_kind == 'drop' and self.target_obj.shape == 'rect' \
                    and self.face == self.flip_face and not self._discard:
                # Door posts want a full vertical stack (wait for the flip);
                # a lone vertical box needs its un-flip first. A vertical face
                # holding 2-3 boxes can't be un-flipped: just drop it (posts).
                n = self.faces[self.face]
                door_posts = self._expects_vertical(n)
                can_flip = self._flip_allowed_here()
                wait = (n > 0 and ((not self.vertical[self.face] and door_posts)
                                   or (self.vertical[self.face] and not door_posts and can_flip)))
                if wait:
                    return self._await_flip(obs)
            if self.target_kind == 'drop' and self.target_obj.shape == 'circle' \
                    and not self.vertical[self.face]:
                # Not vertical yet - the flip for this face is requested
                # independently every step (_decide_flip, concurrent world.py
                # slot), including while we were still driving here. Just
                # hold position; once obs.flip_result reports it done this
                # branch stops matching and the real DROP goes out below.
                return self._await_flip(obs)
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
