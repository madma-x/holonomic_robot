"""PlannerBrain: StrategyCore's execution layer with the decision replaced by a
beam-search plan (receding horizon: replan whenever a target completes, run only
the first action). Same step(obs, dt) -> Command interface; ROS-free.
"""
import os
import math
import time
from dataclasses import replace
from itertools import permutations

import numpy as np
import yaml

from table_sim.geometry import closest_point_on_poly

from table_sim.field import other_team  # noqa: F401  (kept for symmetry with core)

from .core import Command, Observation, StrategyCore, _wrap  # noqa: F401  (headless looks these up on the brain module)
from .plan_state import NONE, TOWER, WALL, DOOR, PlanModel, PlanState, apply, points
from .planner import plan, plan_robust
from .scenarios import ScenarioSet
from .rules import build_guard
from .scout import Viewpoints


class PlannerBrain(StrategyCore):
    def __init__(self, field, team, params=None, sim_cfg=None):
        p = dict(params or {})
        # The rule-based brain retries a blocked target forever (1e6 s). The
        # planner has alternatives, so give up quickly and replan around it.
        p['blocked_timeout_s'] = float(p.get('planner_blocked_timeout_s', 8.0))
        super().__init__(field, team, p)
        self._known_skips = {}          # name -> the `until` of a ban registered by us (anything else is a new skip from core)
        self._ban_info = {}             # name -> {'kind', 't', 'xy'}: why it is banned (for early lifting)
        self._ban_n = {}                # zone name -> consecutive opponent-caused stuck bans (escalation)
        self._side_bad = {}             # (stack/zone name, approach side) -> until: opponent there / stuck there
        self._imposs = {}               # (name, side, signature) -> number of sides: stuck with nobody near
        self._dropped_seen = 0
        if sim_cfg is None:
            from table_sim.headless import default_sim_file
            with open(default_sim_file()) as fh:
                sim_cfg = yaml.safe_load(fh)
        self.beam = int(p.get('plan_beam', 150))
        self.depth = int(p.get('plan_depth', 26))
        from . import planner as _pl          # reachable-points beam ranking (door / towers first)
        _pl.POT_DOOR_W = float(p.get('plan_pot_door_w', _pl.POT_DOOR_W))
        _pl.POT_TOWER_W = float(p.get('plan_pot_tower_w', _pl.POT_TOWER_W))
        self.rank_tw = float(p.get('plan_rank_time_weight', 0.0))
        self.time_w = float(p.get('plan_time_weight', 0.02))   # points per second of finish time
        dp = p.get('dock_poses_file')
        # no forced dock face: the search chooses the face, the dock pose gives position + approach
        self.model = PlanModel(field, sim_cfg, team, dp)
        self.model.travel_factor = float(p.get('plan_travel_factor', 1.3))
        self.model.pick_overhead = float(p.get('plan_pick_overhead_s', 1.2))
        self.model.drop_overhead = float(p.get('plan_drop_overhead_s', 1.5))
        self.model.home_reserve = float(p.get('plan_home_reserve_s', 3.0))
        # hard stop while executing: give up the current target and go home when the time left drops
        # below the realistic time home + this (a drop already being released is let finish)
        self.home_abort_s = float(p.get('home_abort_margin_s', 1.0))
        self._timing = None       # (label, start t, planned s) of the action being executed
        self._timing_n = 0
        self.model.deny_w = float(p.get('deny_weight', 1.0))
        self.model.deny_horizon = float(p.get('deny_horizon_s', 50.0))
        self.model.deny_tau = float(p.get('deny_tau_s', 15.0))
        self.model.castle_w = float(p.get('castle_risk_w', 6.0))
        self.model.contact_w = float(p.get('contact_risk_w', 6.0))
        self.model.unseen_cost = float(p.get('steal_unseen_cost', 3.0))
        self.opening_cfg = {k: [x for x in p.get('opening_' + k, [])] for k in ('bottom', 'top')}
        self.opening_default = str(p.get('opening_default', 'bottom'))
        self.model.opening_bonus = float(p.get('opening_bonus', 2.0))
        self.opening_until = float(p.get('opening_until_s', 12.0))
        self.opening_margin = float(p.get('opening_side_margin_m', 0.1))
        self.opening_block_d = float(p.get('opening_block_dist_m', 0.25))    # opponent this close to our centre stack itself...
        self.opening_block_s = float(p.get('opening_block_s', 0.5))          # ...for this long: switch edge
        self.opening_blocked_timeout = float(p.get('opening_blocked_timeout_s', 2.0))   # stuck on the way: give up fast
        self._open_skip, self._open_blk, self._open_t = set(), {}, 0.0
        self._open_side, self._open_switched = None, False     # opening edge, committed (see _opening)
        self._pre, self._pre_done = None, False                 # decision made before the start (_pre_match)
        self._spec, self._spec_for, self._replan_due, self._late_for = None, None, False, None
        self._force_search = False
        self.plan_budget = float(p.get('plan_budget_s', 0.6))      # in-match plan search time limit (s)
        self.replan_opp_near = float(p.get('replan_opp_near_m', 0.6))
        self.replan_late_s = float(p.get('replan_late_s', 2.0))
        self._dock_dwell, self._dock_t = {}, 0.0                # opponent docked at a stack: seconds (_observe)
        self.taken_dwell = float(p.get('taken_dock_dwell_s', 0.6))
        self.steal_prior_t = float(p.get('steal_prior_t_s', 25.0))
        self.steal_prior_boxes = int(p.get('steal_prior_boxes', 3))
        self.steal_recheck_s = float(p.get('steal_recheck_s', 20.0))
        self.model.contact_r = float(p.get('contact_risk_r_m', 0.9))
        self.model.castle_t0 = float(p.get('castle_risk_t0_s', 8.0))
        self.model.castle_t1 = float(p.get('castle_risk_t1_s', 45.0))
        self.model.castle_near = float(p.get('castle_near_m', 0.45))
        self.model.keepout_from = float(p.get('keepout_from_s', 20.0))
        self.model.scen_p = float(p.get('pick_block_p', 0.85))
        self.model.crowded_pick_s = float(p.get('plan_crowded_pick_s', 1.0))
        self.model.tower_partial_cost = float(p.get('tower_partial_cost', 1.0))
        self.model.pick_factor = float(p.get('plan_pick_time_factor', 1.0))
        self.model.drop_factor = float(p.get('plan_drop_time_factor', 1.0))
        self.model.steal_tower_slack = float(p.get('plan_steal_tower_slack_s', 3.0))
        self.model.door_earliest = float(p.get('door_earliest_s', 60.0))
        self.model.cross_time = float(p.get('plan_cross_time_s', 1.0))
        # turning cost per leg (Max): s per 90 deg of body heading change; 0 = off
        self.model.turn_s_per_rad = float(p.get('plan_turn_s_per_90deg', 0.25)) / (math.pi / 2)
        self.face_swap_margin = math.radians(float(p.get('face_swap_margin_deg', 20.0)))
        self.hysteresis_loaded = float(p.get('plan_hysteresis_loaded', 4.0))   # keep the current plan while loaded
        self._opp_moved = 0.0     # farthest the opponent was seen from its start pose
        self.idle_after_s = float(p.get('idle_evidence_after_s', 2.0))
        self.idle_still_m = float(p.get('idle_still_m', 0.15))
        self.hysteresis = float(p.get('plan_hysteresis', 2.0))
        self._incumbent = None
        self._present_t = {}
        self._flip_ahead = False
        base_guard = build_guard(self.model, p.get('drop_rules'))
        self.opening_strict = bool(p.get('opening_strict', True))
        self._base_blocked_timeout = float(p.get('planner_blocked_timeout_s', 8.0))

        def guard(st, a, _b=base_guard, _m=self.model):
            # commit to the opening: while it is not completed, the next pick must be its next stack
            if self.opening_strict and a[0] == 'pick':
                op, k = _m.opening, len(st.picked)
                if k < len(op) and tuple(st.picked) == tuple(op[:k]) and a[1] != op[k]:
                    return False
            # ...and the first round comes back with all faces loaded (Max): no first drop while a
            # face is empty and some stack still has boxes - a gone opening stack is replaced by
            # any other one (the opening only fixes the order of the stacks still there)
            if a[0] == 'drop' and not st.dropped and 0 in st.faces:
                if any(c > 0 for i, c in enumerate(st.stacks) if not _m.stacks[i].steal_zone):
                    return False
            # no stealing from the opponent's zones while any pick stack P1-P10 still has boxes
            # (disqualification) - even one we may no longer pick (P9/P10 after the keep-out starts)
            if a[0] == 'pick':
                sz = _m.stacks[a[1]].steal_zone
                if sz and self.field.zone(sz).team != self.team:
                    if any(c > 0 for i, c in enumerate(st.stacks) if not _m.stacks[i].steal_zone):
                        return False
            return _b(st, a)
        self.guard = guard
        self.threshold = float(p.get('points_threshold', 50.0))
        # P(points >= threshold) at or above this counts as safe enough: those plans are then ranked by
        # expected points (Max: it locked in a certain 57 with the door at 53 s instead of ~75)
        self.p_enough = float(p.get('plan_p_ok_enough', 0.8))
        self.scen = ScenarioSet(field, team, p) if p.get('use_scenarios', True) else None
        self.model.scen = self.scen
        self._taken_reported = set()
        self.plan_actions = []
        self.plan_value = 0.0
        self.plan_ms = 0.0
        # Pick watchdog (_watch_target): give up a pick target as soon as the opponent is AT that
        # stack (docked: within take_dock_m of its footprint, nearest stack, nearly stopped) or
        # clearly heading there first (within take_head_m, moving, stack nearest in its +-cone,
        # arriving take_eta_margin_s before us), held for take_debounce_s; ban it take_ban_s.
        self.take_dock_d = float(p.get('take_dock_m', 0.15))
        self.take_dock_v = float(p.get('take_dock_speed', 0.10))
        self.take_head_d = float(p.get('take_head_m', 0.6))
        self.take_move_v = float(p.get('take_move_speed', 0.15))
        self.take_cone = math.radians(float(p.get('take_cone_deg', 20.0)))
        self.take_eta_margin = float(p.get('take_eta_margin_s', 1.0))
        self.take_debounce = float(p.get('take_debounce_s', 0.5))
        self.take_sigma = float(p.get('take_max_sigma_m', 0.1))
        self.take_ban = float(p.get('take_ban_s', 3.0))             # opponent docked at / heading to the stack
        self.take_lift_d = float(p.get('take_lift_m', 0.6))         # ...lifted early: camera sees it full, opponent this far
        self.stuck_ban = [float(x) for x in p.get('stuck_ban_s', [3.0, 6.0, 12.0])]   # stuck with the opponent close: escalates per target
        self.stuck_opp_d = float(p.get('stuck_opp_near_m', 0.6))    # 'opponent close to us' when stuck
        self.stuck_lift_d = float(p.get('stuck_lift_m', 0.9))       # ...lifted early when it is this far from the target
        self.side_ban = float(p.get('side_ban_s', 3.0))             # an approach side with the opponent on it is avoided this long
        self.side_debounce = float(p.get('side_debounce_s', 0.3))
        self._side_since = None
        self.home_margin = float(p.get('home_replan_margin_s', 3.0))  # keep replanning while driving home / parked
        # Door endgame scouting
        self.scout_on = bool(p.get('scout_on', True))
        self.scout_from = float(p.get('scout_from_s', 40.0))         # the door endgame: not before this match time
        self.scout_confirm = float(p.get('scout_confirm_s', 10.0))   # a sighting this fresh with a full stack = 'confirmed full'
        self.scout_p_lo = float(p.get('scout_p_taken_lo', 0.15))     # an unseen stack is uncertain once P(taken) is at least this
        self.scout_margin = float(p.get('scout_margin_s', 2.0))
        self.scout_wait = float(p.get('scout_wait_s', 2.0))
        self.scout_gain = float(p.get('scout_door_gain', 15.0))      # points a door adds over the wall it replaces
        self.scout_tcost = float(p.get('scout_time_cost', 0.4))      # viewpoint score: #stacks seen - this * eta (s)
        self._look = None
        self._look_tried = {}
        self._look_fail_until = -1.0
        cam = dict(sim_cfg.get('camera', {}))
        self.viewpoints = Viewpoints(field, team, cam, self.face_angles, self.r_out) if self.scout_on else None
        self.pick_replan_s = float(p.get('pick_replan_s', 1.0))     # replan while driving to a pick / home (0 = off)
        self.replan_opp_move = float(p.get('replan_opp_move_m', 0.20))  # periodic replan only if the opponent moved this much (or counts/bans changed)
        self._plan_sig = None
        self._take_acc, self._take_t = 0.0, None
        self._last_plan_t = -1e9
        self._current_action = None
        self._pending = None

    # ------------------------------------------------------------ state
    def _plan_state(self, obs):
        m = self.model
        m.travel.set_node('cur', self.pose[:2])
        o = self.tracker.opp
        m.opp_now = (obs.t, o.x, o.y, o.vx, o.vy) if (o.seen_ever and o.sigma < 1.0) else None
        m.in_castle['cur'] = self.field.in_castle(self.team, self.pose[0], self.pose[1])
        # camera/pick-based counts only: opponent depletion is the scenarios' job now
        counts, unseen = [], set()
        for i, st in enumerate(m.stacks):
            c = max(0, int(round(self.belief._count[st.name])))
            if st.steal_zone and self.field.zone(st.steal_zone).team != self.team:
                seen = self.belief._last_seen[st.name]
                if not (seen > 0 and obs.t - seen <= self.steal_recheck_s):
                    # not looked at lately: the opponent has probably built something there by now
                    c = self.steal_prior_boxes if obs.t >= self.steal_prior_t else 0
                    if c:
                        unseen.add(i)
            counts.append(c)
        counts = tuple(counts)
        m.unseen = unseen
        m.opening = self._opening(obs, m)
        zc = self.belief.zone_counts()
        zbox, zvert, zkind = [], [], []
        door_zone, door_used = -1, False
        for i, z in enumerate(m.zones):
            c = int(zc.get(z.name, 0))
            if z.shape == 'circle':
                zbox.append(min(c, 1)); zvert.append(min(c, 1)); zkind.append(TOWER if c else NONE)
            elif self._door_label_of(z) and self._door_phase == 'posts':
                zbox.append(2); zvert.append(2); zkind.append(NONE); door_zone = i
            elif self._door_label_of(z) and self._door_phase == 'done':
                zbox.append(max(c, 3)); zvert.append(2); zkind.append(DOOR); door_used = True
            else:
                zbox.append(c); zvert.append(0); zkind.append(WALL if c >= z.capacity else NONE)
        idx = lambda names, d: tuple(d[n] for n in names if n in d)   # noqa: E731
        return PlanState(obs.t, 'cur', tuple(self.faces), tuple(self.vertical), counts,
                         tuple(zbox), tuple(zvert), tuple(zkind), door_used, door_zone,
                         idx(self.dropped_log, m.zone_idx), idx(self.picked_log, m.stack_idx),
                         last=self._last_kind, dock_f=self._last_face, regrip_flip=self._regrip_flip_due(),
                         head=self.pose[2])

    def _swap_pick_face(self, acts, i, a):
        """Faces 1 and 2 both empty: pick with whichever turns less from where the robot faces NOW (the plan
        may come from a predicted heading); the plan's later face numbers are swapped to match (Max)."""
        m, f, ff = self.model, a[2], self.flip_face
        if f == ff or self.faces[f] or self.vertical[f] or f in m.dock_face.values():
            return acts, a
        others = [g for g in range(len(self.faces)) if g not in (f, ff) and not self.faces[g]
                  and not self.vertical[g] and g not in m.dock_face.values()]
        if not others:
            return acts, a
        node, h = m.stack_nodes[a[1]], self.pose[2]

        def turn(g):
            hg = m.head_at(node, g)
            return math.inf if hg is None else abs(_wrap(hg - h))
        g = min(others, key=turn)
        if turn(g) + self.face_swap_margin >= turn(f):
            return acts, a
        swap = {f: g, g: f}

        def sw(b):
            if b[0] in ('pick', 'drop', 'discard'):
                return (b[0], b[1], swap.get(b[2], b[2])) + tuple(b[3:])
            if b[0] == 'regrip':
                return (b[0], swap.get(b[1], b[1]))
            return b
        new = list(acts[:i]) + [sw(b) for b in acts[i:]]
        self.debug.say('face swap: %s -> face %d (turn %.0f deg instead of %.0f), later faces swapped'
                       % (self._fmt(a), g, math.degrees(turn(g)), math.degrees(turn(f))))
        return new, new[i]

    def _regrip_flip_due(self):
        """The regripped stack is still lying on the flip face: later plans must flip it, not wall it."""
        ff = self.flip_face
        if self._regrip_held and (self.faces[ff] == 0 or self.vertical[ff]):
            self._regrip_held = False
        return self._regrip_held

    def _opening(self, obs, m):
        """First picks along ONE edge: the default one (P6 -> P8 -> P10), committed. No guessing where
        the opponent goes (Max: it heads for its own P10, then P9 - reading that as 'coming for P6' made
        the opening flip-flop). The only reason to switch to the other edge: before we picked our centre
        stack, the opponent is right ON it (opening_block_dist_m of the stack itself, opening_block_s)
        or it is seen gone."""
        if obs.t > self.opening_until or not any(self.opening_cfg.values()):
            return ()
        if self._open_side is None:
            self._open_side = self.opening_default
        o = self.tracker.opp
        labels = {st.label: i for i, st in enumerate(m.stacks)}
        dt, self._open_t = min(0.2, max(0.0, obs.t - self._open_t)), obs.t   # per tick (no jump on the first call)
        side_list = [labels[x] for x in self.opening_cfg.get(self._open_side, []) if x in labels]
        if side_list and not self._open_switched:
            c = side_list[0]                              # our centre stack (P6 / P5)
            st = m.stacks[c]
            if st.name not in self.picked_log:
                on_it = False
                if o.seen_ever and o.visible:
                    _, d = closest_point_on_poly(np.array([o.x, o.y]), np.asarray(st.poly))
                    on_it = d < self.opening_block_d
                self._open_blk[c] = self._open_blk.get(c, 0.0) + dt if on_it else 0.0
                if self._open_blk[c] > self.opening_block_s or self.belief._count[st.name] < 0.5:
                    other = next((k for k in self.opening_cfg if k != self._open_side), None)
                    if other is not None:
                        self.debug.say(f'opening: opponent on {st.label} first, switching to the {other} edge')
                        self._open_side, self._open_switched = other, True
                        side_list = [labels[x] for x in self.opening_cfg.get(other, []) if x in labels]
        eff = []
        for i in side_list:
            name = m.stacks[i].name
            if name in self.picked_log:
                eff.append(i)       # done by us: stays in the list (the guard matches our k-th pick to entry k)
            elif self.belief._count[name] >= 0.5 and self._skip_until.get(name, -1) <= obs.t:
                eff.append(i)       # a stack seen gone / banned is skipped, the rest still follow
        return tuple(eff)

    # --------------------------------------------------------- decision
    def _make_guard(self, obs):
        """The plan rules + what is banned / impossible right now."""
        banned = self._banned(obs.t) | self._impossible_names()
        guard, m = self.guard, self.model
        if banned:
            base = self.guard

            def guard(s, a, _b=banned, _base=base):
                if a[0] == 'pick' and m.stacks[a[1]].name in _b:
                    return False
                if a[0] == 'drop' and m.zones[a[1]].name in _b:
                    return False
                return _base(s, a)
        return guard

    def _search(self, obs, s0, guard, budget):
        """One plan search from s0 (budget: seconds, None = no limit)."""
        kw = {'deadline': time.time() + budget} if budget else {}
        if self.scen is not None:
            self._update_scenarios(obs)
            acts, info = plan_robust(self.model, s0, guard, self.scen, self.threshold,
                                     self.beam, self.depth, rank_time_weight=self.rank_tw, time_weight=self.time_w,
                                     incumbent=self._incumbent,
                                     hysteresis=self.hysteresis_loaded if sum(self.faces) > 0 else self.hysteresis,
                                     p_enough=self.p_enough, **kw)
            self.debug.say('scenarios %s | P(>=%d)=%.2f E=%.0f%s' % (
                self.scen.as_dict(), self.threshold, info.get('p_ok', 0.0), info.get('e_pts', 0.0),
                ' (kept previous plan)' if info.get('kept_incumbent') else ''))
            return acts, info.get('value', 0.0)
        return plan(self.model, s0, guard, self.beam, self.depth, rank_time_weight=self.rank_tw,
                    time_weight=self.time_w, **kw)

    def _plan_starts_ok(self, s0, acts, guard):
        """Is the plan's next real step (through any leading flips) still legal from s0?"""
        x = s0
        for a in acts:
            if a[0] == 'home':
                return True
            x = apply(self.model, x, a, guard, check_scen=False)
            if x is None:
                return False
            if a[0] != 'flip':
                return True
        return False

    def _ready_plan(self, obs, s0, guard):
        """A plan that needs no search now (Max: no waiting after an action):
        (1) the one made while the last pick/drop was running, from the state predicted after it,
            if the action ended as predicted;
        (2) else the rest of the current plan, if its next step is still legal (a replan is then
            due, made later while driving)."""
        spec, self._spec = self._spec, None
        force, self._force_search = self._force_search, False
        if not obs.active or force:
            return None
        done = len(self.dropped_log) + len(self.picked_log)
        if spec is not None:
            if (spec['faces'] == (tuple(self.faces), tuple(self.vertical)) and spec['done'] == done
                    and self._plan_starts_ok(s0, spec['acts'], guard)):
                return spec['acts'], spec['val'], 'planned during the last action'
            return None                       # not as predicted: plan from what really happened
        inc = self._incumbent
        if inc and any(a[0] != 'home' for a in inc) and self._plan_starts_ok(s0, inc, guard):
            self._replan_due = True
            return list(inc), self.plan_value, 'rest of the current plan, replan due'
        return None

    def _while_busy(self, obs):
        """A pick / drop is running (robot docked, still): plan from the state the model predicts
        after it, so the next target is ready the moment it ends."""
        a = self._current_action
        if a is None or a[0] not in ('pick', 'drop', 'discard') or self._spec_for is a:
            return
        self._spec_for = a
        s0 = self._plan_state(obs)
        sp = apply(self.model, s0, a, None, check_home=False, check_scen=False)
        if sp is None:
            return
        guard = self._make_guard(obs)
        t0 = time.time()
        acts, val = self._search(obs, sp, guard, self.plan_budget)
        acts = self._order_towers(sp, list(acts), guard)
        self._spec = {'acts': list(acts), 'val': val, 'faces': (sp.faces, sp.vert),
                      'done': len(self.dropped_log) + len(self.picked_log) + 1}
        self.debug.say(f'planned ahead during {self._fmt(a)} in {(time.time() - t0) * 1000:.0f} ms: '
                       + ' '.join(self._fmt(x) for x in acts[:8]))

    def _pre_match(self, obs):
        """Before the start: make the first decision now (the first plan takes ~0.9 s, which used to
        be spent standing still at t = 0, Max), once per reset."""
        if (self._pre is None and not self._pre_done and self.pose is not None
                and obs.time_left >= self.field.match_duration - 0.5):      # a fresh match is pending
            self._pre_done = True
            out = self._decide(obs)
            self._pre = (out, tuple(self.pose[:2]))

    def _observe(self, obs):
        """The opponent docked at ANY stack (not only our target) for taken_dwell_s: count it taken.
        Its position is known all match (beacon), so a stack it emptied while we were elsewhere is no
        longer driven to and found empty (Max: P3 at 31-36 s)."""
        o = self.tracker.opp
        dt, self._dock_t = min(0.2, max(0.0, obs.t - self._dock_t)), obs.t
        if not (obs.active and o.seen_ever and o.visible and o.sigma < self.take_sigma
                and math.hypot(o.vx, o.vy) < self.take_dock_v):
            self._dock_dwell = {}
            return
        p = np.array([o.x, o.y])
        live = [s for s in self.field.stacks if not s.steal_zone and self.belief._count[s.name] >= 0.5]
        if not live:
            return
        near = min(live, key=lambda s: closest_point_on_poly(p, np.asarray(s.poly))[1])
        if closest_point_on_poly(p, np.asarray(near.poly))[1] > self.take_dock_d:
            self._dock_dwell = {}
            return
        d = self._dock_dwell.get(near.name, 0.0) + dt
        self._dock_dwell = {near.name: d}
        if d >= self.taken_dwell:
            b = self.belief
            b._count[near.name], b._last_seen[near.name], b._fresh[near.name] = 0.0, obs.t, True
            si = self.model.stack_idx.get(near.name) if hasattr(self.model, 'stack_idx') else None
            if self.scen is not None and si is not None and si not in self._taken_reported:
                self.scen.observe_taken(si, obs.t)
                self._taken_reported.add(si)
            self.debug.say(f'{near.label}: opponent docked at it {d:.1f} s, counting it taken')
            self._dock_dwell = {}

    def _decide(self, obs):
        pre, self._pre = self._pre, None
        if pre is not None and obs.active and obs.t < 1.5 and self.pose is not None \
                and math.hypot(self.pose[0] - pre[1][0], self.pose[1] - pre[1][1]) < 0.05:
            return pre[0]                    # planned before the start: go straight away
        if self._pending is not None:        # decided by the watchdog's replan a moment ago
            out, self._pending = self._pending, None
            return out
        self.blocked_timeout = self.opening_blocked_timeout if obs.t < self.opening_until else self._base_blocked_timeout
        self._forced_face = None
        self._drop_face = None
        self._plan_vertical = False
        self._flip_ahead = False
        self._lead_flip = False
        if obs.time_left < 1.0:
            return 'home', None
        # the model clock is the match clock (m.duration); keep it in sync
        t0 = time.time()
        self._log_timing(obs)
        s0 = self._plan_state(obs)
        for n in self.dropped_log[self._dropped_seen:]:
            self._ban_n.pop(n, None)              # a successful drop resets the escalation
        self._dropped_seen = len(self.dropped_log)
        m = self.model
        guard = self._make_guard(obs)
        ready = self._ready_plan(obs, s0, guard)
        if ready is not None:
            acts, val, how = ready            # no search now: the robot goes on straight away
        else:
            how = None
            acts, val = self._search(obs, s0, guard, None if not obs.active else self.plan_budget)
            acts = self._order_towers(s0, list(acts), guard)
        self._last_plan_t = obs.t
        self._plan_sig = self._obs_sig(obs)
        if ready is None:
            self._replan_due = False
        self.plan_ms = (time.time() - t0) * 1000
        self.plan_actions, self.plan_value = acts, val
        self.debug.say((f'plan {val:.1f} ({how}): ' if how else f'plan {val:.1f} in {self.plan_ms:.0f} ms: ')
                       + ' '.join(self._fmt(a) for a in acts[:12]))
        m = self.model
        self._incumbent = None
        look = self._maybe_look(obs, s0, acts, guard)
        if look is not None:
            self._incumbent = list(acts)
            self._current_action = None
            return 'look', look
        for i, a in enumerate(acts):
            if a[0] == 'flip':
                continue
            if a[0] == 'home':
                return 'home', None
            if a[0] == 'regrip':
                # floor regrip right here: the core drops, backs out and repicks with the flip face
                self._incumbent = acts[i + 1:]
                return 'regrip', a[1]
            # A flip the plan puts BEFORE this action (e.g. `flip pick:P9@1 drop:YC3@0`) must happen
            # now, while we are still where the plan flipped (inside the castle) - skipping it and
            # flipping later outside the castle is not possible.
            lead_flip = (a[2] != self.flip_face and not self.vertical[self.flip_face]
                         and sum(1 for b in acts[:i] if b[0] == 'flip') % 2 == 1)
            self._lead_flip = lead_flip
            self._flip_ahead = lead_flip or self._flip_coming(acts, i, a)
            if lead_flip and not self._flip_allowed_here():
                # not allowed to flip where we are: go to the Y5 flip spot first
                pose = m.flip_spot
                self._incumbent = [b for b in acts[:i] if b[0] != 'flip'] + list(acts[i:])
                return 'flip', pose
            if a[0] == 'pick':
                acts, a = self._swap_pick_face(acts, i, a)
            self._incumbent = acts[i + 1:]
            self._current_action = a
            self._take_acc, self._take_t = 0.0, None
            ns = apply(m, s0, a, None, check_home=False, check_scen=False) if i == 0 else None
            if ns is not None:
                self._timing = (self._fmt(a), obs.t, ns.t - s0.t)
                self._timing_n = len(self.dropped_log) + len(self.picked_log)
            if a[0] == 'pick':
                self._forced_face, self._discard = a[2], False
                return 'pick', m.stacks[a[1]]
            zone = m.zones[a[1]]
            if a[0] == 'discard':
                # over-picked stack next to the zone's built wall (its discard dock), extras
                self._discard, self._plan_vertical, self._drop_face = True, False, a[2]
                return 'drop', zone
            self._discard = False
            # vertical at the drop iff the plan flips the flip face an odd number of times first
            vert = self.vertical[self.flip_face]
            for b in acts[:i]:
                if b[0] == 'flip':
                    vert = not vert
            self._plan_vertical = zone.shape == 'circle' or (vert and a[2] == self.flip_face)
            self._drop_face = a[2]
            return 'drop', zone
        return 'home', None

    def _order_towers(self, s0, acts, guard):
        """A run of consecutive tower drops (YC...) at the head of the plan, re-sorted into the
        quickest legal order (incl. the leg to the action after the run). The search ranks plans
        by points first and keeps its previous plan within a hysteresis, so it does not reliably
        fix a 0.5 s detour like YC1 -> YC3 -> YC2 by itself."""
        m = self.model
        i0 = next((i for i, a in enumerate(acts) if a[0] != 'flip'), None)
        if i0 is None:
            return acts
        j = i0
        while j < len(acts) and acts[j][0] == 'drop' and m.zones[acts[j][1]].shape == 'circle':
            j += 1
        run = acts[i0:j]
        if len(run) < 2:
            return acts
        s = s0
        for a in acts[:i0]:
            s = apply(m, s, a, guard)
            if s is None:
                return acts
        nxt = acts[j] if j < len(acts) else None
        nxt_node = (None if nxt is None else 'home' if nxt[0] == 'home'
                    else m.stack_nodes[nxt[1]] if nxt[0] == 'pick'
                    else m.zone_nodes[nxt[1]] if nxt[0] == 'drop' else None)
        best, best_t = None, math.inf
        for perm in permutations(run):
            ss = s
            for a in perm:
                ss = apply(m, ss, a, guard)
                if ss is None:
                    break
            if ss is None:
                continue
            tt = ss.t + (m.travel.time(ss.node, nxt_node) if nxt_node else 0.0)
            if tt < best_t - 1e-6:
                best, best_t = list(perm), tt
        if best is None or best == run:
            return acts
        self.debug.say('tower order: ' + ' '.join(m.zone_label[a[1]] for a in run) + ' -> '
                       + ' '.join(m.zone_label[a[1]] for a in best))
        return acts[:i0] + best + acts[j:]

    # ------------------------------------------------ endgame time guard
    def _must_go_home(self, obs):
        """Hard check while executing: not enough time left for anything but getting home."""
        if self.target_kind in (None, 'home') or not obs.active:
            return False
        if self.stage == 'final' and obs.tool_busy:
            return False                      # a pick/drop is being executed right now: let it finish
        m = self.model
        m.travel.set_node('cur', self.pose[:2])
        need = m.tt('cur', 'home') + self.home_abort_s
        if obs.time_left >= need:
            return False
        self.debug.say(f'endgame: {obs.time_left:.1f} s left, home needs {need:.1f} s - '
                       f'dropping {getattr(self.target_obj, "label", self.target_kind)}, going home')
        self._pending = ('home', None)
        self.target_kind, self.target_obj, self.spot = None, None, None
        self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None
        return True

    def _log_timing(self, obs):
        """planned vs actual duration of the action that just ended (to re-tune plan_* overheads)."""
        if self._timing is None:
            return
        lbl, t0, planned = self._timing
        n_done = len(self.dropped_log) + len(self.picked_log)
        if n_done != self._timing_n:
            self.debug.say(f'timing {lbl}: planned {planned:.1f} s, actual {obs.t - t0:.1f} s')
        self._timing = None

    # ------------------------------------------------ observation changes
    def _obs_sig(self, obs):
        """What we have observed: stack counts (camera / pick results), bans, the opponent's position."""
        b, t = self.belief, obs.t
        counts = tuple(int(round(b._count[st.name])) for st in self.model.stacks)
        bans = frozenset(n for n, u in self._skip_until.items() if u > t)
        o = self.tracker.opp
        opp = (o.x, o.y) if (o.seen_ever and o.sigma < 1.0) else None
        return counts, bans, len(self.dropped_log), len(self.picked_log), opp

    def _obs_changed(self, obs):
        """Replan on the way only on an event (Max: less CPU): a stack count / ban / pick / drop
        changed, a replan is due (_ready_plan), we are replan_late_s behind the plan's timing for
        this action (once), or the opponent is within replan_opp_near_m of our target."""
        if self._replan_due:
            return True
        old = getattr(self, '_plan_sig', None)
        if old is None:
            return True
        if self._obs_sig(obs)[:4] != old[:4]:
            return True
        tm = self._timing
        if tm is not None and self._late_for != tm and obs.t - tm[1] > tm[2] + self.replan_late_s:
            self._late_for = tm
            return True
        o, sp = self.tracker.opp, self.spot
        if sp is not None and o.seen_ever and o.visible:
            if math.hypot(o.x - sp.pre_x, o.y - sp.pre_y) < self.replan_opp_near:
                return True
        return False

    # ------------------------------------------------------ pick watchdog
    def _opp_taking(self, st):
        """Why the opponent is taking stack `st` right now, or None. Tight on purpose: neighbouring
        stacks (0.5 m apart along an edge) must never be confused."""
        o = self.tracker.opp
        if not (o.seen_ever and o.visible and o.sigma < self.take_sigma):
            return None
        p = np.array([o.x, o.y])
        live = [s for s in self.field.stacks if not s.steal_zone
                and (s.name == st.name or self.belief._count[s.name] >= 0.5)]
        def pd(s):
            return closest_point_on_poly(p, np.asarray(s.poly))[1]
        speed = math.hypot(o.vx, o.vy)
        if speed < self.take_dock_v and pd(st) <= self.take_dock_d and min(live, key=pd).name == st.name:
            return 'docked at it'
        if speed > self.take_move_v:
            dc = math.hypot(st.x - o.x, st.y - o.y)
            if dc <= self.take_head_d:
                hd = math.atan2(o.vy, o.vx)
                cone = [s for s in live
                        if abs((math.atan2(s.y - o.y, s.x - o.x) - hd + math.pi) % (2 * math.pi) - math.pi) <= self.take_cone]
                if cone and min(cone, key=lambda s: math.hypot(s.x - o.x, s.y - o.y)).name == st.name:
                    sp = self.spot if (self.spot is not None and self.target_obj is not None
                                       and self.target_obj.name == st.name) else st
                    eta_us = math.hypot(sp.x - self.pose[0], sp.y - self.pose[1]) / max(self.vmax, 1e-3)
                    if dc / speed + self.take_eta_margin <= eta_us:
                        return 'heading there first'
        return None

    def _watch_target(self, obs):
        """Drop a pick target that is no longer worth it; replan every pick_replan_s on the way."""
        if self.target_kind == 'home':
            return self._watch_home(obs)
        if self._must_go_home(obs):
            return True
        if self.target_kind != 'pick' or self.target_obj is None or self.target_obj.name == 'floor':
            return False
        st, t, m = self.target_obj, obs.t, self.model
        si = m.stack_idx.get(st.name)
        reason, taking = None, False
        if si is not None and not st.steal_zone and m.near_opp_castle(si) and t >= m.keepout_from:
            reason = 'opponent castle is forbidden now'
        elif (self.belief._last_seen[st.name] > 0 and t - self.belief._last_seen[st.name] <= 0.3
              and self.belief._count[st.name] < 0.5):
            reason = 'camera sees it empty'
        elif self.stage in ('approach', 'verify') and not st.steal_zone:
            why = self._opp_taking(st)
            dt = 0.0 if self._take_t is None else t - self._take_t
            self._take_t = t
            self._take_acc = self._take_acc + dt if why else 0.0
            if why and self._take_acc >= self.take_debounce:
                reason, taking = 'opponent ' + why, True
                if self.scen is not None and si is not None and si not in self._taken_reported:
                    self.scen.observe_taken(si, t)
        if reason:
            self.debug.say(f'{st.label}: {reason}, changing target (banned {self.take_ban:.0f} s)')
            self._register_ban(st.name, self.take_ban, 'take' if taking else 'other', t, (st.x, st.y))
            self.target_kind, self.target_obj, self.spot = None, None, None
            self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None
            return True
        if self.stage == 'approach' and not st.steal_zone and self._n_sides('pick', st) > 1:
            # the opponent is at THIS side's pre-pose: try the stack's other approach side (no ban)
            sp = self.spot
            near = sp is not None and self._opponent_blocks(sp.pre_x, sp.pre_y)
            if not near:
                self._side_since = None
            elif self._side_since is None:
                self._side_since = t
            if near and t - self._side_since >= self.side_debounce:
                self._side_since = None
                self._side_bad[(st.name, self._spot_side)] = t + self.side_ban
                if any(not self._side_blocked('pick', st, i, sp2) for i, sp2 in enumerate(self.field.pick_spots(st, self.r_in))):
                    self.debug.say(f'{st.label}: opponent at side {self._spot_side}, trying its other side')
                    face = self.face
                    self._pending = ('pick', st)
                    self._forced_face = face
                    self.target_kind, self.target_obj, self.spot = None, None, None
                    self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None
                    return True
        if self.pick_replan_s > 0 and self.stage == 'approach' and t - self._last_plan_t >= self.pick_replan_s:
            if not self._obs_changed(obs):
                # nothing new was observed since the plan was made: keep it (the opponent model's own
                # drift is not a reason to switch target)
                self._last_plan_t = t
                return False
            cur, face = self._current_action, self.face
            self._incumbent = ([cur] if cur else []) + list(self._incumbent or [])
            self._force_search = True              # an event: a real search, not the plan we already have
            kind, obj = self._decide(obs)
            if kind == 'pick' and obj is not None and obj.name == st.name:
                # same stack: keep the face we are already turning to (Max: P8 face 0 -> face 2 made
                # it spin in place); the plan after the pick adapts to it
                self._forced_face = face
                ca = self._current_action
                if ca is not None and ca[0] == 'pick':
                    self._current_action = ('pick', ca[1], face)
                return False
            self.debug.say(f'replan on the way: {st.label} -> '
                           + (getattr(obj, 'label', None) or str(kind)))
            self._pending = (kind, obj)
            self.target_kind, self.target_obj, self.spot = None, None, None
            self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None
            return True
        return False

    # ------------------------------------------------------------- bans
    def _register_ban(self, name, dur, kind, t, xy):
        """Keep `name` out of plans for dur seconds (lifted earlier by _ban_lifted). Registered, so
        _banned() never extends it."""
        until = t + dur
        self._skip_until[name] = until
        self._known_skips[name] = until
        self._ban_info[name] = {'kind': kind, 't': t, 'xy': xy}
        self._incumbent = None

    def _opp_xy(self):
        o = self.tracker.opp
        return (o.x, o.y) if (o.seen_ever and o.visible) else None

    def _ban_lifted(self, name, info, t):
        """Early lift of a timed ban: the reason is gone."""
        oxy = self._opp_xy()
        if info['kind'] == 'take':
            seen = self.belief._last_seen[name]
            if not (seen > 0 and t - seen <= 0.3 and self.belief._count[name] >= 0.5):
                return False                                   # camera has not just seen it full
            st = self.field.stack(name)
            return oxy is None or math.hypot(oxy[0] - st.x, oxy[1] - st.y) > self.take_lift_d
        if info['kind'] == 'stuck_opp':
            if t - info['t'] < 1.0:
                return False
            return oxy is None or math.hypot(oxy[0] - info['xy'][0], oxy[1] - info['xy'][1]) > self.stuck_lift_d
        return False

    def _banned(self, t):
        """Names kept out of plans: skips still running. Our own bans are registered with a reason
        and lifted early when it is gone; the short skips core sets itself (needs boxes on a face,
        waiting for a flip, ...) run their own short time - nothing is extended."""
        for name, until in list(self._skip_until.items()):
            if until <= t:
                self._ban_info.pop(name, None)
                continue
            if self._known_skips.get(name) != until:
                self._known_skips[name] = until
                self._incumbent = None                           # the plan that led here failed
            info = self._ban_info.get(name)
            if info is not None and self._ban_lifted(name, info, t):
                self.debug.say(f'{self.field.stack(name).label if name in self.model.stack_idx else name}: ban lifted early')
                del self._skip_until[name]
                self._known_skips.pop(name, None)
                del self._ban_info[name]
        return {n for n, u in self._skip_until.items() if u > t}

    # ----------------------------------------------- approach sides / impossible
    def _n_sides(self, kind, obj):
        if kind == 'drop' and obj.label in self._dock_override:
            return 1
        return max(1, len(obj.approach))

    def _sig(self):
        """What we carry and which piles stand where: a target we could not reach with nobody near is
        impossible only while this stays the same."""
        return (tuple(self.faces), tuple(self.vertical), tuple(sorted(self.belief.zone_counts().items())),
                self._floor_pile is not None)

    def _impossible_names(self):
        sig, seen, out = self._sig(), {}, set()
        for (name, side, sg), n in self._imposs.items():
            if sg == sig:
                seen.setdefault(name, [n, set()])[1].add(side)
        for name, (n, sides) in seen.items():
            if len(sides) >= n:
                out.add(name)
        return out

    def _side_blocked(self, kind, obj, side, spot):
        if self._side_bad.get((obj.name, side), -1.0) > self._now:
            return True
        if (obj.name, side, self._sig()) in self._imposs:
            return True
        return bool(kind == 'pick' and not getattr(obj, 'steal_zone', '')
                    and self._opponent_blocks(spot.pre_x, spot.pre_y))

    def _pick_goal_blocked(self, obj, spot):
        return False        # no deferral because the opponent is near the pre-pose: the watchdog's tight rule decides

    def _opp_close_to_us(self):
        oxy = self._opp_xy()
        return oxy is not None and math.hypot(oxy[0] - self.pose[0], oxy[1] - self.pose[1]) < self.stuck_opp_d

    def _stuck_pick(self, obs):
        st, t = self.target_obj, obs.t
        side, n = self._spot_side, self._n_sides('pick', st)
        if self._opp_close_to_us():
            self._side_bad[(st.name, side)] = t + self.side_ban
            others = any(not self._side_blocked('pick', st, i, sp) for i, sp in enumerate(self.field.pick_spots(st, self.r_in)))
            if n > 1 and others:
                self.debug.say(f'{st.label}: stuck with the opponent close, trying its other side')
            else:
                self.debug.say(f'{st.label}: stuck with the opponent close, banned {self.stuck_ban[0]:.0f} s')
                self._register_ban(st.name, self.stuck_ban[0], 'stuck_opp', t, (st.x, st.y))
        else:
            self._imposs[(st.name, side, self._sig())] = n
            self.debug.say(f'{st.label}: stuck, nobody near: impossible while what we carry / the piles stay the same')

    def _stuck_drop(self, obs):
        z, t = self.target_obj, obs.t
        side, n = self._spot_side, self._n_sides('drop', z)
        if self._opp_close_to_us():
            k = self._ban_n.get(z.name, 0)
            self._ban_n[z.name] = k + 1
            dur = self.stuck_ban[min(k, len(self.stuck_ban) - 1)]
            self.debug.say(f'{z.label}: stuck with the opponent close, banned {dur:.0f} s')
            self._register_ban(z.name, dur, 'stuck_opp', t, (z.x, z.y))
            self._drop_retry_offset[z.name] = self._drop_retry_offset.get(z.name, 0) + 1
        else:
            self._imposs[(z.name, side, self._sig())] = n
            self.debug.say(f'{z.label}: stuck, nobody near: impossible while what we carry / the piles stay the same')
            if n > 1:
                self._drop_retry_offset[z.name] = self._drop_retry_offset.get(z.name, 0) + 1

    def _stuck_other(self, obs):
        if self.target_kind == 'look':
            self.debug.say('stuck on the way to the scouting viewpoint, giving up scouting for 10 s')
            self._look_fail_until = obs.t + 10.0
            self._look = None

    def _pick_failed(self, obj, message):
        # 'no stack in reach': nothing there to pick -> empty (the camera corrects it if it is wrong)
        if 'no stack in reach' in message and obj.name in self.belief._count and not obj.steal_zone:
            self.belief._count[obj.name] = 0.0
            si = self.model.stack_idx.get(obj.name)
            if si is not None:
                self._taken_reported.add(si)          # not evidence about the opponent

    # ------------------------------------------------- home replanning
    def _watch_home(self, obs):
        """Driving home or parked with time to spare: keep replanning (incumbent = stay home, with the
        hysteresis); switch as soon as a plan that does something else wins."""
        t, m = obs.t, self.model
        if self.pick_replan_s <= 0 or t - self._last_plan_t < self.pick_replan_s:
            return False
        m.travel.set_node('cur', self.pose[:2])
        if obs.time_left <= m.tt('cur', 'home') + self.home_margin:
            return False
        if not self._obs_changed(obs):
            self._last_plan_t = t                     # nothing new since the last plan: no search (CPU)
            return False
        self._incumbent = [('home',)]
        kind, obj = self._decide(obs)
        if kind == 'home':
            return False
        self.debug.say('replan from home: ' + (getattr(obj, 'label', None) or str(kind)))
        self._pending = (kind, obj)
        self.target_kind, self.target_obj, self.spot = None, None, None
        self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None
        return True

    # ------------------------------------------------- door endgame: scouting
    def _replay_points(self, s, acts, guard):
        """Points of the plan `acts` run from state s (illegal actions skipped), parked if there is time."""
        m = self.model
        for a in acts:
            if a[0] == 'home':
                continue
            ns = apply(m, s, a, guard, check_home=False, check_scen=False)
            if ns is not None and ns.t <= m.duration:
                s = ns
        h = apply(m, s, ('home',), None)
        return points(m, h) if h is not None else points(m, s, include_park=False), s

    def _scout_targets(self, obs, s0):
        """The stacks worth a look: they may hold a full stack for the door but we do not know."""
        m, t, b = self.model, obs.t, self.belief
        banned = {n for n, u in self._skip_until.items() if u > t} | self._impossible_names()
        mine = set(self.picked_log)
        pile_left = any(b._count[st.name] >= 0.5 for st in m.stacks if not st.steal_zone)
        steal_possible = any(st.steal_zone and self.field.zone(st.steal_zone).team != self.team
                             and (b._count[st.name] >= 0.5 or not (b._last_seen[st.name] > 0
                                                                   and t - b._last_seen[st.name] <= self.steal_recheck_s))
                             for st in m.stacks)
        out = []
        for i, st in enumerate(m.stacks):
            name = st.name
            if name in banned or name in mine or self._look_tried.get(name, -99.0) > t - 15.0:
                continue
            seen = b._last_seen[name]
            if st.steal_zone:
                if self.field.zone(st.steal_zone).team == self.team or pile_left:
                    continue                                    # own pile / the steal guard still forbids it
                fresh = seen > 0 and t - seen <= self.steal_recheck_s
                if not fresh:
                    out.append((i, st, 0.5))
                continue
            if b._count[name] < 0.5:
                continue
            if seen > 0 and t - seen <= self.steal_recheck_s:
                continue                                        # looked at lately
            p = self.scen.p_taken(i, t + 5.0) if self.scen is not None else 0.5
            if p < self.scout_p_lo:
                continue
            if m.near_opp_castle(i) and t >= m.keepout_from:
                # forbidden to pick now, but it still blocks stealing while believed full: a look
                # (from outside the castle) that finds it empty frees the enemy piles
                if steal_possible:
                    out.append((i, st, 0.5 * p))
                continue
            out.append((i, st, 1.0 - p))
        return out

    def _maybe_look(self, obs, s0, acts, guard):
        """Door endgame (A): the door (23 pts) is not built, there is time for it, we hold no usable
        stack and nothing is known to be full: drive to the viewpoint that sees the most uncertain
        stacks (stale belief, or enemy drop-zone piles once every P is gone), look, then replan."""
        t, m = obs.t, self.model
        if (not self.scout_on or t < self.scout_from or self._look_fail_until > t or s0.door_used or s0.door_zone != -1
                or any(self.vertical) or (acts and acts[0][0] in ('flip', 'regrip'))):
            return None
        if s0.faces[m.flip_face] == self.capacity and not s0.vert[m.flip_face]:
            return None                                         # already holding a door stack
        y5 = next((i for i, z in enumerate(m.zones) if z.shape == 'rect' and z.label.endswith('5')), None)
        if y5 is None or s0.zbox[y5] > 0 or obs.time_left < self.door_min_time + self.scout_margin + 1.0:
            return None
        b = self.belief
        for st in m.stacks:                                     # something confirmed full: the planner has it
            seen = b._last_seen[st.name]
            if (seen > 0 and t - seen <= self.scout_confirm and b._count[st.name] >= self.capacity - 0.5
                    and st.name not in self._skip_until and st.name not in self.picked_log
                    and not (st.steal_zone and self.field.zone(st.steal_zone).team == self.team)):
                return None
        if self._replay_points(s0, acts, guard)[1].door_used:
            return None                                         # the plan already builds the door
        cand = self._scout_targets(obs, s0)
        if not cand:
            return None
        o = self._opp_xy()
        vp = self.viewpoints.best(self.pose[:2], self.pose[2], [c[1] for c in cand], m.travel.vmax,
                                  self.scout_tcost, avoid=o)
        if vp is None:
            return None
        x, y, th, seen = vp
        names = [st.name for st in seen]
        m.travel.set_node('cur', self.pose[:2])
        m.travel.set_node('V:look', (x, y))
        ta = m.tt('cur', 'V:look') + 1.0
        # time: viewpoint + look + the door itself (pick, flip, Y5) + back home
        need = ta + self.door_min_time + self.scout_margin
        if obs.time_left < need or t + ta + m.tt('V:look', 'home') > m.duration:
            return None
        # what the detour costs the plan we have vs what a door would add
        base, _ = self._replay_points(s0, acts, guard)
        slip, _ = self._replay_points(replace(s0, t=s0.t + ta, node='V:look'), acts, guard)
        p_full = max(c[2] for c in cand if c[1].name in names)
        if self.scout_gain * p_full <= base - slip:
            return None
        self._look = {'x': x, 'y': y, 'th': th, 'names': names, 't0': t, 'phase': 'go', 'tw': None}
        self.debug.say('look: scouting %s from (%.2f, %.2f) heading %.0f deg (eta %.1f s, p_full %.2f, plan loses %.0f)' % (
            ','.join(self.field.stack(n).label for n in names), x, y, math.degrees(th), ta, p_full, base - slip))
        return (x, y, th)

    def _look_done(self, obs, why):
        L = self._look
        for n in (L['names'] if L else ()):
            if self.belief._last_seen[n] <= L['t0']:
                self._look_tried[n] = obs.t               # could not be seen: do not retry at once
        self.debug.say(f'look done ({why}): ' + ' '.join(
            f"{self.field.stack(n).label}={self.belief._count[n]:.0f}" for n in (L['names'] if L else ())))
        self._look = None
        self.target_kind, self.target_obj, self.spot = None, None, None
        self.stage, self.path, self.blocked_t, self._prev_xy = 'approach', [], 0.0, None
        self.v, self.wz = (0.0, 0.0), 0.0
        return Command()

    def _look_step(self, obs, dt):
        L = self._look
        if L is None:
            self.target_kind = None
            return Command()
        m = self.model
        m.travel.set_node('cur', self.pose[:2])
        if obs.time_left < m.tt('cur', 'home') + self.home_margin + self.door_min_time:
            return self._look_done(obs, 'no time left for a door')
        if all(self.belief._last_seen[n] > L['t0'] for n in L['names']):
            return self._look_done(obs, 'all seen')
        self.debug.state = 'look'
        if L['phase'] == 'go':
            arrived, vx, vy, wz = self._goto(L['x'], L['y'], L['th'], dt, pos_tol=0.04, yaw_tol=0.1)     # a real heading: the camera must point at the stacks
            self.debug.path = list(self.path)
            if not arrived:
                return Command(vx, vy, wz, goal_pose=(L['x'], L['y'], L['th']))
            L['phase'], L['tw'] = 'wait', obs.t
            self.v, self.wz = (0.0, 0.0), 0.0
        if obs.t - L['tw'] >= self.scout_wait:
            return self._look_done(obs, 'timeout')
        self.v, self.wz = (0.0, 0.0), 0.0
        return Command(0.0, 0.0, 0.0)

    def _update_scenarios(self, obs):
        opp = self.tracker.opp
        if opp.visible:
            self.scen.observe_position(obs.t, opp.x, opp.y, opp.sigma)
            ox, oy, _ = self.field.start_pose(other_team(self.team))
            self._opp_moved = max(self._opp_moved, math.hypot(opp.x - ox, opp.y - oy))
            if obs.t >= self.idle_after_s and self._opp_moved < self.idle_still_m:
                # never left its start pose: strong (untempered) evidence for 'idle'
                self.scen.observe_stationary(obs.t, opp.x, opp.y)
        for i, st in enumerate(self.model.stacks):      # camera sees it still full -> scenarios that emptied it are wrong
            if st.steal_zone:
                continue                                # the opponent never takes these: no evidence
            seen = self.belief._last_seen[st.name]
            if (seen > 0 and obs.t - seen <= 0.3 and self.belief._count[st.name] >= 0.5
                    and obs.t - self._present_t.get(i, -9.0) >= 3.0):
                self._present_t[i] = obs.t
                self.scen.observe_present(i, obs.t)
        mine = set(self.picked_log)
        for i, st in enumerate(self.model.stacks):
            if (not st.steal_zone and i not in self._taken_reported and st.name not in mine
                    and self.belief._count[st.name] < 0.5):
                self._taken_reported.add(i)
                self.scen.observe_taken(i, obs.t)

    def _fmt(self, a):
        if a[0] == 'pick':
            return f'pick:{self.model.stacks[a[1]].label}@{a[2]}'
        if a[0] == 'regrip':
            return f'regrip:{a[1]}->{self.flip_face}'
        if a[0] == 'drop':
            return f'drop:{self.model.zone_label[a[1]]}@{a[2]}'
        if a[0] == 'discard':
            return f'discard:{self.model.zone_label[a[1]]}@{a[2]}'
        if a[0] == 'flip' and len(a) > 3 and a[2] == 'at':
            return f'flip@{self.model.zone_label[a[3]]}'
        return a[0]

    def reset(self):
        """New match (/sim/reset): the core's state AND the planner brain's own per-match memories.
        Before this, a side marked impossible in the previous match ("stuck, nobody near") was still
        avoided after a reset: with nothing carried the signature matched, so the next match went
        to P6's far side (Max)."""
        super().reset()
        if not hasattr(self, 'model'):
            return                      # first call, from StrategyCore.__init__: nothing of ours yet
        self._known_skips, self._ban_info, self._ban_n = {}, {}, {}
        self._side_bad, self._imposs, self._look_tried, self._present_t = {}, {}, {}, {}
        self._open_skip, self._open_blk, self._open_t = set(), {}, 0.0
        self._open_side, self._open_switched = None, False
        self._pre, self._pre_done = None, False
        self._spec, self._spec_for, self._replan_due, self._late_for = None, None, False, None
        self._force_search = False
        self._dock_dwell, self._dock_t = {}, 0.0
        self._taken_reported = set()
        self._dropped_seen, self._timing, self._timing_n, self._opp_moved = 0, None, 0, 0.0
        self._incumbent, self._current_action, self._pending, self._plan_sig = None, None, None, None
        self._side_since, self._look, self._look_fail_until, self._last_plan_t = None, None, -1.0, -1e9
        self._forced_face, self._plan_vertical, self._flip_ahead, self._lead_flip = None, False, False, False
        self.plan_actions, self.plan_value = [], 0.0
        if self.scen is not None:
            self.scen = ScenarioSet(self.field, self.team, self._params)
            self.model.scen = self.scen

    def _flip_coming(self, acts, i, a):
        """True if the plan flips the flip face after action i without touching that face first,
        and this action itself doesn't use it: then flip it NOW, while docked / driving inside the
        castle, instead of later needing a detour back in."""
        ff = self.flip_face
        if a[2] == ff:
            return False
        for b in acts[i + 1:]:
            if b[0] == 'flip':
                return True
            if b[0] in ('home', 'regrip') or b[2] == ff:
                return False
        return False

    def _hold_for_flip(self, obs):
        """The plan flips before this action: stay put (inside the castle) until the flip is done.
        Gives up as soon as the flip is refused or done, so it can never hang."""
        ff = self.flip_face
        if not getattr(self, '_lead_flip', False) or self.faces[ff] == 0 or self.vertical[ff]:
            self._lead_flip = False
            return False
        if obs.flip_busy:
            return True
        if obs.t < self._flip_block_until or not self._flip_allowed_here():
            self._lead_flip = False          # refused / not possible here: go on, the flip detour handles it
            return False
        return True

    def _expects_vertical(self, n):
        t = self.target_obj
        return bool(getattr(self, '_plan_vertical', False)) and t is not None and t.shape == 'rect'

    def _wants_vertical(self):
        # flip a loaded flip-face only for the circle / door drop the plan wants next
        return (bool(getattr(self, '_plan_vertical', False)) or self._flip_ahead) and self.faces[self.flip_face] > 0
