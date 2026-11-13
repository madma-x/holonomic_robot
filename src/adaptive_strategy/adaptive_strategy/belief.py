"""Our own belief about pick-stack occupancy and drop-zone fill state.

Never reads ground truth. Three layered signals, most recent direct
observation wins, otherwise a time-decayed prior:
  1. camera (ArUco) sighting of a stack - exact true count, highest trust.
  2. our own PICK result at a stack we were standing at - ground truth at
     that moment, for that stack only.
  3. absent either of the above: a decaying prior starting from the known
     full capacity (a public game rule, not a cheat), decaying faster for
     stacks nearer the opponent's side (they're statistically more likely to
     have gone there first) and slower for our own side - this is what makes
     "steal enemy stacks when there's value in it, play safe late" fall out
     of a single utility score instead of a hardcoded match-phase rule.

Drop zones are different: we filled them ourselves, so our own fill count is
ground truth, not belief - just bookkeeping from our own DROP results. The
opponent's own zones are the opposite: we have zero ground truth there, only
whatever the camera happens to see in passing (e.g. heading for P9/P10 can
reveal BC1 or B1-B5 nearby) - that's enough to avoid driving into a box
they've already dropped (collision, and a disqualification risk), without
needing to understand their fill strategy.
"""

import math


class StackBelief:
    @staticmethod
    def _resolve_labels(field, labels):
        """Global P<n>/Y<n>/B<n> labels -> Stack objects, in the given
        order, dropping any label that doesn't resolve - small shared
        helper so pick_sequence/contested_hi_group/contested_lo_group/
        drop_sequence's `pick:` all parse the same way."""
        out = []
        for label in labels or []:
            stk = next((s for s in field.stacks if s.label == label), None)
            if stk is not None:
                out.append(stk)
        return out

    def __init__(self, field, team, params=None):
        self.field = field
        self.team = team
        p = dict(params or {})
        self.decay_own = float(p.get('decay_own_per_s', 0.002))
        self.decay_center = float(p.get('decay_center_per_s', 0.01))
        self.decay_enemy = float(p.get('decay_enemy_per_s', 0.02))
        self.box_points = float(field.scoring.get('box_in_zone', 3))
        self.handling_time = float(p.get('handling_time_s', 3.0))
        self.travel_speed = float(p.get('travel_speed_mps', 0.5))

        # Opponent-reactive stack choice (B2) is parked for now - Max wants
        # a single fixed, explicit pick order instead (see pick_sequence/
        # next_pick below), opponent toggled off while this is tuned. Left
        # in place (unused) rather than deleted in case group-reactive
        # picking comes back later.
        self._contested = {s.name for s in field.stacks if s.side != self.team}
        self.contested_penalty = float(p.get('contested_penalty', 0.7))
        self.contested_lookahead_s = float(p.get('contested_lookahead_s', 1.5))
        self.contested_radius_m = float(p.get('contested_radius_m', 0.8))
        self.contested_confidence_scale_m = float(p.get('contested_confidence_scale_m', 2.0))

        # Fixed pick order (Max): global P<n> labels, tried strictly in this
        # order - the next one still believed non-empty, never scored/
        # reordered by distance. Resolved to actual Stack objects once here.
        self._pick_order = self._resolve_labels(field, p.get('pick_sequence'))

        # Semi-modular exception to the strict order above (Max): P7/P9 are
        # the enemy-side "hi" stacks - if the opponent currently looks to be
        # headed for either one, swap the due hi pick for whichever of P6/
        # P8/P10 (the "lo" alternatives) still has stock left, instead of
        # either ploughing into the opponent or sitting on the hi stack
        # until it clears. No threat there -> pick P7/P9 as normal and
        # leave P6/P8/P10 for their own turn later in pick_sequence.
        self._contested_hi = self._resolve_labels(field, p.get('contested_hi_group'))
        self._contested_lo = self._resolve_labels(field, p.get('contested_lo_group'))
        self._contested_hi_names = {s.name for s in self._contested_hi}

        # Fill order Max set in the table-reference artifact (state.json's
        # `assignments`): {kind: rect|circle, index: own-team zone number
        # (Y3/B3 -> 3, YC2/BC2 -> 2), type: H|V|D}. Keyed by (shape, index)
        # so it applies to whichever team this belief is for. `type` isn't
        # otherwise used - V vs H already falls out of zone.shape (circle vs
        # rect), and D (Y5's door) isn't implemented yet so it's just
        # treated as a plain horizontal rect drop like any other H zone.
        self._drop_order = {}
        # Zone labels (own-team, e.g. YC4) never to target at all - excluded
        # from best_drop's candidates entirely, not just deprioritized.
        self._excluded_zones = set(p.get('excluded_drop_zones') or [])
        # Optional `pick: P<n>` on an entry pins the stack (global P1..P10
        # label, shared across both teams) that should feed that zone - Max:
        # P1 before YC1, P2 before YC4. Resolved to the actual stack once
        # here rather than re-searching field.stacks on every query.
        self._pick_for = {}
        for entry in p.get('drop_sequence') or []:
            shape = 'circle' if entry['kind'] == 'circle' else 'rect'
            key = (shape, int(entry['index']))
            self._drop_order[key] = int(entry['order'])
            label = entry.get('pick')
            if label:
                stk = next((s for s in field.stacks if s.label == label), None)
                if stk is not None:
                    self._pick_for[key] = stk
        self._pinned_stacks = {s.name for s in self._pick_for.values()}

        cap = float(field.boxes_per_stack)
        self._count = {s.name: (0.0 if s.steal_zone else cap) for s in field.stacks}   # enemy piles: camera only
        self._last_seen = {s.name: 0.0 for s in field.stacks}
        self._fresh = {s.name: False for s in field.stacks}   # True until the prior starts decaying

        self._zone_count = {z.name: 0 for z in field.zones if z.team == team}
        self._steal_of = {s.steal_zone: s.name for s in field.stacks if s.steal_zone}
        # Opponent's own zones: camera-only, no prior (no decay - unlike
        # stacks, a zone's box count only ever goes up, and we have no basis
        # to guess when/whether that happens without having seen it).
        self._enemy_zone = {z.name: 0 for z in field.zones if z.team != team}
        # Real pose (x, y, th) of where our own first drop into a zone
        # actually landed - self-knowledge (we know our own pose and which
        # face we used), not ground truth about the opponent. Used for the
        # costmap obstacle instead of the zone's idealized preset centre,
        # so Nav2 avoids where the pile really is, not just "somewhere in
        # this 0.4x0.2m zone" - same reasoning as table_sim/world.py's own
        # pile_pose for rendering/collision.
        self._zone_pose = {}

    def _opponent_near(self, stack, opp):
        """True if the opponent currently looks to be headed for `stack`'s
        spot - same projected-position/confidence shape as pick_score's
        contested discount, but a hard gate here instead of a soft
        multiplier: next_pick needs a yes/no to decide whether to swap
        groups, not a score to blend in."""
        if opp is None or not opp.seen_ever:
            return False
        tx = opp.x + opp.vx * self.contested_lookahead_s
        ty = opp.y + opp.vy * self.contested_lookahead_s
        d = math.hypot(stack.x - tx, stack.y - ty)
        confidence = max(0.0, 1.0 - opp.sigma / self.contested_confidence_scale_m)
        return d < self.contested_radius_m and confidence > 0.3

    def next_pick(self, exclude=(), opp=None):
        """The next stack in the fixed pick_sequence that's still believed
        non-empty (ground truth from our own pick results/camera, not a
        decayed guess here - believed_count(t) handles that) and not in
        `exclude` (temporarily skipped, e.g. opponent camped on it). Strict
        order, with one semi-modular exception: if the due stack is one of
        contested_hi_group (P7/P9) and the opponent currently looks headed
        for either of them, swap to whichever contested_lo_group stack
        (P6/P8/P10, in that configured order) is still believed non-empty,
        instead of either contesting the opponent there or sitting on P7/P9
        until it clears. No threat on the hi side -> just pick the due hi
        stack as normal, leaving the lo alternatives for their own turn
        later in pick_sequence."""
        for s in self._pick_order:
            if s.name in exclude:
                continue
            if self._count[s.name] < 0.5:
                continue
            if s.name in self._contested_hi_names and any(
                    self._opponent_near(h, opp) for h in self._contested_hi):
                alt = next((a for a in self._contested_lo
                           if a.name not in exclude and self._count[a.name] >= 0.5), None)
                if alt is not None:
                    return alt
            return s
        return None

    # ------------------------------------------------------------- updates
    def on_camera(self, cam, t):
        """cam: {name: true_count} from Observation.cam, or None."""
        if not cam:
            return
        for name, count in cam.items():
            if name in self._count:
                self._count[name] = float(count)
                self._last_seen[name] = t
                self._fresh[name] = True
            elif name in self._enemy_zone:
                self._enemy_zone[name] = int(count)

    def on_pick_result(self, stack_name, success, count, t):
        if stack_name not in self._count:
            return
        if not success and stack_name in self._steal_of.values():
            self._count[stack_name], self._last_seen[stack_name], self._fresh[stack_name] = 0.0, t, True   # nothing there
            return
        if not success:
            # Not in reach etc - no information about the stack's actual
            # contents, so don't touch the belief (previously this zeroed
            # it out, wrongly marking a merely-misaligned stack as empty).
            return
        # count == 0 on a successful (in-reach) pick is itself ground
        # truth that the stack is empty NOW - previously this only ever
        # subtracted (count - 0 = no change), so a stale belief (still
        # showing full) was never corrected and next_pick kept retrying
        # the same already-empty stack forever.
        self._count[stack_name] = 0.0 if count == 0 else max(0.0, self._count[stack_name] - count)
        self._last_seen[stack_name] = t
        self._fresh[stack_name] = True

    def on_drop_result(self, zone_name, success, count, pose=None):
        if success and zone_name in self._zone_count:
            self._zone_count[zone_name] += count
            st = self._steal_of.get(zone_name)
            if st is not None:                    # our own pile is a pickable 'stack' (drop-and-repick)
                self._count[st] = float(self._zone_count[zone_name])
                self._fresh[st] = True
                self._last_seen[st] = 0.0
            if pose is not None and zone_name not in self._zone_pose:
                self._zone_pose[zone_name] = pose

    def on_zone_emptied(self, zone_name, count):
        """We picked our own pile back up (drop-and-repick)."""
        if zone_name in self._zone_count:
            self._zone_count[zone_name] = max(0, self._zone_count[zone_name] - count)
            st = self._steal_of.get(zone_name)
            if st is not None:
                self._count[st] = float(self._zone_count[zone_name])
            if self._zone_count[zone_name] == 0:
                self._zone_pose.pop(zone_name, None)

    def zone_pose(self, zone_name):
        return self._zone_pose.get(zone_name)

    # --------------------------------------------------------------- query
    def _decay_rate(self, side):
        if side == 'center':
            return self.decay_center
        return self.decay_own if side == self.team else self.decay_enemy

    def confidence(self, stack, t):
        """How much to trust the current believed_count, in [0, 1] - 1.0
        right after a direct (camera or pick-result) observation, decaying
        the longer it's been since. Used to decide when a pick is "risky"
        enough to warrant a camera check before committing to it."""
        dt = t if not self._fresh[stack.name] else max(0.0, t - self._last_seen[stack.name])
        return math.exp(-self._decay_rate(stack.side) * dt)

    def believed_count(self, stack, t):
        raw = self._count[stack.name]
        if raw <= 0:
            return 0.0
        return raw * self.confidence(stack, t)

    def pick_score(self, stack, from_xy, t, opp=None):
        believed = self.believed_count(stack, t)
        if believed < 0.5:
            return 0.0
        d = math.hypot(stack.x - from_xy[0], stack.y - from_xy[1])
        travel = d / max(self.travel_speed, 0.05)
        score = (believed * self.box_points) / (travel + self.handling_time)
        if opp is not None and opp.seen_ever and stack.name in self._contested:
            # Apparent target: current position projected forward along the
            # tracked velocity. Discount proportional to how close THIS
            # stack is to that point - a credible threat only if the
            # opponent looks to be genuinely arriving here, not just
            # somewhere on the same half of the table (see the comment on
            # contested_radius_m in __init__ for why that distinction
            # matters).
            tx = opp.x + opp.vx * self.contested_lookahead_s
            ty = opp.y + opp.vy * self.contested_lookahead_s
            d_target = math.hypot(stack.x - tx, stack.y - ty)
            proximity = max(0.0, 1.0 - d_target / self.contested_radius_m)
            if proximity > 0.0:
                confidence = max(0.0, 1.0 - opp.sigma / self.contested_confidence_scale_m)
                score *= 1.0 - self.contested_penalty * proximity * confidence
        return score

    def best_pick(self, from_xy, t, exclude=(), opp=None):
        best, best_score = None, 0.0
        for s in self.field.stacks:
            if s.name in exclude or s.steal_zone:
                continue
            sc = self.pick_score(s, from_xy, t, opp=opp)
            if sc > best_score:
                best, best_score = s, sc
        return best, best_score

    def required_pick(self, zone):
        """The stack pinned (via drop_sequence's `pick:`) to feed `zone`,
        or None if it isn't pinned to one."""
        idx = int(''.join(c for c in zone.label if c.isdigit()) or 0)
        return self._pick_for.get((zone.shape, idx))

    def is_pinned(self, stack):
        """True if `stack` is the designated feed for some circle zone."""
        return stack.name in self._pinned_stacks

    def is_excluded(self, zone):
        """True if `zone` is in excluded_drop_zones - never a drop target."""
        return zone.label in self._excluded_zones

    def due_order(self, zone):
        """drop_sequence order of the due `zone` (see best_drop's sort_key) -
        exposed so core.py can tell whether it's still mid a strictly-
        sequential pair (Y1 -> Y3) that must finish before picking again."""
        idx = int(''.join(c for c in zone.label if c.isdigit()) or 0)
        return self._drop_order.get((zone.shape, idx), 1000 + idx)

    def first_interleave_order(self):
        """Lowest drop_sequence order among circle-zone entries - Max's
        choreography only interleaves picking back in once the due drop
        reaches the first circle zone (P1/P3 -> YC1), never mid-way through
        the earlier strictly-sequential rect zones (Y1 -> Y3 must finish
        first). No circle zones configured: nothing blocks interleaving."""
        circle_orders = [order for (shape, _idx), order in self._drop_order.items() if shape == 'circle']
        return min(circle_orders) if circle_orders else 0

    def best_drop(self, exclude=(), include_vertical=True):
        """The DUE zone: lowest-order not-yet-full zone of our own team, in
        `drop_sequence` order if one was given (Max's artifact-edited fill
        order), else lowest-numbered first - field.py's own Y1..Y5/B1..B5
        labels are already sorted farthest-from-center first, which is also
        nearest our own start/end zone, so that fallback still keeps later
        zones reachable instead of boxing ourselves in. `include_vertical=
        False` restricts to rect (H) zones only - circle (V) zones need the
        flip mechanism, tested separately.

        Strict: always returns the due zone, even if it was just
        stuck-abandoned. `exclude` is accepted for call-site compatibility
        but no longer skips past it to a later zone - Max: a temporarily
        blocked zone should be waited/retried, not skipped, which is how
        YC1 got dropped before Y3 was ever attempted. The stuck-abandon
        retry in core.py still rotates to a different curated approach side
        each time (_drop_retry_offset), so a retry isn't just banging on the
        exact same blocked spot. Open question for Max: should a zone
        blocked long-term (e.g. opponent camped on it) eventually give way
        to a later one, and after how long? Until answered, order always
        wins - caller falls back to 'wait' when the due zone can't be acted
        on right now (see core.py's _decide)."""
        zs = [z for z in self.field.team_zones(self.team)
              if z.label not in self._excluded_zones
              and (z.shape == 'rect' or (include_vertical and z.shape == 'circle'))]

        def sort_key(z):
            idx = int(''.join(c for c in z.label if c.isdigit()) or 0)
            return self._drop_order.get((z.shape, idx), 1000 + idx)

        zs.sort(key=sort_key)
        for z in zs:
            # Circle zones physically hold up to 3 now (world.py - a DROP can
            # place 1, 2 or 3 at once), but the current choreography only
            # ever wants ONE box per circle before moving to the next one in
            # order - so "due" for a circle means "still empty", not "still
            # under capacity", or this would keep re-targeting e.g. YC1
            # instead of advancing to YC2 after its first (default) box.
            target = z.capacity if z.shape == 'rect' else 1
            if self._zone_count.get(z.name, 0) < target:
                return z
        return None

    def zone_counts(self):
        return dict(self._zone_count)

    def enemy_occupied_zones(self):
        """Opponent zones we've actually seen boxes in via camera - treated
        as obstacles (collision/disqualification avoidance), not targets."""
        return [z for z in self.field.zones
                if z.name in self._enemy_zone and self._enemy_zone[z.name] > 0]
