"""Opponent scenarios for the planner (no ROS, no ground truth).

A scenario = a probability + the order the opponent is expected to empty the
stacks in. From the public field layout and a simple timing model we derive,
per scenario, `take[si]` = the time stack si is emptied (inf = never) and a
piecewise path of where the opponent is at time t. Probabilities are updated
from what the brain actually observes (lidar sightings, stacks seen emptied).
"""
import math

from table_sim.field import other_team

DEFAULTS = {          # name: prior probability
    'defensive': 0.40,   # own half first, nearest first          (~ mirror)
    'aggressive': 0.15,  # our half first, then centre, then its own (~ raider)
    'adaptive': 0.27,    # centre first, then its own half          (~ center_first / greedy)
    'blocker': 0.10,     # never picks, parks/patrols in front of our drop zones
    'idle': 0.08,        # never leaves its zone
}


class Scenario:
    def __init__(self, name, prob, take, path):
        self.name, self.prob, self.take, self.path = name, prob, take, path   # path: [(t, x, y)]

    def pos_at(self, t):
        p = self.path
        if t <= p[0][0]:
            return p[0][1:]
        for (t0, x0, y0), (t1, x1, y1) in zip(p, p[1:]):
            if t <= t1:
                a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
                return x0 + a * (x1 - x0), y0 + a * (y1 - y0)
        return p[-1][1:]


def _order(name, field, team, opp):
    stacks = [s for s in field.stacks if not s.steal_zone]    # the opponent never empties drop-zone piles here
    ox, oy, _ = field.start_pose(opp)
    ux, uy, _ = field.start_pose(team)
    d = lambda s, x, y: math.hypot(s.x - x, s.y - y)          # noqa: E731
    if name == 'defensive':
        key = lambda s: (s.side != opp, d(s, ox, oy))         # noqa: E731
    elif name == 'aggressive':
        key = lambda s: ({team: 0, 'center': 1}.get(s.side, 2), d(s, ux, uy))   # noqa: E731
    elif name == 'adaptive':
        key = lambda s: ({'center': 0, opp: 1}.get(s.side, 2), d(s, ox, oy))    # noqa: E731
    else:
        return []
    idx = {s.name: i for i, s in enumerate(field.stacks)}
    return [idx[s.name] for s in sorted(stacks, key=key)]


class ScenarioSet:
    def __init__(self, field, team, params=None, speed=0.8, leg_overhead=1.0, engage=0.4, lift=0.6,
                 drop_cycle=6.5, faces=3):
        # Timing calibrated on the scripted opponents (mirror: P9 2.9 s, P10 7.1 s, P8 10.3 s, next
        # drops ~14 s, P7 ~17.5-19 s): each leg = distance / speed + a fixed docking overhead, the
        # stack is emptied `engage` after arrival, then `lift` before leaving; every `faces`
        # picks a drop trip costs `drop_cycle`.
        p = dict(params or {})
        opp = other_team(team)
        probs = dict(DEFAULTS)
        probs.update(p.get('opponent_scenarios') or {})
        tot = sum(probs.values()) or 1.0
        self.field, self.floor = field, float(p.get('scenario_floor', 0.02))
        self.pos_power = float(p.get('scenario_pos_power', 0.35))     # tempering of sighting evidence
        self.taken_power = float(p.get('scenario_taken_power', 0.7))  # ... of 'stack seen emptied'
        ox, oy, _ = field.start_pose(opp)
        self.scenarios = []
        n = len(field.stacks)
        zs = field.team_zones(team)
        for name, pr in probs.items():
            take, path = [math.inf] * n, [(0.0, ox, oy)]
            x, y, t = ox, oy, 0.0
            if name == 'blocker' and zs:
                # parks in front of our drop zones (between them and the table centre)
                bx = sum(z.x for z in zs) / len(zs) + (0.5 if team == 'yellow' else -0.5)
                by = sum(z.y for z in zs) / len(zs)
                path.append((math.hypot(bx - ox, by - oy) / speed, bx, by))
            for k, si in enumerate(_order(name, field, team, opp)):
                s = field.stacks[si]
                t += math.hypot(s.x - x, s.y - y) / speed + leg_overhead
                x, y = s.x, s.y
                path.append((t, x, y))
                t += engage
                take[si] = t
                t += lift
                path.append((t, x, y))
                if (k + 1) % faces == 0:           # full: drop trip back home and out again
                    t += drop_cycle
                    path.append((t, x, y))
            self.scenarios.append(Scenario(name, pr / tot, take, path))
        self._taken_seen = set()

    # ------------------------------------------------------------ queries
    def p_taken(self, si, t):
        return sum(s.prob for s in self.scenarios if s.take[si] <= t)

    # ------------------------------------------------------------ updates
    def _reweight(self, like, power=1.0):
        """Bayes update with the likelihood raised to `power` (<1 = tempered: one noisy
        observation can't flip the belief)."""
        for s, l in zip(self.scenarios, like):
            s.prob *= max(l, 1e-3) ** power
        tot = sum(s.prob for s in self.scenarios)
        for s in self.scenarios:
            s.prob = max(self.floor, s.prob / tot)
        tot = sum(s.prob for s in self.scenarios)
        for s in self.scenarios:
            s.prob /= tot

    def observe_position(self, t, x, y, sigma):
        """Opponent seen at (x, y) at time t: favour scenarios that predicted it there."""
        sd = 0.6 + sigma
        self._reweight([math.exp(-0.5 * (math.dist(s.pos_at(t), (x, y)) / sd) ** 2)
                        for s in self.scenarios], power=self.pos_power)

    def observe_taken(self, si, t):
        """Stack si seen emptied (not by us) around time t."""
        if si in self._taken_seen:
            return
        self._taken_seen.add(si)
        like = []
        for s in self.scenarios:
            tt = s.take[si]
            like.append(0.05 if math.isinf(tt) else math.exp(-0.5 * ((t - tt) / 8.0) ** 2) + 0.05)
        self._reweight(like, power=self.taken_power)

    def observe_stationary(self, t, x, y, sd=0.3):
        """The opponent has not left its start pose: untempered, narrow position evidence - every
        scenario that has it elsewhere by now loses fast, 'idle' (and anything still at the start) wins."""
        self._reweight([math.exp(-0.5 * (math.dist(s.pos_at(t), (x, y)) / sd) ** 2)
                        for s in self.scenarios], power=1.0)

    def observe_present(self, si, t):
        """Stack si seen still full at time t: scenarios that had it emptied well before t are wrong."""
        self._reweight([0.25 if s.take[si] < t - 1.0 else 1.0 for s in self.scenarios], power=1.0)

    def as_dict(self):
        return {s.name: round(s.prob, 3) for s in self.scenarios}
