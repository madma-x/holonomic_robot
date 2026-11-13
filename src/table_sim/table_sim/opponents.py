"""Scripted opponents for the simulator.

They read the ground truth from the World (they are test fixtures, not a
strategy) and drive the 'opp' robot through the same command / tool API as
our robot. Behaviours:

  mirror        own half first, closest stacks first (a sane opponent)
  greedy        always the closest non-empty stack
  raider        empties OUR half first, then the centre, then its own
  center_first  the two centre stacks first, then its own half
  random        random stack order (seeded)
  blocker       never picks: patrols in front of our drop zones
  idle          stays in its start zone
  teleop        does nothing: drive it from /opponent/cmd_vel
"""

import math
import random

from .field import other_team
from .geometry import wrap
from .gridplan import GridPlanner
from .world import DROP, PICK

BEHAVIORS = ('mirror', 'greedy', 'raider', 'center_first', 'random',
             'blocker', 'idle', 'teleop')


class ScriptedOpponent:
    def __init__(self, world, behavior='mirror', avoid_us=True, speed_scale=1.0,
                 seed=0, name='opp'):
        if behavior not in BEHAVIORS:
            raise ValueError(f'unknown opponent behavior {behavior!r}, use one of {BEHAVIORS}')
        self.w = world
        self.name = name
        self.behavior = behavior
        self.avoid = avoid_us
        self.rng = random.Random(seed)
        self.r = world.robots[name]
        self.team = self.r.team
        self.enemy = 'us' if name == 'opp' else 'opp'
        f = world.field
        self.planner = GridPlanner(f.length, f.width, res=0.04)
        self.vmax = self.r.model.vmax * speed_scale
        self.amax = self.r.model.amax
        self.task = None
        self.stage = 'approach'
        self.path = []
        self.blocked_t = 0.0
        self.skip = {}
        self.order = self._initial_order()
        self.patrol_i = 0
        self.retreat = None
        self.status = 'waiting'

    # ------------------------------------------------------------ ordering
    def _initial_order(self):
        f = self.w.field
        sx, sy, _ = f.start_pose(self.team)
        stacks = [s for s in f.stacks if not s.steal_zone]
        dist = {s.name: math.hypot(s.x - sx, s.y - sy) for s in stacks}
        if self.behavior == 'mirror':
            return sorted(stacks, key=lambda s: (s.side != self.team, dist[s.name]))
        if self.behavior == 'raider':
            ours = other_team(self.team)
            ex, ey, _ = f.start_pose(ours)
            return sorted(stacks, key=lambda s: ({ours: 0, 'center': 1}.get(s.side, 2),
                                                 math.hypot(s.x - ex, s.y - ey)))
        if self.behavior == 'center_first':
            return sorted(stacks, key=lambda s: ({'center': 0, self.team: 1}.get(s.side, 2),
                                                 dist[s.name]))
        if self.behavior == 'random':
            stacks = stacks[:]
            self.rng.shuffle(stacks)
            return stacks
        return stacks

    # ---------------------------------------------------------- helpers
    def _static(self):
        w = self.w
        names = tuple(sorted(n for n, c in w.stacks.items() if c))
        piles = tuple(sorted(n for n, c in w.zones.items() if c))
        polys = [w.stack_polys[n] for n in names] + [w.pile_polys[n] for n in piles]
        self.planner.set_static(names + piles, polys, self.r.model.r_out + 0.02)

    def _eta(self, x, y):
        return math.hypot(x - self.r.x, y - self.r.y) / max(self.vmax * 0.6, 0.1)

    def _best_face(self, face_heading, want):
        best, best_d = None, 9.0
        for f, a in enumerate(self.r.model.face_angles):
            n = self.r.faces[f]
            if want == 'free' and n >= self.r.model.capacity:
                continue
            if want == 'loaded' and n == 0:
                continue
            d = abs(wrap(face_heading - a - self.r.th))
            if want == 'free':
                d += 0.5 * (n == 0)        # top up a partly loaded face first
            if want == 'loaded':
                d -= 0.5 * n               # unload the fullest face first
            if d < best_d:
                best, best_d = f, d
        return best

    def _goto(self, x, y, th, dt, plan=True):
        """Follow a planned path. Returns True when at the pose."""
        r = self.r
        d = math.hypot(x - r.x, y - r.y)
        eth = wrap(th - r.th)
        if d < 0.015 and abs(eth) < 0.04:
            self.w.set_cmd(self.name, 0, 0, 0)
            return True
        if not plan:
            self.path = [(x, y)]
        elif not self.path or self.path[-1] != (x, y):
            self._static()
            wps, _ = self.planner.plan((r.x, r.y), (x, y))
            self.path = wps if wps else [(x, y)]
        while len(self.path) > 1 and math.hypot(self.path[0][0] - r.x, self.path[0][1] - r.y) < 0.12:
            self.path.pop(0)
        tx, ty = self.path[0]
        dx, dy = tx - r.x, ty - r.y
        dd = math.hypot(dx, dy)
        v = min(self.vmax, math.sqrt(2 * self.amax * 0.8 * d), 3.0 * d + 0.02)
        vx, vy = (dx / dd * v, dy / dd * v) if dd > 1e-6 else (0.0, 0.0)
        e = self.w.robots[self.enemy]
        ex, ey = e.x - r.x, e.y - r.y
        de = math.hypot(ex, ey)
        actual = math.hypot(r.vx, r.vy)
        if self.avoid and de < 0.6 and (vx * ex + vy * ey) > 0.3 * v * de:
            vx = vy = 0.0
            self.blocked_t += dt
        elif v > 0.1 and actual < 0.03:      # pushing against something
            self.blocked_t += dt
        else:
            self.blocked_t = 0.0
        c, s = math.cos(r.th), math.sin(r.th)
        self.w.set_cmd(self.name, c * vx + s * vy, -s * vx + c * vy,
                       max(-3.0, min(3.0, 4.0 * eth)))
        return False

    # ------------------------------------------------------------- tasks
    def _next_pick(self):
        w, f = self.w, self.w.field
        now = w.match_time
        cands = [s for s in self.order if w.stacks[s.name] and self.skip.get(s.name, -1) < now]
        if self.behavior == 'greedy':
            cands.sort(key=lambda s: math.hypot(s.x - self.r.x, s.y - self.r.y))
        for s in cands:
            spots = f.pick_spots(s, self.r.model.r_in)
            sp = min(spots, key=lambda p: math.hypot(p.x - self.r.x, p.y - self.r.y))
            face = self._best_face(sp.face_heading, 'free')
            if face is None:
                return None
            return ('pick', s.name, sp, face)
        return None

    def _next_drop(self):
        w, f = self.w, self.w.field
        cands = []
        for z in f.team_zones(self.team):
            if z.vertical_only:
                continue   # scripted opponents never flip, so can't use these
            if w.zones[z.name] >= z.capacity:
                continue
            for sp in f.drop_spots(z, self.r.model.r_in):
                cands.append((math.hypot(sp.x - self.r.x, sp.y - self.r.y), z.name, sp))
        for _, zname, sp in sorted(cands, key=lambda c: c[0]):
            face = self._best_face(sp.face_heading, 'loaded')
            if face is not None:
                return ('drop', zname, sp, face)
        return None

    def _home_task(self):
        x, y, _ = self.w.field.start_pose(self.team)
        return ('home', None, (x, y), None)

    def _decide(self):
        if self.behavior == 'idle':
            return self._home_task()
        r = self.r
        hx, hy, _ = self.w.field.start_pose(self.team)
        t_home = self._eta(hx, hy) + 4.0
        if self.w.time_left < t_home:
            return self._home_task()
        if r.carried() and self.w.time_left < t_home + 6.0:
            return self._next_drop() or self._home_task()
        if self.behavior == 'blocker':
            return ('patrol', None, None, None)
        full_faces = sum(1 for n in r.faces if n >= r.model.capacity)
        if full_faces == len(r.faces):
            return self._next_drop() or self._home_task()
        t = self._next_pick()
        if t is None:
            return self._next_drop() if r.carried() else self._home_task()
        return t

    # ---------------------------------------------------------------- step
    def step(self, dt):
        w = self.w
        if not w.running or self.behavior == 'teleop':
            return
        r = self.r
        res = w.pop_result(self.name)
        if res is not None and self.task:
            kind = self.task[0]
            if kind == 'pick' and not res.success:
                self.skip[self.task[1]] = w.match_time + 1e9
            if kind == 'drop' and res.success:
                hd = self.task[2].face_heading
                self.retreat = (r.x - 0.12 * math.cos(hd), r.y - 0.12 * math.sin(hd), r.th)
            self.task = None
        if r.action is not None:
            self.w.set_cmd(self.name, 0, 0, 0)
            self.status = r.action.phase
            return
        if self.retreat is not None:
            if self._goto(self.retreat[0], self.retreat[1], self.retreat[2], dt, plan=False) \
                    or self.blocked_t > 1.0:
                self.retreat, self.path = None, []
            return
        if self.task is None or (self.task[0] in ('pick', 'patrol') and w.time_left < 15.0):
            new = self._decide()
            if new != self.task:
                self.path = []
                self.stage = 'approach'
            self.task = new
        if self.task is None:
            self.w.set_cmd(self.name, 0, 0, 0)
            return
        kind, key, target, face = self.task
        self.status = f'{kind} {self.w.field.label(key) if key else ""}'.strip()
        if kind == 'home':
            self._goto(target[0], target[1], r.th, dt)
            return
        if kind == 'patrol':
            ours = other_team(self.team)
            zs = w.field.team_zones(ours)
            sx, _, _ = w.field.start_pose(ours)
            pts = [(z.x + (0.4 if sx < 1.5 else -0.4), z.y) for z in zs]
            px, py = pts[self.patrol_i % len(pts)]
            if self._goto(px, py, r.th, dt) or self.blocked_t > 1.5:
                self.patrol_i += 1
                self.path = []
            return
        heading = target.face_heading - r.model.face_angles[face]
        if self.stage == 'approach':
            if self._goto(target.pre_x, target.pre_y, heading, dt):
                self.stage = 'final'
                self.path = []
            elif self.blocked_t > 2.5:
                if kind == 'pick':
                    self.skip[key] = w.match_time + 6.0
                self.task, self.path, self.blocked_t = None, [], 0.0
            return
        if self._goto(target.x, target.y, heading, dt, plan=False):
            ok, _ = w.request_action(self.name, face, PICK if kind == 'pick' else DROP)
            if not ok:
                self.task = None
        elif self.blocked_t > 2.5:
            self.task, self.path, self.blocked_t = None, [], 0.0


def make_opponent(world, cfg):
    o = dict(cfg or {})
    return ScriptedOpponent(world, behavior=o.get('behavior', 'mirror'),
                            avoid_us=bool(o.get('avoid_us', True)),
                            speed_scale=float(o.get('speed_scale', 1.0)),
                            seed=int(o.get('seed', 0)))
