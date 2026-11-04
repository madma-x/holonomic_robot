"""2D match renderer: draws one frame dict onto a matplotlib axes.

Extracted from headless.py so the same drawing code serves both producers of
frames:
  * headless.py        - frames straight out of a simulated match
  * replay_match.py    - frames rebuilt from a recorded rosbag (bag2frames)

Deliberately knows nothing about World or ROS: everything it needs is a
Field, the two RobotModels, and a plain frame dict, all of which can be
rebuilt from field_2027.yaml + sim.yaml. That is what lets a bag replay run
on a machine with no ROS installed at all.

Frame dict keys (see headless.run_match and bag2frames):
    t          float, match seconds
    robots     {'us'|'opp': (x, y, th, [f0,f1,f2], phase)}
    stacks     {stack_name: count}      - internal names, not P1..P10 labels
    zones      {zone_name: count}
    score      {'yellow': int, 'blue': int}
    us_state   brain state string       us_path   [(x, y), ...]
    opp_est    {x, y, sigma, visible, seen_ever} | None
    belief     {stack_name: float}      scan_pts  [(x, y), ...]
    opp_state  str                      log       [str, ...]
Optional, added by C1/bag2frames - the whole point of the replay:
    goal       [x, y, th] | None        coarse Nav2 goal
    dock_pose  [x, y, th] | None        final docking pose (never sent to Nav2)
    target     str | None               stage     str
    face       int                      req_yaw_deg / yaw_err_deg  float
    pile_poses {zone_name: (x, y, th)}  where piles really landed
    nav_mode   'nav2_coarse' | 'docking' - which controller is actually
               driving right now (goal set = Nav2's FollowPath/
               motion_controller; goal None = the brain's own direct _goto)
    cmd        (vx, vy, wz) | None      whatever's actually on /cmd_vel
    speed      float | None             linear magnitude of cmd, m/s
"""

import math

TEAM_COL = {'yellow': '#e8b730', 'blue': '#4f7fd0'}
BOX_FC, BOX_EC = '#c99a5b', '#3b2a12'
GOAL_COL, DOCK_COL = '#00d8ff', '#7CFC00'


def normalize_frame(fr):
    """Fill in whatever a frame is missing with harmless defaults.

    Frames arrive from three places with different completeness: headless
    run_match (all 12 keys), bag2frames (those plus the goal fields), and
    World.snapshot() (only t/robots/stacks/zones). Rather than have every
    producer pad its own dicts, the renderer accepts any of them.
    """
    out = dict(fr)
    out.setdefault('t', 0.0)
    out.setdefault('score', {'yellow': 0, 'blue': 0})
    for k in ('us_path', 'scan_pts', 'log'):
        out.setdefault(k, [])
    for k in ('stacks', 'zones', 'belief'):
        out.setdefault(k, {})
    for k in ('us_state', 'opp_state', 'opp_est'):
        out.setdefault(k, None)
    return out


def draw_table(ax, f):
    """Static layer: table, start zones, drop zones, decor circles."""
    from matplotlib.patches import Circle, Polygon, Rectangle
    ax.add_patch(Rectangle((0, 0), f.length, f.width, fc='#5d8a3f', ec='#333333', lw=2))
    for team, col in TEAM_COL.items():
        ax.add_patch(Polygon(f.start_zones[team].poly, closed=True, fc=col, ec='none', alpha=0.85))
        for z in f.team_zones(team):
            ax.add_patch(Polygon(z.poly, closed=True, fc='#9a9a9a', ec=col, lw=2))
    for (x, y, r) in f.decor:
        ax.add_patch(Circle((x, y), r, fc='none', ec='#444444', lw=1))
    for team, poly in getattr(f, 'castle', {}).items():
        if len(poly):
            ax.add_patch(Polygon(poly, closed=True, fc='none', ec=TEAM_COL[team],
                                 lw=1.0, ls=':', alpha=0.7))
    ax.set_xlim(-0.05, f.length + 0.05)
    ax.set_ylim(-0.05, f.width + 0.05)
    ax.set_aspect('equal')
    ax.set_xticks([0, 0.5, 1, 1.5, 2, 2.5, 3])
    ax.set_yticks([0, 0.5, 1, 1.5, 2])


class TableRenderer:
    """Draws frame dicts. `attach()` once, then `draw(frame)` per frame.

    Dynamic artists are tracked and removed each frame; the static table
    layer drawn by attach() persists.
    """

    def __init__(self, field, models, team_us='yellow', lidar_range=2.8,
                 opponent_label='', trail_len=60, show_goal=True):
        self.f = field
        self.models = models
        self.team_us = team_us
        self.team_opp = 'blue' if team_us == 'yellow' else 'yellow'
        self.lidar_range = lidar_range
        self.opponent_label = opponent_label
        self.trail_len = trail_len
        self.show_goal = show_goal
        self.ax = None
        self.side = None
        self._dyn = []
        self._trail = {'us': [], 'opp': []}

    # ------------------------------------------------------------- setup
    def attach(self, ax, side=None):
        self.ax, self.side = ax, side
        if side is not None:
            side.axis('off')
        draw_table(ax, self.f)
        self._dyn = []
        self._trail = {'us': [], 'opp': []}
        return self

    def reset_trails(self):
        """Call when jumping around in a replay - otherwise the trail draws a
        straight line across the table between the two timestamps."""
        self._trail = {'us': [], 'opp': []}

    def seed_trails(self, frames):
        """Rebuild trail history from the frames leading up to the current
        one. Scrubbing to an arbitrary point otherwise means either a missing
        trail or redrawing every intermediate frame just to accumulate it."""
        self._trail = {}
        for fr in frames[-self.trail_len:]:
            for name, rb in fr['robots'].items():
                self._trail.setdefault(name, []).append((rb[0], rb[1]))

    # ------------------------------------------------------------- frame
    def draw(self, fr):
        from matplotlib.patches import Circle, Polygon
        ax, f = self.ax, self.f
        fr = normalize_frame(fr)
        for a in self._dyn:
            try:
                a.remove()
            except (ValueError, NotImplementedError):
                pass
        self._dyn.clear()

        robots = {k: tuple(v) for k, v in fr['robots'].items()}
        for name, rb in robots.items():
            self._trail.setdefault(name, []).append((rb[0], rb[1]))
            del self._trail[name][:-self.trail_len]

        # --- stacks
        for s in f.stacks:
            n = fr['stacks'].get(s.name, 0)
            if n:
                self._patch(Polygon(s.poly, closed=True, fc=BOX_FC, ec=BOX_EC, lw=1.2))
                self._dyn.append(ax.text(s.x, s.y, str(n), ha='center', va='center',
                                         fontsize=9, weight='bold'))
            else:
                self._patch(Polygon(s.poly, closed=True, fc='none', ec=BOX_EC, lw=0.8, ls='--'))

        # --- piles in zones (at the real drop pose when the frame carries it)
        pile_poses = fr.get('pile_poses') or {}
        for z in f.zones:
            n = fr['zones'].get(z.name, 0)
            if not n:
                continue
            pose = pile_poses.get(z.name)
            poly = f.pile_poly(z, tuple(pose) if pose else None)
            self._patch(Polygon(poly, closed=True, fc=BOX_FC, ec=TEAM_COL[z.team],
                                lw=2.5 if n >= z.capacity else 1.2))
            self._dyn.append(ax.text(z.x, z.y, f'{n}/{z.capacity}', ha='center',
                                     va='center', fontsize=8, weight='bold'))

        # --- trails
        for name, col in (('us', 'red'), ('opp', 'white')):
            if len(self._trail.get(name, [])) > 1:
                ln, = ax.plot(*zip(*self._trail[name]), '-', color=col, lw=1.2, alpha=0.7)
                self._dyn.append(ln)

        ux, uy = robots['us'][0], robots['us'][1]

        # --- our planned path
        if fr.get('us_path'):
            pts = [(ux, uy)] + [tuple(p) for p in fr['us_path']]
            ln, = ax.plot(*zip(*pts), ':', color='red', lw=1.5)
            self._dyn.append(ln)

        # --- lidar range + returns
        self._patch(Circle((ux, uy), self.lidar_range, fc='none', ec='red',
                           lw=0.6, ls='--', alpha=0.6))
        if fr.get('scan_pts'):
            self._dyn.append(ax.scatter(*zip(*[tuple(p) for p in fr['scan_pts']]),
                                        s=6, c='yellow', zorder=6))

        # --- opponent estimate
        e = fr.get('opp_est')
        if e and e.get('seen_ever'):
            self._patch(Circle((e['x'], e['y']), min(0.5, max(0.05, e['sigma'])), fc='none',
                               ec='magenta', lw=1.5 if e['visible'] else 0.8,
                               ls='-' if e['visible'] else '--'))

        # --- goal + required heading (the thing a replay exists to check)
        if self.show_goal:
            self._draw_goal(fr)

        # --- robots
        for name, team, edge in (('us', self.team_us, 'red'), ('opp', self.team_opp, 'white')):
            if name not in robots:
                continue
            rx, ry, rth, faces = robots[name][0], robots[name][1], robots[name][2], robots[name][3]
            for a in self._robot_artists(name, rx, ry, rth, faces, TEAM_COL[team], edge):
                if hasattr(a, 'get_xy') or isinstance(a, Circle):
                    ax.add_patch(a)
                self._dyn.append(a)

        if self.side is not None:
            self._dyn.append(self.side.text(0, 1, '\n'.join(self.panel_lines(fr)), va='top',
                                            ha='left', family='monospace', fontsize=8.5))
        sc = fr['score']
        ax.set_title(f"Eurobot 2027  -  yellow {sc['yellow']}  :  {sc['blue']} blue", fontsize=12)
        return self._dyn

    # ------------------------------------------------------------- parts
    def _patch(self, p):
        self.ax.add_patch(p)
        self._dyn.append(p)
        return p

    def _robot_artists(self, name, x, y, th, faces, col, edge):
        import numpy as np
        from matplotlib.patches import Circle, Polygon

        from .geometry import rect_poly, transform
        ax, f = self.ax, self.f
        m = self.models[name]
        arts = [Polygon(transform(m.local_poly, x, y, th), closed=True, fc=col,
                        ec=edge, lw=2, alpha=0.95)]
        c = m.face_center_local(0)
        arts.append(Circle(transform(np.array([c * 0.75]), x, y, th)[0], 0.018, fc=edge, ec='none'))
        for fi, n in enumerate(faces):
            if not n:
                continue
            a = th + m.face_angles[fi]
            nv = np.array([math.cos(a), math.sin(a)])
            bc = np.array([x, y]) + m.r_in * nv + nv * (f.box_width / 2 + 0.003)
            arts.append(Polygon(rect_poly(bc[0], bc[1], f.box_length, f.box_width, a + math.pi / 2),
                                closed=True, fc=BOX_FC, ec=BOX_EC, lw=1))
            arts.append(ax.text(bc[0], bc[1], str(n), ha='center', va='center', fontsize=8,
                                color='black', weight='bold'))
        arts.append(Circle((x, y), m.mast / 2, fc='#222222', ec='white', lw=0.8))
        return arts

    def _draw_goal(self, fr):
        """Ghost footprint + heading arrow at the pose being driven to.

        Two of them: the coarse Nav2 goal (pre-approach point) and the final
        docking pose, which is never sent to Nav2 at all and so is invisible
        everywhere else. The arrow is the requested robot yaw - what the
        replay is for is checking it is the angle it should be.
        """
        from matplotlib.patches import Polygon

        from .geometry import transform
        ax = self.ax
        m = self.models.get('us')
        if m is None:
            return
        for key, col, lw in (('goal', GOAL_COL, 1.2), ('dock_pose', DOCK_COL, 1.8)):
            g = fr.get(key)
            if not g:
                continue
            gx, gy, gth = float(g[0]), float(g[1]), float(g[2])
            self._patch(Polygon(transform(m.local_poly, gx, gy, gth), closed=True,
                                fc='none', ec=col, lw=lw, ls='--', alpha=0.9))
            L = 0.26
            ar = ax.annotate('', xy=(gx + L * math.cos(gth), gy + L * math.sin(gth)),
                             xytext=(gx, gy),
                             arrowprops=dict(arrowstyle='-|>', color=col, lw=lw + 0.4,
                                             shrinkA=0, shrinkB=0))
            self._dyn.append(ar)
            if key == 'dock_pose':
                self._dyn.append(ax.text(gx + L * math.cos(gth), gy + L * math.sin(gth),
                                         f'  {math.degrees(gth):.0f}°', color=col,
                                         fontsize=8, weight='bold', va='center'))

    def panel_lines(self, fr):
        """Side-panel text. Mirrors sim_panel's fields, plus the goal/heading
        block that makes step-by-step angle validation possible."""
        f = self.f
        fr = normalize_frame(fr)
        sc = fr['score']
        robots = {k: tuple(v) for k, v in fr['robots'].items()}
        lines = [
            f"t = {fr['t']:5.1f} s   (left {f.match_duration - fr['t']:4.1f} s)",
            '',
            f"US  ({self.team_us}, red edge)      {sc[self.team_us]:3d} pts",
            f"   state : {fr.get('us_state')}",
            f"   faces : {robots['us'][3]}",
        ]
        if fr.get('target') is not None or fr.get('stage'):
            lines += [
                f"   target: {fr.get('target')}  stage {fr.get('stage')}",
                f"   face  : {fr.get('face')}",
            ]
        if fr.get('nav_mode'):
            cmd = fr.get('cmd')
            cmd_str = f"vx{cmd[0]:+.2f} vy{cmd[1]:+.2f} wz{cmd[2]:+.2f}" if cmd else "-"
            speed = fr.get('speed')
            lines.append(f"   nav   : {fr['nav_mode']:<11} cmd {cmd_str}"
                        + (f"  |v|={speed:.2f}" if speed is not None else ""))
        if fr.get('req_yaw_deg') is not None:
            err = fr.get('yaw_err_deg')
            mark = '' if err is None else ('  OK' if abs(err) <= 10 else f'  OFF {err:+.0f}')
            lines.append(f"   yaw   : want {fr['req_yaw_deg']:+6.1f} deg{mark}")
        if 'opp' in robots:
            lines += [
                '',
                f"OPP ({self.team_opp}, '{self.opponent_label}')  {sc[self.team_opp]:3d} pts",
                f"   state : {fr.get('opp_state')}",
                f"   faces : {robots['opp'][3]}",
            ]
        lines += ['', 'what we believe is left (true):']
        belief = fr.get('belief') or {}
        for s in f.stacks:
            lines.append(f"   {s.label:4s} {s.name:8s} {belief.get(s.name, 0):3.1f}"
                         f"   ({fr['stacks'].get(s.name, 0)})")
        lines += ['', 'strategy log:'] + ['  ' + ln[:38] for ln in (fr.get('log') or [])]
        return lines
