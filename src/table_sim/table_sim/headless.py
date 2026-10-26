"""Headless match runner: world + our strategy brain + scripted opponent.

Runs 85 s matches much faster than real time (no ROS needed) to compare
strategy changes against every opponent behaviour, and can render a match
as a video so you can watch both robots move.

    ros2 run table_sim headless_match --opponent all --runs 3
    ros2 run table_sim headless_match --opponent raider --video raider.mp4
    python3 -m table_sim.headless --opponent mirror --video /tmp/m.mp4

Our robot is driven by --brain: "scripted:<behaviour>" (default
scripted:mirror) or "module:Class" for a strategy that exposes step(obs, dt)
and whose module defines Observation(...) and Command.
"""

import argparse
import importlib
import json
import math
import os
import sys
import time

import yaml

from .field import Field
from .opponents import BEHAVIORS, ScriptedOpponent
from .world import World


def default_sim_file():
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [os.path.join(here, '..', 'config', 'sim.yaml')]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.insert(0, os.path.join(get_package_share_directory('table_sim'), 'config', 'sim.yaml'))
    except Exception:  # noqa: BLE001
        pass
    for c in cands:
        if os.path.exists(c):
            return os.path.abspath(c)
    raise FileNotFoundError('sim.yaml not found')


def load_brain(spec):
    mod_name, cls_name = spec.split(':')
    mod = importlib.import_module(mod_name)
    return mod, getattr(mod, cls_name)


def load_params(path):
    if not path:
        return {}
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    for v in d.values():          # accept a ROS params file
        if isinstance(v, dict) and 'ros__parameters' in v:
            return dict(v['ros__parameters'])
    return d


def run_match(field, sim_cfg, team, behavior, brain_spec, params, seed=0, avoid=True,
              control_dt=0.05, record_every=0.0, verbose=False):
    world = World(field, sim_cfg, team_us=team, seed=seed)
    opp = ScriptedOpponent(world, behavior=behavior, avoid_us=avoid, seed=seed,
                           speed_scale=float(sim_cfg.get('opponent', {}).get('speed_scale', 1.0)))
    us_script = None
    if brain_spec.startswith('scripted'):
        beh = brain_spec.split(':', 1)[1] if ':' in brain_spec else 'mirror'
        us_script = ScriptedOpponent(world, behavior=beh, avoid_us=True, seed=seed, name='us')
        mod = brain = None
    else:
        mod, cls = load_brain(brain_spec)
        brain = cls(field, team, params)
    lidar_period = 1.0 / float(sim_cfg.get('lidar', {}).get('rate', 10.0))
    lidar_yaw = float(sim_cfg.get('lidar', {}).get('yaw_offset', math.pi))
    world.start_match()
    pending = None
    next_scan = 0.0
    next_rec = 0.0
    frames = []
    last_scan_pts = None
    steps = int(math.ceil(field.match_duration / control_dt)) + 2
    for _ in range(steps):
        r = world.robots['us']
        if us_script is not None:
            us_script.step(control_dt)
            opp.step(control_dt)
            world.step(control_dt)
            if record_every and world.time >= next_rec:
                next_rec = world.time + record_every
                snap = world.snapshot()
                snap.update({'us_state': us_script.status, 'us_path': list(us_script.path),
                             'opp_est': None, 'belief': {}, 'opp_state': opp.status, 'scan_pts': [],
                             'score': {t: world.score(t)['total'] for t in ('yellow', 'blue')}, 'log': []})
                frames.append(snap)
            if world.finished:
                break
            continue
        ranges, a_min, inc = None, -math.pi, 0.0
        if world.time >= next_scan:
            ranges, a_min, inc = world.scan('us')
            next_scan = world.time + lidar_period
        res = world.pop_result('us')
        if res is not None:
            pending = (res.face, res.mode, res.success, res.count, res.message)
        obs = mod.Observation(t=world.match_time, active=world.running, time_left=world.time_left,
                              pose=(r.x, r.y, r.th), ranges=ranges, angle_min=a_min,
                              angle_inc=inc, lidar_yaw=lidar_yaw,
                              tool_busy=r.action is not None, result=pending)
        pending = None
        cmd = brain.step(obs, control_dt)
        world.set_cmd('us', cmd.vx, cmd.vy, cmd.wz)
        if cmd.cancel:
            world.cancel_action('us')
        if cmd.action is not None:
            ok, why = world.request_action('us', cmd.action[0], cmd.action[1])
            if not ok:
                pending = (cmd.action[0], cmd.action[1], False, 0, why)
        opp.step(control_dt)
        world.step(control_dt)
        if record_every and world.time >= next_rec:
            next_rec = world.time + record_every
            snap = world.snapshot()
            dbg = getattr(brain, 'debug', None)
            if ranges is not None and hasattr(brain, 'tracker'):
                last_scan_pts = [tuple(p) for p in brain.tracker.last_points]
            snap['us_state'] = getattr(dbg, 'state', '')
            snap['us_path'] = list(getattr(dbg, 'path', []) or [])
            snap['opp_est'] = dict(dbg.opp) if dbg is not None and dbg.opp else None
            snap['belief'] = dict(getattr(dbg, 'belief', {}) or {})
            snap['opp_state'] = opp.status
            snap['scan_pts'] = last_scan_pts or []
            snap['score'] = {t: world.score(t)['total'] for t in ('yellow', 'blue')}
            snap['log'] = (getattr(dbg, 'log', []) or [])[-4:]
            frames.append(snap)
        if world.finished:
            break
    final = world.final_score or {t: world.score(t) for t in ('yellow', 'blue')}
    log = getattr(getattr(brain, 'debug', None), 'log', [])
    if verbose:
        for e in world.events:
            print('   world', e)
        for line in log:
            print('   brain', line)
    them = 'blue' if team == 'yellow' else 'yellow'
    return {'team': team, 'opponent': behavior, 'seed': seed,
            'us': final[team], 'them': final[them], 'world': world,
            'frames': frames, 'brain_log': log}


# --------------------------------------------------------------------------
# drawing
# --------------------------------------------------------------------------
TEAM_COL = {'yellow': '#e8b730', 'blue': '#4f7fd0'}


def _draw_table(ax, f):
    from matplotlib.patches import Polygon, Circle, Rectangle
    ax.add_patch(Rectangle((0, 0), f.length, f.width, fc='#5d8a3f', ec='#333333', lw=2))
    for team, col in TEAM_COL.items():
        ax.add_patch(Polygon(f.start_zones[team].poly, closed=True, fc=col, ec='none', alpha=0.85))
        for z in f.team_zones(team):
            ax.add_patch(Polygon(z.poly, closed=True, fc='#9a9a9a', ec=col, lw=2))
    for (x, y, r) in f.decor:
        ax.add_patch(Circle((x, y), r, fc='none', ec='#444444', lw=1))
    ax.set_xlim(-0.05, f.length + 0.05)
    ax.set_ylim(-0.05, f.width + 0.05)
    ax.set_aspect('equal')
    ax.set_xticks([0, 0.5, 1, 1.5, 2, 2.5, 3])
    ax.set_yticks([0, 0.5, 1, 1.5, 2])


def render(res, path, fps=20, speed=2.0, frame_dt=0.1):
    """Render recorded frames to an .mp4 (or .gif) video."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.patches import Polygon, Circle
    import numpy as np
    from .geometry import rect_poly, transform

    w = res['world']
    f = w.field
    frames = res['frames']
    step = max(1, int(round(speed / (fps * frame_dt))))
    frames = frames[::step]
    team_us = res['team']
    team_opp = 'blue' if team_us == 'yellow' else 'yellow'
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax = fig.add_axes([0.02, 0.06, 0.70, 0.88])
    side = fig.add_axes([0.73, 0.06, 0.26, 0.88])
    side.axis('off')
    _draw_table(ax, f)
    dyn = []
    trail = {'us': [], 'opp': []}

    def robot_artists(name, x, y, th, faces, col, edge):
        m = w.models[name]
        arts = [Polygon(transform(m.local_poly, x, y, th), closed=True, fc=col, ec=edge, lw=2, alpha=0.95)]
        # face 0 marker
        c = m.face_center_local(0)
        p = transform(np.array([c * 0.75]), x, y, th)[0]
        arts.append(Circle(p, 0.018, fc=edge, ec='none'))
        for fi, n in enumerate(faces):
            if n:
                a = th + m.face_angles[fi]
                nv = np.array([math.cos(a), math.sin(a)])
                fc = np.array([x, y]) + m.r_in * nv
                bc = fc + nv * (f.box_width / 2 + 0.003)
                arts.append(Polygon(rect_poly(bc[0], bc[1], f.box_length, f.box_width, a + math.pi / 2),
                                    closed=True, fc='#c99a5b', ec='#3b2a12', lw=1))
                arts.append(ax.text(bc[0], bc[1], str(n), ha='center', va='center', fontsize=8,
                                    color='black', weight='bold'))
        arts.append(Circle((x, y), m.mast / 2, fc='#222222', ec='white', lw=0.8))
        return arts

    def draw(i):
        for a in dyn:
            a.remove()
        dyn.clear()
        fr = frames[i]
        for name, (x, y, th, faces, phase) in fr['robots'].items():
            trail[name].append((x, y))
            del trail[name][:-60]
        # stacks
        for s in f.stacks:
            n = fr['stacks'][s.name]
            if n:
                p = Polygon(s.poly, closed=True, fc='#c99a5b', ec='#3b2a12', lw=1.2)
                ax.add_patch(p)
                dyn.append(p)
                dyn.append(ax.text(s.x, s.y, str(n), ha='center', va='center', fontsize=9, weight='bold'))
            else:
                p = Polygon(s.poly, closed=True, fc='none', ec='#3b2a12', lw=0.8, ls='--')
                ax.add_patch(p)
                dyn.append(p)
        # piles in zones
        for z in f.zones:
            n = fr['zones'][z.name]
            if n:
                p = Polygon(f.pile_poly(z), closed=True, fc='#c99a5b',
                            ec=TEAM_COL[z.team], lw=2.5 if n >= f.zone_capacity else 1.2)
                ax.add_patch(p)
                dyn.append(p)
                dyn.append(ax.text(z.x, z.y, f'{n}/{f.zone_capacity}', ha='center', va='center',
                                   fontsize=8, weight='bold'))
        # trails
        for name, col in (('us', 'red'), ('opp', 'white')):
            if len(trail[name]) > 1:
                ln, = ax.plot(*zip(*trail[name]), '-', color=col, lw=1.2, alpha=0.7)
                dyn.append(ln)
        # our planned path
        if fr['us_path']:
            x, y = fr['robots']['us'][0], fr['robots']['us'][1]
            pts = [(x, y)] + list(fr['us_path'])
            ln, = ax.plot(*zip(*pts), ':', color='red', lw=1.5)
            dyn.append(ln)
        # lidar range + returns
        x, y = fr['robots']['us'][0], fr['robots']['us'][1]
        c = Circle((x, y), 1.0, fc='none', ec='red', lw=0.6, ls='--', alpha=0.6)
        ax.add_patch(c)
        dyn.append(c)
        if fr['scan_pts']:
            sc = ax.scatter(*zip(*fr['scan_pts']), s=6, c='yellow', zorder=6)
            dyn.append(sc)
        # opponent estimate
        e = fr['opp_est']
        if e and e.get('seen_ever'):
            c = Circle((e['x'], e['y']), min(0.5, max(0.05, e['sigma'])), fc='none',
                       ec='magenta', lw=1.5 if e['visible'] else 0.8, ls='-' if e['visible'] else '--')
            ax.add_patch(c)
            dyn.append(c)
        # robots
        for name, team, edge in (('us', team_us, 'red'), ('opp', team_opp, 'white')):
            rx, ry, rth, faces, _ = fr['robots'][name]
            for a in robot_artists(name, rx, ry, rth, faces, TEAM_COL[team], edge):
                if hasattr(a, 'get_xy') or isinstance(a, Circle):
                    ax.add_patch(a)
                dyn.append(a)
        # side panel
        sc = fr['score']
        lines = [
            f"t = {fr['t']:5.1f} s   (left {f.match_duration - fr['t']:4.1f} s)",
            '',
            f"US  ({team_us}, red edge)      {sc[team_us]:3d} pts",
            f"   state : {fr['us_state']}",
            f"   faces : {fr['robots']['us'][3]}",
            '',
            f"OPP ({team_opp}, '{res['opponent']}')  {sc[team_opp]:3d} pts",
            f"   state : {fr['opp_state']}",
            f"   faces : {fr['robots']['opp'][3]}",
            '',
            'what we believe is left (true):',
        ]
        for s in f.stacks:
            lines.append(f"   {s.label:4s} {s.name:8s} {fr['belief'].get(s.name, 0):3.1f}   ({fr['stacks'][s.name]})")
        lines += ['', 'strategy log:'] + ['  ' + ln[:38] for ln in fr['log']]
        dyn.append(side.text(0, 1, '\n'.join(lines), va='top', ha='left', family='monospace', fontsize=8.5))
        ax.set_title(f"Eurobot 2027 sim  -  yellow {sc['yellow']}  :  {sc['blue']} blue", fontsize=12)
        return dyn

    anim = animation.FuncAnimation(fig, draw, frames=len(frames), interval=1000 / fps, blit=False)
    if path.endswith('.gif'):
        anim.save(path, writer=animation.PillowWriter(fps=fps))
    else:
        anim.save(path, writer=animation.FFMpegWriter(fps=fps, bitrate=2500,
                                                      extra_args=['-pix_fmt', 'yuv420p']))
    plt.close(fig)


def snapshot_png(res, path, index=-1):
    """Single still frame (default: final state)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    w = res['world']
    f = w.field
    fig, ax = plt.subplots(figsize=(10, 7))
    _draw_table(ax, f)
    for s in f.stacks:
        if w.stacks[s.name]:
            ax.add_patch(Polygon(s.poly, closed=True, fc='#c99a5b', ec='k'))
            ax.text(s.x, s.y, str(w.stacks[s.name]), ha='center', va='center')
    for z in f.zones:
        if w.zones[z.name]:
            ax.add_patch(Polygon(f.pile_poly(z), closed=True, fc='#c99a5b', ec=TEAM_COL[z.team], lw=2))
            ax.text(z.x, z.y, str(w.zones[z.name]), ha='center', va='center')
    for k, col in (('us', 'red'), ('opp', 'white')):
        tr = [(fr['robots'][k][0], fr['robots'][k][1]) for fr in res['frames']]
        if tr:
            ax.plot(*zip(*tr), '-', color=col, lw=1)
        ax.add_patch(Polygon(w.robots[k].poly, closed=True, fc='none', ec=col, lw=2))
    ax.set_title(f"us({res['team']}) {res['us']['total']} - {res['them']['total']} opp:{res['opponent']}")
    fig.savefig(path, dpi=90, bbox_inches='tight')
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--field', default=None)
    ap.add_argument('--sim', default=None)
    ap.add_argument('--brain', default='scripted:mirror',
                    help='scripted:<behaviour> or module:Class (e.g. adaptive_strategy.core:StrategyCore)')
    ap.add_argument('--params', default=None, help='strategy yaml (ROS params file ok)')
    ap.add_argument('--team', default='yellow', choices=['yellow', 'blue', 'both'])
    ap.add_argument('--opponent', default='all', help=f"one of {BEHAVIORS} or 'all'")
    ap.add_argument('--runs', type=int, default=3, help='seeds per opponent')
    ap.add_argument('--seed', type=int, default=0, help='first seed')
    ap.add_argument('--no-avoid', action='store_true', help='opponent does not avoid us')
    ap.add_argument('--video', default=None, help='.mp4/.gif of the last match per opponent')
    ap.add_argument('--speed', type=float, default=2.0, help='video playback speed-up')
    ap.add_argument('--png', default=None, help='final-state picture per opponent')
    ap.add_argument('--json', default=None)
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args(argv)

    field = Field(args.field)
    with open(args.sim or default_sim_file()) as fh:
        sim_cfg = yaml.safe_load(fh)
    params = load_params(args.params)
    behaviors = [b for b in BEHAVIORS if b != 'teleop'] if args.opponent == 'all' else [args.opponent]
    teams = ['yellow', 'blue'] if args.team == 'both' else [args.team]
    multi = len(behaviors) * len(teams) > 1
    rows = []
    t0 = time.time()
    print(f"{'team':6} {'opponent':13} {'seed':>4} | {'us':>4} {'them':>4} | boxes stacks carried end     contacts")
    for team in teams:
        for b in behaviors:
            last = None
            for seed in range(args.seed, args.seed + args.runs):
                want = bool(args.video or args.png) and seed == args.seed + args.runs - 1
                res = run_match(field, sim_cfg, team, b, args.brain, params, seed=seed,
                                avoid=not args.no_avoid, record_every=0.1 if want else 0.0,
                                verbose=args.verbose)
                u, th = res['us'], res['them']
                print(f"{team:6} {b:13} {seed:4d} | {u['total']:4d} {th['total']:4d} | "
                      f"{u['boxes']:5d} {u['full_stacks']:6d} {u['carried']:7d} "
                      f"{u['end_zone']:7} {u['contacts']:3d}")
                rows.append({k: v for k, v in res.items() if k not in ('world', 'frames')})
                last = res
            for opt, fn in ((args.video, render), (args.png, snapshot_png)):
                if opt and last is not None:
                    root, ext = os.path.splitext(opt)
                    out = f'{root}_{team}_{b}{ext}' if multi else opt
                    fn(last, out)
                    print(f'   -> {out}')
    n = len(rows)
    if n:
        avg_us = sum(r['us']['total'] for r in rows) / n
        avg_them = sum(r['them']['total'] for r in rows) / n
        wins = sum(r['us']['total'] > r['them']['total'] for r in rows)
        home = sum(r['us']['end_zone'] == 'full' for r in rows)
        coll = sum(r['us']['contacts'] for r in rows)
        print(f'\n{n} matches in {time.time() - t0:.1f}s | avg us {avg_us:.1f} vs {avg_them:.1f} | '
              f'wins {wins}/{n} | parked {home}/{n} | contacts {coll}')
    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(rows, fh, indent=1, default=str)
    return 0


if __name__ == '__main__':
    sys.exit(main())
