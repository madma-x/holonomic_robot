"""Headless match runner: world + our strategy brain + scripted opponent.

Runs 85 s matches much faster than real time (no ROS needed) to compare
strategy changes against every opponent behaviour, and can render a match
as a video so you can watch both robots move.

    ros2 run table_sim headless_match --opponent all --runs 3
    ros2 run table_sim headless_match --opponent raider --video raider.mp4
    python3 -m table_sim.headless --opponent mirror --video /tmp/m.mp4

Our robot is driven by --brain: "scripted:<behaviour>" (default
scripted:mirror) or "module:Class" for a strategy that exposes step(obs, dt)
and whose module defines Observation(...) and Command. Observation fields:
t, active, time_left, pose, ranges, angle_min, angle_inc, lidar_yaw, cam,
tool_busy, result, flip_busy, flip_result (see run_match below for exactly
what's passed each tick - ranges/cam are None on ticks without a fresh
lidar/camera frame; flip_busy/flip_result track FLIP's own concurrent slot,
separate from tool_busy/result which are PICK/DROP only).
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
            d = dict(v['ros__parameters'])
            break
    # Same as strategy_node: dock_poses_file is relative to the params file.
    dpf = d.get('dock_poses_file')
    if dpf and not os.path.isabs(dpf):
        d['dock_poses_file'] = os.path.join(os.path.dirname(os.path.abspath(path)), dpf)
    return d


def _goal_snapshot(brain, cmd):
    """Mirror of strategy_node._goal_fields for the headless path: the pose
    being driven to and the heading asked for, so a headless run replays in
    replay_match with the same angle validation a recorded match gets."""
    out = {'target': None, 'stage': None, 'face': None, 'goal': None,
           'dock_pose': None, 'req_yaw_deg': None, 'yaw_err_deg': None,
           'mode': getattr(brain, '_mode', None)}
    sp = getattr(brain, 'spot', None)
    if sp is None:
        return out
    face = getattr(brain, 'face', 0)
    req = (sp.face_heading - brain.face_angles[face] + math.pi) % (2 * math.pi) - math.pi
    out.update({
        'target': getattr(brain.target_obj, 'label', None),
        'stage': getattr(brain, 'stage', None),
        'face': face,
        'goal': list(cmd.goal_pose) if cmd.goal_pose is not None else None,
        'dock_pose': [round(sp.x, 4), round(sp.y, 4), round(req, 4)],
        'req_yaw_deg': round(math.degrees(req), 1),
        'yaw_err_deg': round(math.degrees(
            (req - brain.pose[2] + math.pi) % (2 * math.pi) - math.pi), 1),
    })
    return out


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
    cam_period = 1.0 / float(sim_cfg.get('camera', {}).get('rate', 15.0))
    world.start_match()
    pending = None
    flip_pending = None
    next_scan = 0.0
    next_cam = 0.0
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
        cam = None
        if world.time >= next_cam:
            cam = world.camera_view('us')
            next_cam = world.time + cam_period
        res = world.pop_result('us')
        if res is not None:
            pending = (res.face, res.mode, res.success, res.count, res.message)
        flip_res = world.pop_result('us', flip=True)
        if flip_res is not None:
            flip_pending = (flip_res.face, flip_res.success, flip_res.count, flip_res.message)
        obs = mod.Observation(t=world.match_time, active=world.running, time_left=world.time_left,
                              pose=(r.x, r.y, r.th), ranges=ranges, angle_min=a_min,
                              angle_inc=inc, lidar_yaw=lidar_yaw, cam=cam,
                              tool_busy=r.action is not None, result=pending,
                              flip_busy=r.flip_action is not None, flip_result=flip_pending,
                              **({'opp_xy': (world.robots['opp'].x, world.robots['opp'].y)}
                                 if params.get('opponent_beacon') else {}))
        pending, flip_pending = None, None
        cmd = brain.step(obs, control_dt)
        world.set_cmd('us', cmd.vx, cmd.vy, cmd.wz)
        if cmd.cancel:
            world.cancel_action('us')
        if cmd.action is not None:
            count = cmd.action[2] if len(cmd.action) > 2 else 0
            ok, why = world.request_action('us', cmd.action[0], cmd.action[1], count)
            if not ok:
                pending = (cmd.action[0], cmd.action[1], False, 0, why)
        flip_action = getattr(cmd, 'flip_action', None)
        if flip_action is not None and r.flip_action is None:
            ok, why = world.request_action('us', flip_action[0], 2, 0)   # FLIP=2
            if not ok:
                flip_pending = (flip_action[0], False, 0, why)
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
            # Same goal/heading fields strategy_node publishes on
            # /strategy/status, so replay_match can validate a headless run
            # exactly like a recorded one (see bag2frames).
            snap.update(_goal_snapshot(brain, cmd))
            # Where piles really landed, rather than each zone's preset centre
            snap['pile_poses'] = {k: tuple(v) for k, v in world.pile_pose.items()}
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
# drawing  (the renderer itself lives in render.py, shared with replay_match)
# --------------------------------------------------------------------------
from .render import TEAM_COL, TableRenderer, draw_table as _draw_table  # noqa: E402,F401


def render(res, path, fps=20, speed=2.0, frame_dt=0.1):
    """Render recorded frames to an .mp4 (or .gif) video."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import animation

    w = res['world']
    frames = res['frames']
    step = max(1, int(round(speed / (fps * frame_dt))))
    frames = frames[::step]
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax = fig.add_axes([0.02, 0.06, 0.70, 0.88])
    side = fig.add_axes([0.73, 0.06, 0.26, 0.88])
    r = TableRenderer(w.field, w.models, team_us=res['team'],
                      lidar_range=float(w.lidar.get('detect_range', 2.8)),
                      opponent_label=res['opponent']).attach(ax, side)

    anim = animation.FuncAnimation(fig, lambda i: r.draw(frames[i]),
                                   frames=len(frames), interval=1000 / fps, blit=False)
    if path.endswith('.gif'):
        anim.save(path, writer=animation.PillowWriter(fps=fps))
    else:
        anim.save(path, writer=animation.FFMpegWriter(fps=fps, bitrate=2500,
                                                      extra_args=['-pix_fmt', 'yuv420p']))
    plt.close(fig)


def dump_frames(res, path):
    """Write the recorded frames as the same JSON bag2frames produces, so
    replay_match can step through a headless run - no ROS, no bag, no
    container. Tuples become lists through JSON; the renderer handles both."""
    w = res['world']
    with open(path, 'w') as fh:
        json.dump({'frames': res['frames'], 'team': res['team'],
                   'opponent': res['opponent'],
                   'lidar_range': float(w.lidar.get('detect_range', 2.8)),
                   'source': f"headless {res['opponent']} seed {res['seed']}"}, fh)


def snapshot_png(res, path, index=-1):
    """Single still frame (default: the last recorded one).

    `index` used to be accepted and silently ignored - it always drew the
    live final world state regardless.
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    w = res['world']
    frames = res['frames']
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax = fig.add_axes([0.02, 0.06, 0.70, 0.88])
    side = fig.add_axes([0.73, 0.06, 0.26, 0.88])
    r = TableRenderer(w.field, w.models, team_us=res['team'],
                      lidar_range=float(w.lidar.get('detect_range', 2.8)),
                      opponent_label=res['opponent']).attach(ax, side)
    if frames:
        i = index if index >= 0 else len(frames) + index
        i = max(0, min(i, len(frames) - 1))
        r.seed_trails(frames[:i])
        r.draw(frames[i])
    else:
        # no frames recorded (record_every == 0): live final state, which is
        # the same schema minus the strategy-debug keys
        r.draw(w.snapshot())
    ax.set_title(f"us({res['team']}) {res['us']['total']} - {res['them']['total']} "
                 f"opp:{res['opponent']}")
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
    ap.add_argument('--fps', type=int, default=20, help='video frame rate')
    ap.add_argument('--png', default=None, help='final-state picture per opponent')
    ap.add_argument('--frames', default=None,
                    help='.json of the last match per opponent, for replay_match '
                         '(same schema bag2frames writes, minus what only a live '
                         'run records)')
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
    print(f"{'team':6} {'opponent':13} {'seed':>4} | {'us':>4} {'them':>4} | boxes W T D carried end     contacts faults")
    for team in teams:
        for b in behaviors:
            last = None
            for seed in range(args.seed, args.seed + args.runs):
                want = (bool(args.video or args.png or args.frames)
                        and seed == args.seed + args.runs - 1)
                res = run_match(field, sim_cfg, team, b, args.brain, params, seed=seed,
                                avoid=not args.no_avoid, record_every=0.1 if want else 0.0,
                                verbose=args.verbose)
                u, th = res['us'], res['them']
                print(f"{team:6} {b:13} {seed:4d} | {u['total']:4d} {th['total']:4d} | "
                      f"{u['boxes']:5d} {u['walls']} {u['towers']} {u['doors']} {u['carried']:7d} "
                      f"{u['end_zone']:7} {u['contacts']:3d} {u.get('castle_faults', 0):6d}")
                rows.append({k: v for k, v in res.items() if k not in ('world', 'frames')})
                last = res
            # --speed/--fps were declared but never passed through, so they
            # silently did nothing - render always used its own defaults.
            for opt, fn, kw in ((args.video, render, {'fps': args.fps, 'speed': args.speed}),
                                (args.png, snapshot_png, {}),
                                (args.frames, dump_frames, {})):
                if opt and last is not None:
                    root, ext = os.path.splitext(opt)
                    out = f'{root}_{team}_{b}{ext}' if multi else opt
                    fn(last, out, **kw)
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
