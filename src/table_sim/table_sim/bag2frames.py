"""Turn a recorded match (rosbag) into the plain frame dicts render.py draws.

Stage 1 of the two-stage replay. Needs ROS (rosbag2_py), so it runs in the
container; the output is plain JSON, so the viewer (replay_match.py) runs
anywhere, including a laptop with no ROS at all.

    ros2 run table_sim bag2frames /tmp/table_sim_bags/20261101_1200 -o match.json
    python3 -m table_sim.replay_match match.json         # anywhere

The frame schema is exactly what headless.run_match produces (so the same
renderer serves both), plus the goal/heading fields from /strategy/status
that make step-by-step angle validation possible. See render.py's docstring.

Topic -> frame mapping:
    /sim/score        t, robots, stacks, zones, score, opp_state, events
    /strategy/status  us_state, belief, opp_est, target, stage, face,
                      goal, dock_pose, req_yaw_deg, yaw_err_deg, log
    /scan             scan_pts (raycast back out to world points)
    /cmd_vel          cmd (vx, vy, wz), speed (linear magnitude) - whatever
                      is actually driving the robot right now, regardless
                      of source (direct docking control or Nav2's
                      FollowPath/motion_controller)

nav_mode (derived, not a topic): 'nav2_coarse' while /strategy/status's
`goal` is set (the brain handed Nav2 a NavigateToPose goal), else 'docking'
(the brain's own direct _goto, final approach or backout).

Sampling: one frame per `--dt` of match time, carrying the most recent
message seen on each topic - the topics publish at different rates (10 Hz
score, 20 Hz status, 10 Hz scan) and nothing lines them up, so a frame is a
snapshot of "latest of each" rather than a true simultaneous sample.
"""

import argparse
import json
import math
import os
import sys

from .field import Field, default_field_file


def _open(path):
    import rosbag2_py
    storage_id = ''
    meta = os.path.join(path, 'metadata.yaml')
    if os.path.isdir(path) and os.path.exists(meta):
        import yaml
        with open(meta) as f:
            info = yaml.safe_load(f) or {}
        storage_id = (info.get('rosbag2_bagfile_information', {})
                          .get('storage_identifier', '') or '')
    elif os.path.isdir(path):
        # No metadata.yaml - e.g. the recorder was killed (SIGKILL/crash)
        # rather than shut down cleanly, so it never got to write one.
        # Infer the storage plugin from whatever's actually in the directory
        # rather than giving up: the bag is still readable.
        exts = {os.path.splitext(f)[1] for f in os.listdir(path)}
        if '.mcap' in exts:
            storage_id = 'mcap'
        elif '.db3' in exts:
            storage_id = 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def _scan_points(scan, pose):
    """LaserScan -> world-frame hit points. Same angle convention world.py's
    own scan() uses, so the points land where the sim says they should."""
    if scan is None or pose is None:
        return []
    x, y, th = pose
    pts = []
    for i, rng in enumerate(scan.ranges):
        if not math.isfinite(rng) or rng <= 0.0 or rng >= scan.range_max:
            continue
        a = th + scan.angle_min + i * scan.angle_increment
        pts.append((round(x + rng * math.cos(a), 4), round(y + rng * math.sin(a), 4)))
    return pts


def _relabel(d, name_by_label):
    """{'P1': 3} -> {'Y_hi_1': 3}. /sim/score and /strategy/status are keyed
    by label; the renderer and field.py are keyed by internal name."""
    if not d:
        return {}
    return {name_by_label.get(k, k): v for k, v in d.items()}


def extract(bag, field=None, dt=0.1):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    f = field or Field(default_field_file())
    name_by_label = f.name_by_label()
    reader, types = _open(bag)

    score = status = scan = cmd_vel = None
    frames = []
    next_t = 0.0
    last_t = None

    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        msg_type = types.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(data, get_message(msg_type))
        if topic == '/sim/score':
            score = json.loads(msg.data)
        elif topic == '/strategy/status':
            status = json.loads(msg.data)
        elif topic == '/scan':
            scan = msg
        elif topic == '/cmd_vel':
            cmd_vel = msg
        else:
            continue

        if score is None:
            continue
        t = float(score.get('time', 0.0))
        # a /sim/reset_event or a re-run rewinds the clock: start over
        if last_t is not None and t < last_t - 1.0:
            frames, next_t = [], 0.0
        last_t = t
        if t < next_t:
            continue
        next_t = t + dt
        frames.append(_frame(t, score, status, scan, cmd_vel, name_by_label))

    return {'frames': frames, 'team': (score or {}).get('team', 'yellow'),
            'opponent': ((score or {}).get('robots', {}).get('opp', {}) or {}).get('behavior', ''),
            'source': os.path.abspath(bag)}


def _frame(t, score, status, scan, cmd_vel, name_by_label):
    robots = {}
    for key, rb in (score.get('robots') or {}).items():
        robots[key] = (rb.get('x', 0.0), rb.get('y', 0.0),
                       math.radians(rb.get('th_deg', 0.0)),
                       list(rb.get('faces') or [0, 0, 0]),
                       rb.get('action', ''))
    us_pose = robots.get('us', (0.0, 0.0, 0.0))[:3]
    sc = score.get('score') or {}
    st = status or {}
    goal = st.get('goal')
    if cmd_vel is not None:
        cmd = (round(cmd_vel.linear.x, 4), round(cmd_vel.linear.y, 4), round(cmd_vel.angular.z, 4))
        speed = round(math.hypot(cmd_vel.linear.x, cmd_vel.linear.y), 4)
    else:
        cmd, speed = None, None

    fr = {
        't': round(t, 2),
        'robots': robots,
        'stacks': _relabel(score.get('stacks'), name_by_label),
        'zones': _relabel(score.get('zones'), name_by_label),
        'score': {k: (sc.get(k) or {}).get('total', 0) for k in ('yellow', 'blue')},
        'us_state': st.get('state'),
        'us_path': [],      # GridPlanner waypoints are brain-internal, not published
        'opp_est': st.get('opponent'),
        'belief': _relabel(st.get('belief'), name_by_label),
        'opp_state': (score.get('robots', {}).get('opp', {}) or {}).get('status', ''),
        'scan_pts': _scan_points(scan, us_pose),
        'log': [e for e in (score.get('events') or [])][-4:],
        # --- what the replay exists to check
        'target': st.get('target'),
        'stage': st.get('stage'),
        'face': st.get('face'),
        'goal': st.get('goal'),
        'dock_pose': st.get('dock_pose'),
        'req_yaw_deg': st.get('req_yaw_deg'),
        'yaw_err_deg': st.get('yaw_err_deg'),
        'mode': st.get('mode'),
        # nav2_coarse while the brain has handed Nav2 a goal (goal set);
        # docking otherwise - the brain's own direct _goto, final approach
        # or backout, not path-planned.
        'nav_mode': 'nav2_coarse' if goal is not None else 'docking',
        'cmd': cmd,      # (vx, vy, wz) actually on /cmd_vel right now
        'speed': speed,  # linear magnitude of cmd, m/s
    }
    return fr


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag', help='bag directory (as written by sim.launch.py record:=true)')
    ap.add_argument('-o', '--out', default=None, help='output .json (default <bag>.json)')
    ap.add_argument('--dt', type=float, default=0.1, help='frame interval, match seconds')
    ap.add_argument('--field', default=None)
    args = ap.parse_args(argv)

    out = args.out or (args.bag.rstrip('/') + '.json')
    res = extract(args.bag, Field(args.field) if args.field else None, args.dt)
    if not res['frames']:
        print('no frames extracted - was /sim/score recorded?', file=sys.stderr)
        return 1
    with open(out, 'w') as f:
        json.dump(res, f)
    n = len(res['frames'])
    print(f"{n} frames ({res['frames'][0]['t']:.1f} -> {res['frames'][-1]['t']:.1f} s)  -> {out}")
    with_goal = sum(1 for fr in res['frames'] if fr.get('req_yaw_deg') is not None)
    print(f"  {with_goal} frames carry a requested heading")
    return 0


if __name__ == '__main__':
    sys.exit(main())
