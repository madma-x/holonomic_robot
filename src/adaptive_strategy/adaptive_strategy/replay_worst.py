"""Replay the worst matches of a brain: 2D GIF + frames JSON + timestamped decision timeline.

    PYTHONPATH=src/table_sim:src/adaptive_strategy python3 -m adaptive_strategy.replay_worst \
        --brain adaptive_strategy.planner_brain:PlannerBrain --params src/adaptive_strategy/config/strategy.yaml \
        --worst 4 --out replays/

Per match it writes <out>/<opp>_s<seed>_<score>.{gif,json,txt}:
  .gif   2D replay (side panel shows the current plan, scenario weights, last result)
  .json  same frames for `python3 -m table_sim.replay_match <file>` (interactive stepping)
  .txt   timeline: world events + the brain's full decision log, with match time
No ROS.
"""
import argparse
import os
import re
import textwrap

import yaml

from table_sim import headless as H
from table_sim.field import Field
from table_sim.opponents import BEHAVIORS

from . import core


def _install_log_capture(store):
    """Time-stamp every brain debug.say() into store[...] (the brain only keeps 20)."""
    clock = {'t': 0.0}
    step0, say0 = core.StrategyCore.step, core.Debug.say

    def step(self, obs, dt):
        clock['t'] = obs.t
        return step0(self, obs, dt)

    def say(self, msg):
        store.append((clock['t'], msg))
        return say0(self, msg)

    core.StrategyCore.step, core.Debug.say = step, say
    return lambda: (setattr(core.StrategyCore, 'step', step0), setattr(core.Debug, 'say', say0))


def _panel_log(entries, t):
    """Side-panel lines at time t: current plan (wrapped), scenarios, last result."""
    plan = scen = last = None
    for et, msg in entries:
        if et > t:
            break
        if msg.startswith('plan '):
            plan = msg
        elif msg.startswith('scenarios '):
            scen = msg
        else:
            last = msg
    out = []
    if plan:
        body = plan.split(': ', 1)[-1].replace('pick:', 'P ').replace('drop:', 'D ')
        out += textwrap.wrap('PLAN ' + body, 38)[:4]
    if scen:
        short = re.sub(r"[{}']", '', scen.split(' | ')[0].replace('scenarios ', ''))
        out += textwrap.wrap('scen ' + short, 38)[:2]
        out.append(scen.split(' | ')[-1][:38])
    if last:
        out += textwrap.wrap('last: ' + last, 38)[:2]
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--brain', default='adaptive_strategy.planner_brain:PlannerBrain')
    ap.add_argument('--params', default=None)
    ap.add_argument('--field', default=None)
    ap.add_argument('--sim', default=None)
    ap.add_argument('--team', default='yellow')
    ap.add_argument('--opponent', default='all')
    ap.add_argument('--seeds', type=int, default=3)
    ap.add_argument('--worst', type=int, default=4)
    ap.add_argument('--out', default='replays')
    ap.add_argument('--fps', type=int, default=10)
    args = ap.parse_args(argv)

    field = Field(args.field)
    with open(args.sim or H.default_sim_file()) as fh:
        sim_cfg = yaml.safe_load(fh)
    params = H.load_params(args.params)
    opps = [b for b in BEHAVIORS if b != 'teleop'] if args.opponent == 'all' else [args.opponent]

    results = []
    for b in opps:
        for seed in range(args.seeds):
            log = []
            restore = _install_log_capture(log)
            try:
                res = H.run_match(field, sim_cfg, args.team, b, args.brain, params, seed=seed,
                                  record_every=0.2)
            finally:
                restore()
            res['full_log'] = log
            results.append(res)
            print(f"{b:13s} seed {seed}: {res['us']['total']:3d} vs {res['them']['total']:3d}"
                  f"  parked={res['us']['end_zone']} carried={res['us']['carried']}")
            if args.opponent != 'all' and False:
                break

    # worst by our score; one replay per distinct (opponent, score) so identical seeds aren't repeated
    results.sort(key=lambda r: (r['us']['total'], r['them']['total'] * -1))
    picked, seen = [], set()
    for r in results:
        k = (r['opponent'], r['us']['total'], r['us']['carried'])
        if k in seen:
            continue
        seen.add(k)
        picked.append(r)
        if len(picked) >= args.worst:
            break

    os.makedirs(args.out, exist_ok=True)
    index = []
    for r in picked:
        stem = os.path.join(args.out, f"{r['opponent']}_s{r['seed']}_{r['us']['total']}")
        for fr in r['frames']:
            fr['log'] = _panel_log(r['full_log'], fr['t'])
        H.dump_frames(r, stem + '.json')
        H.render(r, stem + '.gif', fps=args.fps, speed=2.0, frame_dt=0.2)
        events = [(float(e[0]), 'WORLD', e[1]) for e in r['world'].events]
        events += [(t, 'BRAIN', m) for t, m in r['full_log']]
        events.sort(key=lambda e: (e[0], e[1] != 'WORLD'))
        w = r['world']
        with open(stem + '.txt', 'w') as fh:
            fh.write(f"{r['opponent']} seed {r['seed']}  us {r['us']['total']} vs {r['them']['total']}  "
                     f"{r['us']}\n")
            fh.write(f"zones: {dict((k, v) for k, v in w.zones.items() if v)}  "
                     f"kinds: {dict((k, v) for k, v in w.zone_kind.items() if v)}\n\n")
            for t, who, msg in events:
                fh.write(f"{t:6.1f}s {who:5s} {msg}\n")
        index.append(f"{os.path.basename(stem)}: us {r['us']['total']} vs {r['them']['total']}, "
                     f"{r['us']['end_zone']}, carried {r['us']['carried']}")
        print('wrote', stem + '.{gif,json,txt}')
    with open(os.path.join(args.out, 'index.txt'), 'w') as fh:
        fh.write('\n'.join(index) + '\n')


if __name__ == '__main__':
    main()
