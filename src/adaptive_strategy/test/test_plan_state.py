"""PlanState simulator vs World: same legality, same scoring.

Run: PYTHONPATH=src/table_sim:src/adaptive_strategy python3 src/adaptive_strategy/test/test_plan_state.py
"""
import copy
import os
import random
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'table_sim', 'test'))
import test_scoring as ts  # noqa: E402
from dataclasses import replace as ps_replace  # noqa: E402

from adaptive_strategy.plan_state import PlanModel, apply, legal_actions, points  # noqa: E402

DOCKS = os.path.join(HERE, '..', 'config', 'dock_poses.json')


def make():
    w = ts.make_world()
    with open(os.path.join(ts.CFG, 'sim.yaml')) as fh:
        sim = yaml.safe_load(fh)
    return w, PlanModel(w.field, sim, 'yellow', DOCKS)


def world_pick(w, m, si, f, n):
    r = w.robots['us']
    assert r.faces[f] == 0, 'model must only pick onto empty faces'
    zr = m.regrip.get(si)
    if zr is not None:                       # repick of our own horizontal pile (mirrors World's pick)
        zn = m.zones[zr].name
        w.zones[zn] -= n
        if w.zones[zn] < w.field.zone_capacity:
            w.zone_kind[zn] = None
        r.faces[f] += n
        return
    w.stacks[m.stacks[si].name] -= n
    r.faces[f] += n


def world_flip(w, f):
    r = w.robots['us']
    r.vertical[f] = not r.vertical[f]
    r.vertical_slots[f] = [r.vertical[f] and i < r.faces[f] for i in range(3)]


def world_drop(w, m, zi, f):
    """True if World accepts the drop (mutates w)."""
    r = w.robots['us']
    res = ts.drop_raw(w, m.zones[zi].name, f)
    return res.success


def compare(w, m, s):
    r = w.robots['us']
    assert tuple(r.faces) == s.faces, (r.faces, s.faces)
    assert tuple(r.vertical) == s.vert
    assert tuple(w.zones[z.name] for z in m.zones) == s.zbox
    assert tuple(w.zone_vertical[z.name] for z in m.zones) == s.zvert
    kinds = {None: 0, 'wall': 1, 'tower': 2, 'door': 3}
    assert tuple(kinds[w.zone_kind[z.name]] for z in m.zones) == s.zkind
    assert r.door_used == s.door_used
    park = w.field.scoring['robot_in_end_zone'] if w.score('yellow')['end_zone'] == 'full' else 0
    assert w.score('yellow')['total'] - park == points(m, s, include_park=False)


def test_random_walks(n_walks=40, depth=14, seed=1):
    rng = random.Random(seed)
    checked = 0
    for _ in range(n_walks):
        w, m = make()
        s = m.initial_state()
        for _ in range(depth):
            # legality: every drop candidate must agree with World
            for zi in range(len(m.zones)):
                for f in range(m.n_faces):
                    ok_a = apply(m, s, ('drop', zi, f), check_home=False) is not None
                    if s.flip_debt:
                        continue        # model-only rule (flip completed on the way in), World has no equivalent
                    ok_w = world_drop(copy.deepcopy(w), m, zi, f) if s.faces[f] else False
                    assert ok_a == ok_w, ('drop', m.zone_label[zi], f, s, ok_a, ok_w)
                    checked += 1
            moves = [(a, ns) for a, ns in legal_actions(m, s) if a[0] != 'home']
            if not moves:
                break
            a, ns = rng.choice(moves)
            if a[0] == 'pick':
                world_pick(w, m, a[1], a[2], ns.faces[a[2]])
            elif a[0] == 'flip':
                world_flip(w, a[1])
            elif a[0] == 'regrip':
                r = w.robots['us']
                r.faces[m.flip_face], r.faces[a[1]] = r.faces[a[1]], 0
            else:
                assert world_drop(w, m, a[1], a[2])
            s = ns
            compare(w, m, s)
    print(f'random walks ok ({checked} drop legality checks)')


def test_scripted_scores():
    w, m = make()
    s = m.initial_state()
    z = {l: i for i, l in enumerate(m.zone_label)}
    seq = [('pick', 1, 0), ('pick', 0, 2), ('drop', z['Y1'], 2)]   # face 2 docked at Y1: face 0 can flip there
    for a in seq:
        s = apply(m, s, a, check_home=False)
        assert s, a
    assert points(m, s, False) == 8
    # door on the flip face: flip, 2 posts, un-flip the last box, lid
    for a in [('flip', 0), ('drop', z['Y3'], 0), ('flip', 0), ('drop', z['Y3'], 0)]:
        s = apply(m, s, a, check_home=False)
        assert s, a
    assert points(m, s, False) == 8 + 23 and s.door_used
    print('scripted ok')


def test_timing():
    w, m = make()
    t0 = time.time()
    s = m.initial_state()
    n = 0
    for _ in range(200):
        for a, ns in legal_actions(m, s):
            n += 1
    dt = time.time() - t0
    print(f'{n} transitions in {dt*1000:.0f} ms (incl. travel-matrix warmup); nodes={len(m.travel.nodes)}')
    t0 = time.time()
    for _ in range(200):
        legal_actions(m, s)
    print(f'legal_actions (warm): {(time.time()-t0)/200*1e6:.0f} us/call')


if __name__ == '__main__':
    test_scripted_scores()
    test_random_walks()
    test_timing()


def test_rules_guard():
    from adaptive_strategy.rules import build_guard
    from table_sim import headless
    params = headless.load_params(os.path.join(HERE, '..', 'config', 'strategy.yaml'))
    w, m = make()
    g = build_guard(m, params['drop_rules'])
    z = {l: i for i, l in enumerate(m.zone_label)}
    s = m.initial_state()
    lab = {st.label: i for i, st in enumerate(m.stacks)}
    # P1/P2 feed towers: only onto the flip face
    assert apply(m, s, ('pick', lab['P1'], 1), g) is None and apply(m, s, ('pick', lab['P2'], 2), g) is None
    assert apply(m, s, ('pick', lab['P1'], m.flip_face), g) is not None
    for a in [('pick', lab['P5'], 1), ('pick', lab['P6'], 2), ('pick', lab['P7'], 0)]:
        s = apply(m, s, a, g)
    # the match's first drop must be Y1 or Y2
    for bad in ('Y3', 'Y4', 'Y5', 'YC2'):
        assert apply(m, s, ('drop', z[bad], 0), g) is None, bad
    assert apply(m, s, ('drop', z['Y2'], 1), g) is not None
    # door posts only at Y5 (lid needs an in-castle un-flip); Y5 door may finish after 'last'
    from dataclasses import replace
    v = replace(m.initial_state(), faces=(3, 0, 0), vert=(True, False, False), dropped=(z['Y1'], z['Y3']))
    assert apply(m, v, ('drop', z['Y2'], 0), g) is None and apply(m, v, ('drop', z['Y4'], 0), g) is None
    d1 = apply(m, v, ('drop', z['Y5'], 0), g)
    assert d1 is not None and d1.door_zone == z['Y5']
    d2 = apply(m, d1, ('flip', 0), g)
    assert d2 is not None and apply(m, d2, ('drop', z['Y5'], 0), g).door_used
    # YC2 needs P1 picked, whatever has been dropped
    from dataclasses import replace as _r
    p1 = next(i for i, st in enumerate(m.stacks) if st.label == 'P1')
    base = _r(m.initial_state(), faces=(3, 0, 0), vert=(True, False, False), dropped=(z['Y1'], z['Y3']))
    assert apply(m, base, ('drop', z['YC2'], 0), g) is None
    assert apply(m, _r(base, picked=(p1,)), ('drop', z['YC2'], 0), g) is not None
    # Y3 needs Y1 dropped before, Y4 needs Y2 (not just a first-drop rule)
    s2 = apply(m, s, ('drop', z['Y2'], 1), g)
    assert s2 is not None and apply(m, s2, ('drop', z['Y3'], 2), g) is None
    # YC1 needs Y1+Y3 dropped and P1 picked
    assert apply(m, s, ('drop', z['YC1'], 0), g) is None
    s = apply(m, s, ('drop', z['Y1'], 1), g)
    assert s
    # Y1 first -> only Y3 (or a flip, or abandoning home) may follow; no pick, no other drop
    kinds = {(a[0], a[1] if a[0] == 'drop' else None) for a, _ in legal_actions(m, s, g)}
    assert kinds <= {('drop', z['Y3']), ('flip', None), ('home', None)}, kinds   # home = abandon
    s = apply(m, s, ('drop', z['Y3'], 2), g)
    assert s and any(a[0] == 'pick' for a, _ in legal_actions(m, s, g))
    # Y5 only with just its own face loaded
    g2 = build_guard(m, [r for r in params['drop_rules'] if r.get('zone') == 'Y5'])   # Y5 rule only
    t = m.initial_state()
    t = apply(m, t, ('pick', 0, 1), g2); t = apply(m, t, ('pick', 1, 2), g2)
    assert apply(m, t, ('drop', z['Y5'], 1), g2) is None
    t2 = apply(m, m.initial_state(), ('pick', 0, 1), g2)
    assert apply(m, t2, ('drop', z['Y5'], 1), g2) is not None
    print('rules guard ok')


if __name__ == '__main__':
    test_rules_guard()


def test_scenarios():
    from adaptive_strategy.scenarios import ScenarioSet
    from adaptive_strategy.planner import plan_robust, simulate_open_loop
    w, m = make()
    sc = ScenarioSet(w.field, 'yellow')
    assert abs(sum(s.prob for s in sc.scenarios) - 1) < 1e-9
    # aggressive opponent empties OUR centre/near stacks before the defensive one
    ag = next(s for s in sc.scenarios if s.name == 'aggressive')
    de = next(s for s in sc.scenarios if s.name == 'defensive')
    ours = [i for i, st in enumerate(w.field.stacks) if st.side == 'yellow']
    assert min(ag.take[i] for i in ours) < min(de.take[i] for i in ours)
    # sighting near the aggressive path shifts weight to it
    x, y = ag.pos_at(10.0)
    before = {s.name: s.prob for s in sc.scenarios}
    sc.observe_position(10.0, x, y, 0.05)
    assert next(s.prob for s in sc.scenarios if s.name == 'aggressive') > before['aggressive']
    m.scen = sc
    acts, info = plan_robust(m, m.initial_state(), None, sc, 40.0, beam=60, depth=10)
    assert acts and 'p_ok' in info
    print('scenarios ok', {k: round(v, 2) for k, v in info.items() if isinstance(v, float)})


def test_scenario_updates():
    from adaptive_strategy.scenarios import ScenarioSet
    w, m = make()
    sc = ScenarioSet(w.field, 'yellow')
    names = {s.name for s in sc.scenarios}
    assert 'blocker' in names and abs(sum(s.prob for s in sc.scenarios) - 1) < 1e-9
    bl = next(s for s in sc.scenarios if s.name == 'blocker')
    assert all(t == float('inf') for t in bl.take)
    # one sighting is tempered: it must not flip the belief (old behaviour went 0.20 -> 0.61)
    ag = next(s for s in sc.scenarios if s.name == 'aggressive')
    x, y = ag.pos_at(10.0)
    sc.observe_position(10.0, x, y, 0.05)
    assert max(s.prob for s in sc.scenarios) < 0.6, sc.as_dict()
    # a stack seen still full at t=30 discredits scenarios that emptied it at 2.5 s
    p10 = next(i for i, st in enumerate(w.field.stacks) if st.label == 'P10')
    de = next(s for s in sc.scenarios if s.name == 'defensive')
    before = de.prob
    sc.observe_present(p10, 30.0)
    assert de.prob < before
    # denial now separates stacks (it used to be ~0.95 for almost all of them)
    m.scen = sc
    from adaptive_strategy.plan_state import _denial
    d = [_denial(m, i, 2.0) for i in range(len(w.field.stacks))]
    assert max(d) > 0.3 and min(d) < 0.5 * max(d) and len({round(x, 1) for x in d}) >= 4, d
    print('scenario updates ok')


def test_first_action_candidates_and_hysteresis():
    from adaptive_strategy.scenarios import ScenarioSet
    from adaptive_strategy.planner import plan, plan_robust
    w, m = make()
    sc = ScenarioSet(w.field, 'yellow'); m.scen = sc
    cands = []
    plan(m, m.initial_state(), None, 60, 8, collect=cands)
    firsts = {tuple(c[1][0]) for c in cands}
    assert len(firsts) >= 6, firsts          # many different first actions reach the re-ranking
    acts, info = plan_robust(m, m.initial_state(), None, sc, 40.0, beam=60, depth=8)
    # the previous best plan, offered back, keeps winning unless something clearly beats it
    acts2, info2 = plan_robust(m, m.initial_state(), None, sc, 40.0, beam=60, depth=8, incumbent=acts[1:] if False else acts)
    assert info2['kept_incumbent'] or info2['e_pts'] > info['e_pts']
    print('first-action candidates + hysteresis ok', len(firsts))


def test_keepout_and_early_risk():
    w, m = make()
    ko = w.field.keepout_poly('yellow')
    area = 0.5 * sum(ko[i][0] * ko[(i + 1) % len(ko)][1] - ko[(i + 1) % len(ko)][0] * ko[i][1] for i in range(len(ko)))
    assert area > 0 and ko[:, 0].min() > 1.9          # the mirrored (blue) castle, made CCW
    pl = m.travel.planner
    occ = pl.occupancy()
    assert occ[pl.cell((2.6, 1.0))] and not occ[pl.cell((1.0, 1.0))]   # inside the keep-out vs free table
    # our robot entering the opponent castle is counted as a fault, once per entry
    r = w.robots['us']
    r.x, r.y = 2.6, 1.45                               # in the blue castle, outside blue's start zone
    w._steal_zone = {}                                 # (no 'going to steal' exemption for this timing check)
    w.match_t0, w.time = 0.0, 5.0
    w._check_keepout()
    assert w.castle_faults == 0                        # allowed before 20 s
    w.time = 25.0
    w._check_keepout(); w._check_keepout()
    assert w.castle_faults == 1
    r.x, r.y = 1.0, 1.0
    w._check_keepout(); r.x, r.y = 2.6, 1.45; w._check_keepout()
    assert w.castle_faults == 2
    # the opponent's start zone is forbidden at ALL times; our own never is
    w2, m2 = make()
    r2 = w2.robots['us']
    w2.match_t0, w2.time = 0.0, 2.0
    r2.x, r2.y = 0.25, 1.0                             # our (yellow) start zone
    w2._check_keepout()
    assert w2.castle_faults == 0
    r2.x, r2.y = 2.75, 1.0                             # blue's start zone, at 2 s
    w2._check_keepout(); w2._check_keepout()
    assert w2.castle_faults == 1
    occ2 = m2.travel.planner.occupancy()
    assert occ2[m2.travel.planner.cell((2.75, 1.0))] and not occ2[m2.travel.planner.cell((0.25, 1.0))]
    assert m2.travel.planner_free.occupancy()[m2.travel.planner_free.cell((2.75, 1.0))]   # even when stealing
    near = [st.label for i, st in enumerate(m.stacks) if m.near_opp_castle(i)]
    assert near == ['P9', 'P10'], near
    p9 = next(i for i, st in enumerate(m.stacks) if st.label == 'P9')
    p5 = next(i for i, st in enumerate(m.stacks) if st.label == 'P5')
    assert m.castle_risk(p9, 3) < 0 < m.castle_risk(p9, 50) and m.castle_risk(p5, 3) == 0
    print('keep-out + early risk ok')


def test_regrip_model():
    """('regrip', f): moves face f's horizontal stack onto the empty flip face, only with room around."""
    w, m = make()
    s0 = m.initial_state()
    p5 = next(i for i, st in enumerate(m.stacks) if st.label == 'P5')
    s = ps_replace(s0, node='cur', faces=(0, 3, 0), stacks=tuple(0 if i == p5 else c for i, c in enumerate(s0.stacks)))
    m.travel.set_node('cur', (1.5, 1.0))                 # table centre: free
    ns = apply(m, s, ('regrip', 1))
    assert ns is not None and ns.faces == (3, 0, 0) and ns.node == 'cur' and ns.t > s.t
    assert ns.risk_cost == s.risk_cost + m.regrip_risk
    assert apply(m, s, ('regrip', 0)) is None                                   # the flip face itself
    assert apply(m, ps_replace(s, faces=(1, 3, 0)), ('regrip', 1)) is None      # flip face not empty
    assert apply(m, ps_replace(s, vert=(False, True, False)), ('regrip', 1)) is None   # vertical stack
    m.travel.set_node('cur', (1.32, 1.5))                # P5's pick pose: free once P5 is empty...
    assert apply(m, s, ('regrip', 1)) is not None
    assert apply(m, ps_replace(s, stacks=s0.stacks), ('regrip', 1)) is None    # ...blocked while it has boxes
    for xy in ((0.25, 1.0), (0.2, 0.5), (2.75, 1.0)):    # castles / walls
        m.travel.set_node('cur', xy)
        assert apply(m, s, ('regrip', 1)) is None, xy
    print('regrip model ok')


def test_regrip_world():
    """World: DROP_FLOOR leaves a floor pile, which another face picks back up."""
    import math
    from table_sim.world import DROP_FLOOR, PICK
    w, m = make()
    r = w.robots['us']
    w.match_t0, w.time = 0.0, 1.0
    r.x, r.y, r.th = 1.5, 1.0, 0.0
    r.faces = [0, 3, 0]
    assert w.request_action('us', 1, DROP_FLOOR)[0]
    w.time += 1.0
    w._update_action(r)
    assert r.faces == [0, 0, 0] and len(w.floor_piles) == 1
    (px, py, pth, n), = w.floor_piles.values()
    assert n == 3 and w.score('yellow')['boxes'] == 0
    # face 0 square to the pile's long side, on the robot's side
    a = math.atan2(r.y - py, r.x - px)
    appr = min((pth + math.pi / 2, pth - math.pi / 2), key=lambda v: abs(math.remainder(v - a, 2 * math.pi)))
    d = m.r_in + w.field.box_width / 2 + 0.01
    r.x, r.y, r.th = px + d * math.cos(appr), py + d * math.sin(appr), appr + math.pi
    assert w.request_action('us', 0, PICK)[0]
    for _ in range(2):
        w.time += 1.0
        w._update_action(r)
    assert r.faces == [3, 0, 0] and not w.floor_piles, (r.faces, w.floor_piles)
    print('regrip world ok')


if __name__ == '__main__':
    test_regrip_model()
    test_regrip_world()
    test_keepout_and_early_risk()
    test_scenarios()
    test_scenario_updates()
    test_first_action_candidates_and_hysteresis()
