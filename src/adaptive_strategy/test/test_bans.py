"""Ban durations by reason, the tight pick rule, home replanning and scouting viewpoints.

Run: PYTHONPATH=src/table_sim:src/adaptive_strategy python3 -m pytest src/adaptive_strategy/test/test_bans.py
"""
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'table_sim', 'test'))
import test_scoring as ts  # noqa: E402

from adaptive_strategy.core import Observation, StrategyCore  # noqa: E402
from adaptive_strategy.planner_brain import PlannerBrain  # noqa: E402
from adaptive_strategy.scout import Viewpoints  # noqa: E402


def params():
    with open(os.path.join(HERE, '..', 'config', 'strategy.yaml')) as fh:
        d = yaml.safe_load(fh)
    p = dict(d['strategy_node']['ros__parameters'])
    p['dock_poses_file'] = os.path.join(HERE, '..', 'config', 'dock_poses.json')
    return p


def brain(cls=PlannerBrain):
    w = ts.make_world()
    with open(os.path.join(ts.CFG, 'sim.yaml')) as fh:
        sim = yaml.safe_load(fh)
    b = cls(w.field, 'yellow', params(), sim_cfg=sim) if cls is PlannerBrain else cls(w.field, 'yellow', params())
    return w, b


def obs(t, pose=(0.25, 1.0, 0.0), cam=None):
    return Observation(t, True, 85.0 - t, pose, None, -math.pi, 0.0, math.pi, cam, False, None)


def set_opp(b, x, y, vx=0.0, vy=0.0, visible=True):
    o = b.tracker.opp
    o.x, o.y, o.vx, o.vy, o.sigma, o.visible, o.seen_ever = x, y, vx, vy, 0.05, visible, True


# ----------------------------------------------------------------- bans
def test_no_flat_extension():
    """A skip the core sets itself keeps its own duration (the old +15 s extension is gone)."""
    _, b = brain()
    b._skip_until['P1'] = 1.5
    assert 'P1' in b._banned(0.0)
    assert b._skip_until['P1'] == 1.5
    assert 'P1' not in b._banned(1.6)


def test_registered_ban_durations_and_escalation():
    _, b = brain()
    st = b.field.stacks[0]
    b._register_ban(st.name, b.take_ban, 'take', 10.0, (st.x, st.y))
    assert b.take_ban == 3.0
    assert st.name in b._banned(12.9)
    assert st.name not in b._banned(13.1)
    # repeated opponent-caused stuck drops of one zone: 3 -> 6 -> 12 s
    z = b.model.zones[0]
    b.target_kind, b.target_obj, b.pose = 'drop', z, (z.x, z.y, 0.0)
    set_opp(b, z.x + 0.3, z.y)
    durs = []
    for k in range(4):
        b._stuck_drop(obs(20.0 + 20 * k))
        durs.append(round(b._skip_until[z.name] - (20.0 + 20 * k), 1))
    assert durs == [3.0, 6.0, 12.0, 12.0]


def test_take_ban_lifted_when_camera_sees_it_full_and_opponent_away():
    _, b = brain()
    st = b.field.stacks[0]
    b._register_ban(st.name, b.take_ban, 'take', 10.0, (st.x, st.y))
    set_opp(b, st.x + 0.1, st.y)                       # opponent still at it: stays banned
    b.belief.on_camera({st.name: 3}, 10.5)
    assert st.name in b._banned(10.6)
    set_opp(b, st.x + 1.5, st.y)                       # opponent gone, camera sees it full: lifted
    b.belief.on_camera({st.name: 3}, 11.0)
    assert st.name not in b._banned(11.1)


def test_stuck_with_nobody_near_is_impossible_until_signature_changes():
    _, b = brain()
    z = b.model.zones[0]
    b.target_kind, b.target_obj, b.pose = 'drop', z, (1.5, 1.0, 0.0)    # far from the opponent (not visible)
    b.faces = [3, 0, 0]
    b._stuck_drop(obs(30.0))
    assert z.name not in b._skip_until                                   # no time ban
    assert z.name in b._impossible_names()
    b.faces = [3, 3, 0]                                                  # what we carry changed
    assert z.name not in b._impossible_names()
    b.faces = [3, 0, 0]
    assert z.name in b._impossible_names()                               # same signature again


def test_pick_no_stack_in_reach_zeroes_belief_without_ban():
    _, b = brain()
    st = b.field.stacks[0]
    assert b.belief._count[st.name] > 0
    b._pick_failed(st, 'no stack in reach')
    assert b.belief._count[st.name] == 0.0
    assert st.name not in b._skip_until


# ------------------------------------------------- tight rule replaces the pre-pose deferral
def test_opponent_at_pre_pose_does_not_defer_a_pick_for_the_planner():
    w, planner = brain()
    _, rule = brain(StrategyCore)
    st = next(s for s in w.field.stacks if s.label == 'P5')
    sp = planner._spot_for('pick', st)
    set_opp(planner, sp.pre_x, sp.pre_y)
    set_opp(rule, sp.pre_x, sp.pre_y)
    assert planner._pick_goal_blocked(st, sp) is False
    assert rule._pick_goal_blocked(st, sp) is True                       # the rule brain keeps its old behaviour


def test_other_approach_side_is_tried_first():
    w, b = brain()
    st = next(s for s in w.field.stacks if s.label == 'P5')
    assert len(st.approach) == 2
    b.pose = (0.5, 1.0, 0.0)
    first = b._spot_for('pick', st)
    side0 = b._spot_side
    set_opp(b, first.pre_x, first.pre_y)                                 # opponent on that side's pre-pose
    second = b._spot_for('pick', st)
    assert b._spot_side != side0 and (second.x, second.y) != (first.x, first.y)


# ------------------------------------------------------- home replanning
def test_parked_at_home_replans_and_switches_away():
    _, b = brain()
    b.pose = (0.25, 1.0, 0.0)
    o = obs(5.0)
    b.target_kind = 'home'
    b._last_plan_t = -1e9
    assert b._watch_home(o) is True                                      # full table, 80 s left: do something
    assert b._pending is not None and b._pending[0] != 'home'
    assert b.target_kind is None


def test_no_home_replan_when_time_is_short():
    _, b = brain()
    b.pose = (0.25, 1.0, 0.0)
    b.target_kind = 'home'
    b._last_plan_t = -1e9
    o = Observation(83.0, True, 2.0, b.pose, None, -math.pi, 0.0, math.pi, None, False, None)
    assert b._watch_home(o) is False


# ------------------------------------------------------------- scouting
def test_viewpoint_camera_sees_what_it_claims():
    """Every claimed target is in World.camera_view from the viewpoint pose."""
    w = ts.make_world()
    with open(os.path.join(ts.CFG, 'sim.yaml')) as fh:
        sim = yaml.safe_load(fh)
    r = w.robots['us']
    vp = Viewpoints(w.field, 'yellow', sim['camera'], r.model.face_angles, 0.15)
    assert len(vp.positions) > 20
    groups = {
        'bottom edge': ['P2', 'P4', 'P8', 'P10'], 'top edge': ['P1', 'P3', 'P7', 'P9'],
        'centre': ['P5', 'P6'], 'enemy piles': [s.name for s in w.field.stacks if s.steal_zone and w.field.zone(s.steal_zone).team == 'blue'],
    }
    by_label = {s.label: s for s in w.field.stacks}
    n_seen = 0
    for name, labels in groups.items():
        targets = [by_label.get(l) or w.field.stack(l) for l in labels]
        best = vp.best((0.25, 1.0), 0.0, targets, 0.8, time_cost=0.0)
        assert best is not None, name
        x, y, th, seen = best
        r.x, r.y, r.th = x, y, th
        view = w.camera_view('us')
        for s in seen:
            assert s.name in view, (name, s.name, x, y, th)
            n_seen += 1
    assert n_seen >= 8


def test_viewpoints_are_clear_of_obstacles():
    w = ts.make_world()
    vp = Viewpoints(w.field, 'yellow', {}, (0.0, 2.1, 4.2), 0.15)
    import numpy as np
    from table_sim.geometry import closest_point_on_poly
    for (x, y) in vp.positions:
        for s in w.field.stacks:
            if not s.steal_zone:
                assert closest_point_on_poly(np.array([x, y]), np.asarray(s.poly))[1] > 0.15
        assert 0.19 < x < w.field.length - 0.19 and 0.19 < y < w.field.width - 0.19


def test_look_is_not_started_before_the_endgame_and_needs_time():
    _, b = brain()
    b.pose = (1.5, 1.0, 0.0)
    s0 = b._plan_state(obs(10.0, b.pose))
    assert b._maybe_look(obs(10.0, b.pose), s0, [('home',)], b.guard) is None      # before scout_from_s
    late = Observation(75.0, True, 10.0, b.pose, None, -math.pi, 0.0, math.pi, None, False, None)
    s0 = b._plan_state(late)
    assert b._maybe_look(late, s0, [('home',)], b.guard) is None                    # 10 s left: no time for a door


def test_look_at_stale_enemy_piles_once_every_p_is_gone_then_finishes_when_seen():
    w, b = brain()
    t = 45.0
    for st in b.field.stacks:
        if st.steal_zone:
            b.belief._count[st.name], b.belief._last_seen[st.name] = (3.0 if st.steal_zone.startswith('B') else 0.0), 5.0
        else:
            b.belief._count[st.name], b.belief._last_seen[st.name] = 0.0, 30.0       # seen empty
    b.pose = (1.5, 1.0, 0.0)
    o = Observation(t, True, 40.0, b.pose, None, -math.pi, 0.0, math.pi, None, False, None)
    s0 = b._plan_state(o)
    vp = b._maybe_look(o, s0, [('home',)], b.guard)
    assert vp is not None, 'should scout the enemy piles'
    names = b._look['names']
    assert names and all(n.startswith('S_B') for n in names)
    # the camera at the viewpoint really sees them (World geometry)
    r = w.robots['us']
    r.x, r.y, r.th = vp
    assert all(n in w.camera_view('us') for n in names)
    # once the camera has read them, the look ends and the target is cleared
    b.target_kind = 'look'
    cam = {n: 3 for n in names}
    b.belief.on_camera(cam, t + 0.5)
    o2 = Observation(t + 0.5, True, 39.5, vp, None, -math.pi, 0.0, math.pi, cam, False, None)
    b.pose = vp
    cmd = b._look_step(o2, 0.05)
    assert b.target_kind is None and b._look is None and cmd.goal_pose is None


def test_no_look_when_a_full_stack_was_just_seen():
    w, b = brain()
    t = 45.0
    for st in b.field.stacks:
        b.belief._count[st.name], b.belief._last_seen[st.name] = 0.0, 30.0
    b.belief._count['mid_hi'], b.belief._last_seen['mid_hi'] = 3.0, 44.0           # P5 confirmed full
    b.pose = (1.5, 1.0, 0.0)
    o = Observation(t, True, 40.0, b.pose, None, -math.pi, 0.0, math.pi, None, False, None)
    s0 = b._plan_state(o)
    assert b._maybe_look(o, s0, [('home',)], b.guard) is None
