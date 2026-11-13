"""Scoring unit tests: drive World._update_action directly (no ROS, no physics).

Run: PYTHONPATH=src/table_sim python3 -m pytest src/table_sim/test/test_scoring.py
"""
import os
from types import SimpleNamespace

import yaml

from table_sim.field import Field
from table_sim.world import World, DROP

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, '..', 'config')


def make_world():
    field = Field(os.path.join(CFG, 'field_2027.yaml'))
    field.zone_max_boxes = 6        # test the extras rule itself, whatever the extra_boxes switch says
    with open(os.path.join(CFG, 'sim.yaml')) as fh:
        sim_cfg = yaml.safe_load(fh)
    return World(field, sim_cfg, 'yellow', seed=0)


def drop(w, zone, n, vertical=False, slot=None):
    """Put n boxes on face 0 and run a drop into `zone` as if the face was above it."""
    r = w.robots['us']
    r.faces[0] = n
    r.vertical[0] = vertical
    r.vertical_slots[0] = [vertical and i < n for i in range(3)]
    return drop_raw(w, zone, 0, slot)


def drop_raw(w, zone, face, slot=None, offset=(0.0, 0.0), dth=0.0):
    """Run a drop from `face` with whatever it currently holds."""
    r = w.robots['us']
    if slot is None and r.vertical[face]:
        slot = next((i for i in range(3) if r.vertical_slots[face][i]), None)
    w.zone_under_face = lambda rr, f: (zone, slot)
    zs = w.field.zone(zone)
    w._face_drop_pose = lambda rr, f: (zs.x + offset[0], zs.y + offset[1], zs.th + dth)   # centred by default
    w._face_drop_pose_slot = lambda rr, f, s: (0, 0, 0)
    r.action = SimpleNamespace(mode=DROP, face=face, done=False, t_start=-10.0, duration=0.0,
                               phase='', count=0, attached=False, engage=0.0, target=None)
    w._update_action(r)
    return w.pop_result('us')


def test_off_centre_drop_is_refused():
    w = make_world(); rect, _ = zones(w)
    r = w.robots['us']; r.faces[0] = 3
    res = drop_raw(w, rect[0], 0, offset=(0.0, 0.12))          # 12 cm across: too far
    assert not res.success and 'not centred' in res.message and w.zones[rect[0]] == 0
    res = drop_raw(w, rect[0], 0, dth=0.5)                     # ~29 deg off
    assert not res.success
    res = drop_raw(w, rect[0], 0, offset=(0.03, 0.02), dth=0.1)  # within tolerance
    assert res.success


def test_extras_switch_off():
    w = make_world(); rect, _ = zones(w)
    w.field.zone_max_boxes = w.field.zone_capacity        # what extra_boxes: false does
    drop(w, rect[0], 3); res = drop(w, rect[0], 1)
    assert not res.success and pts(w) == 8


def pts(w):
    """Construction points only (robot starts parked, so drop the end-zone bonus)."""
    s = w.score('yellow')
    return s['total'] - (w.field.scoring['robot_in_end_zone'] if s['end_zone'] == 'full' else 0)


def zones(w):
    rect = [z.name for z in w.field.team_zones('yellow') if z.shape == 'rect']
    circ = [z.name for z in w.field.team_zones('yellow') if z.shape == 'circle']
    return rect, circ


def test_wall_is_8():
    w = make_world(); rect, _ = zones(w)
    drop(w, rect[0], 3)
    assert pts(w) == 8 and w.score('yellow')['walls'] == 1


def test_wall_plus_extra_is_9():
    w = make_world(); rect, _ = zones(w)
    drop(w, rect[0], 3); drop(w, rect[0], 1)
    assert pts(w) == 9 and w.score('yellow')['walls'] == 1


def test_second_stack_in_same_zone_is_plus_one_per_box():
    w = make_world(); rect, _ = zones(w)
    drop(w, rect[0], 3); drop(w, rect[0], 3)
    assert pts(w) == 8 + 3


def test_tower_is_6():
    w = make_world(); _, circ = zones(w)
    drop(w, circ[0], 1, vertical=True, slot=0)
    s = w.score('yellow')
    assert pts(w) == 6 and s['towers'] == 1


def test_door_is_23_and_extra_is_24():
    w = make_world(); rect, _ = zones(w)
    drop(w, rect[0], 2, vertical=True)          # both posts
    r = w.robots['us']; r.vertical[0] = False   # un-flipped lid
    drop(w, rect[0], 1)
    s = w.score('yellow')
    assert pts(w) == 23 and s['doors'] == 1 and r.door_used
    w.field.door_accepts_extras = True
    drop(w, rect[0], 1)
    assert pts(w) == 24
    w.field.door_accepts_extras = False
    drop(w, rect[0], 1)                       # a finished door takes nothing more
    assert pts(w) == 24


def test_partial_then_fill_makes_wall():
    w = make_world(); rect, _ = zones(w)
    drop(w, rect[0], 2); assert pts(w) == 2
    drop(w, rect[0], 1); assert pts(w) == 8


if __name__ == '__main__':
    for k, v in list(globals().items()):
        if k.startswith('test_'):
            v(); print('ok', k)
