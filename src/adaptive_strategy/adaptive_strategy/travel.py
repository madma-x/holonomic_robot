"""Cached travel-time matrix between strategic poses (no ROS).

Nodes: 'home', 'S:<stack name>' (pick spot), 'Z:<zone name>' (drop dock).
Distances come from gridplan A* (stacks as obstacles), converted to seconds
with a trapezoid velocity profile. Computed lazily, once per ordered pair.
"""
import json
import math
import os

from table_sim.gridplan import GridPlanner


def trapezoid_time(d, vmax, acc):
    if d <= 1e-6:
        return 0.0
    d_ramp = vmax * vmax / acc
    if d >= d_ramp:
        return d / vmax + vmax / acc
    return 2.0 * math.sqrt(d / acc)


def node_poses(field, team, r_in, dock_poses_file=None):
    """{node: (x, y)} for home, every stack pick spot and every own-team drop dock."""
    nodes = {'home': tuple(field.start_pose(team)[:2])}
    for s in field.stacks:
        sp = field.pick_spots(s, r_in)[0]
        nodes['S:' + s.name] = (sp.x, sp.y)
    docks = {}
    if dock_poses_file and os.path.exists(dock_poses_file):
        with open(dock_poses_file) as f:
            for k, v in json.load(f).get('poses', {}).items():
                base, _, pos = k.rpartition('_')
                label = base if (base and pos == 'C') else k
                if '_' in k and pos != 'C':
                    continue
                docks[label] = (float(v['x']), float(v['y']))
    for z in field.team_zones(team):
        if z.dock is not None:
            nodes['Z:' + z.name] = (z.dock[0], z.dock[1])
        elif z.label in docks and team == 'yellow':
            nodes['Z:' + z.name] = docks[z.label]
        else:
            sp = field.drop_spots(z, r_in)[0]
            nodes['Z:' + z.name] = (sp.x, sp.y)
    return nodes


class TravelMatrix:
    def __init__(self, field, team, robot_cfg, r_in, dock_poses_file=None, res=0.04):
        self.nodes = node_poses(field, team, r_in, dock_poses_file)
        self.vmax = float(robot_cfg.get('max_lin_vel', 1.0)) * 0.8   # leave margin for Nav2
        self.acc = float(robot_cfg.get('max_lin_acc', 1.5))
        self.planner = GridPlanner(field.length, field.width, res=res)
        polys = [s.poly for s in field.stacks if not s.steal_zone]    # drop-zone piles are not fixed obstacles
        ko = field.keepout_poly(team)          # the opponent's castle: forbidden
        sko = field.start_keepout_poly(team)   # the opponent's start zone: forbidden at all times
        self.planner.set_static('plan', polys, r_in + 0.02, keepout=([ko] if len(ko) else []) + [sko])
        # picking from an enemy drop zone is allowed: those legs ignore the castle keep-out (not the start zone)
        self.planner_free = GridPlanner(field.length, field.width, res=res)
        self.planner_free.set_static('plan', polys, r_in + 0.02, keepout=[sko])
        self._steal = {'S:' + s.name for s in field.stacks if s.steal_zone}
        self._cache = {}

    def time(self, a, b):
        if a == b:
            return 0.0
        k = (a, b)
        t = self._cache.get(k)
        if t is None:
            pl = self.planner_free if (a in self._steal or b in self._steal) else self.planner
            _, d = pl.plan(self.nodes[a], self.nodes[b])
            if not math.isfinite(d):
                d = math.dist(self.nodes[a], self.nodes[b]) * 1.5
            t = trapezoid_time(d, self.vmax, self.acc)
            self._cache[k] = t
        return t

    def set_node(self, name, xy):
        """(Re)define a movable node (e.g. 'cur' = the robot's pose); drops its cache."""
        self.nodes[name] = (float(xy[0]), float(xy[1]))
        for k in [k for k in self._cache if name in k]:
            del self._cache[k]
