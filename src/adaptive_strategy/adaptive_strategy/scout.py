"""Scouting viewpoints for the door endgame (no ROS).

A viewpoint is a robot pose (x, y, th) from which the face-mounted camera sees a group of stacks /
enemy drop-zone piles. The geometry is World.camera_view's (distance <= max_range, bearing within
hfov/2 of the camera face), shrunk by a safety margin so a few cm / degrees of docking error do not
lose a target. Positions are sampled on a regular grid over the free table (clear of walls, stacks,
drop zones and the opponent castle); the yaw is chosen per query so one viewpoint covers as many of the
wanted targets as possible - deterministic, no tuning file.
"""
import math

import numpy as np

from table_sim.field import _point_in_polygon
from table_sim.geometry import closest_point_on_poly


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Viewpoints:
    def __init__(self, field, team, camera_cfg=None, face_angles=(0.0,), r_out=0.15,
                 grid=0.15, wall_margin=0.20, obstacle_margin=0.04, n_yaw=24,
                 range_margin=0.85, fov_margin=0.90):
        c = dict(camera_cfg or {})
        self.field, self.team = field, team
        self.hfov = math.radians(float(c.get('hfov_deg', 160.0)))
        self.max_range = float(c.get('max_range', 2.5))
        face = int(c.get('face', 0))
        self.cam_angle = float(face_angles[face % len(face_angles)])
        self.range_margin, self.fov_margin = range_margin, fov_margin
        self.n_yaw = n_yaw
        self.positions = self._free_positions(grid, wall_margin, r_out + obstacle_margin)

    # ------------------------------------------------------------ geometry
    def sees(self, x, y, th, obj):
        """World.camera_view's test on obj's centre, with margins."""
        dx, dy = obj.x - x, obj.y - y
        if math.hypot(dx, dy) > self.max_range * self.range_margin:
            return False
        return abs(_wrap(math.atan2(dy, dx) - (th + self.cam_angle))) <= self.hfov / 2 * self.fov_margin

    def _free_positions(self, grid, wall, clear):
        f = self.field
        polys = [np.asarray(s.poly) for s in f.stacks if not s.steal_zone]
        polys += [np.asarray(z.poly) for z in f.zones]
        ko = f.keepout_poly(self.team)          # the opponent's castle is forbidden; our own is fine to stand in
        if len(ko):
            polys.append(np.asarray(ko))
        polys.append(f.start_keepout_poly(self.team))   # the opponent's start zone: forbidden at all times
        out = []
        nx, ny = int((f.length - 2 * wall) / grid) + 1, int((f.width - 2 * wall) / grid) + 1
        for i in range(nx):
            for j in range(ny):
                x, y = wall + i * grid, wall + j * grid
                p = np.array([x, y])
                if any(_point_in_polygon((x, y), poly) or closest_point_on_poly(p, poly)[1] < clear
                       for poly in polys):
                    continue
                out.append((x, y))
        return out

    def best_yaw(self, x, y, targets, prefer_th=None):
        """(th, [targets seen]) maximising the number seen from (x, y); ties -> the smallest turn
        from prefer_th (if given), then the lowest heading (deterministic)."""
        best, best_key = (0.0, []), None
        for k in range(self.n_yaw):
            th = _wrap(2 * math.pi * k / self.n_yaw)
            seen = [t for t in targets if self.sees(x, y, th, t)]
            turn = abs(_wrap(th - prefer_th)) if prefer_th is not None else 0.0
            key = (-len(seen), round(turn, 3), th)
            if best_key is None or key < best_key:
                best_key, best = key, (th, seen)
        return best

    def best(self, cur_xy, cur_th, targets, vmax, time_cost=0.4, avoid=None, avoid_r=0.7, min_cover=1):
        """Best viewpoint for `targets`: maximise (#seen - time_cost * straight-line eta). Returns
        (x, y, th, [seen targets]) or None. `avoid` = (x, y) of the opponent to keep clear of."""
        best, best_score = None, -1e9
        for (x, y) in self.positions:
            if avoid is not None and math.hypot(x - avoid[0], y - avoid[1]) < avoid_r:
                continue
            th, seen = self.best_yaw(x, y, targets, cur_th)
            if len(seen) < min_cover:
                continue
            eta = math.hypot(x - cur_xy[0], y - cur_xy[1]) / max(vmax, 1e-3)
            score = len(seen) - time_cost * eta
            if score > best_score + 1e-9:
                best_score, best = score, (x, y, th, seen)
        return best
