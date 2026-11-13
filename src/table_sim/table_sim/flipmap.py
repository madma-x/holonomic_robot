"""Where can the flip face flip? (no ROS)

World's rule (World._flip_box_in_castle): the boxes held on the flip face must be
fully inside our castle polygon for the whole flip - the robot centre itself may
be outside. FlipMap precomputes every robot pose (grid x heading) that satisfies
it, so the planner can price "go flip near here" and the brain can find the
nearest pose to flip from.
"""
import math

import numpy as np


def _points_in_polygon(px, py, poly):
    """Vectorised even-odd test (any winding, non-convex ok)."""
    inside = np.zeros(px.shape, bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cross = (y1 > py) != (y2 > py)
        with np.errstate(divide='ignore', invalid='ignore'):
            xint = (x2 - x1) * (py - y1) / (y2 - y1) + x1
        inside ^= cross & (px < xint)
    return inside


class FlipMap:
    def __init__(self, field, team, r_in, face_angle=0.0, res=0.02, n_head=72, margin=0.003,
                 pos_tol=0.03, yaw_tol=math.radians(10)):
        self.field, self.team = field, team
        self.r_in, self.face_angle = float(r_in), float(face_angle)
        bw = field.box_width
        # held stack footprint, horizontal (box_length) or vertical (3 slots of box_width): the larger
        self.along = max(field.box_length, 3 * bw)
        self.depth = bw
        self.off = self.r_in + bw / 2 + margin          # robot centre -> held stack centre
        poly = field.castle.get(team)
        self.poly = None if poly is None or len(poly) == 0 else np.asarray(poly, float)
        self.poses = np.zeros((0, 3))
        if self.poly is None:
            return
        lo = self.poly.min(axis=0) - (self.off + 0.3)
        hi = self.poly.max(axis=0) + (self.off + 0.3)
        xs = np.arange(max(lo[0], 0.0), min(hi[0], field.length), res)
        ys = np.arange(max(lo[1], 0.0), min(hi[1], field.width), res)
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        hs = np.linspace(-math.pi, math.pi, n_head, endpoint=False)
        ok = np.zeros((len(xs), len(ys), n_head), bool)
        for k, th in enumerate(hs):
            m = np.ones(X.shape, bool)
            for cx, cy in self._corners_local(th):
                m &= _points_in_polygon(X + cx, Y + cy, self.poly)
            ok[:, :, k] = m
        # Robust poses only: still valid with +-pos_tol in x/y and +-yaw_tol in heading, so a robot
        # that stops a little off the requested pose can still flip there.
        rob = ok.copy()
        kh = max(1, int(round(yaw_tol / (2 * math.pi / n_head))))
        for s_ in range(1, kh + 1):
            rob &= np.roll(ok, s_, axis=2) & np.roll(ok, -s_, axis=2)
        kp = max(1, int(round(pos_tol / res)))
        for ax in (0, 1):
            for s_ in range(1, kp + 1):
                for sh in (s_, -s_):
                    r = np.roll(rob, sh, axis=ax)
                    edge = [slice(None)] * 3
                    edge[ax] = slice(0, sh) if sh > 0 else slice(sh, None)
                    r[tuple(edge)] = False
                    rob &= r
        ii, jj, kk = np.nonzero(rob)
        self.poses = np.column_stack([xs[ii], ys[jj], hs[kk]])

    def _corners_local(self, th):
        """Held-stack corners relative to the robot centre, robot heading th."""
        a = th + self.face_angle
        n = np.array([math.cos(a), math.sin(a)])
        t = np.array([-n[1], n[0]])
        c = n * self.off
        h, d = self.along / 2, self.depth / 2
        return [c + t * sx * h + n * sy * d for sx in (-1, 1) for sy in (-1, 1)]

    def ok_at(self, x, y, th):
        """Exact: may the flip face flip at this robot pose?"""
        if self.poly is None:
            return True
        cs = np.array(self._corners_local(th)) + np.array([x, y])
        return bool(_points_in_polygon(cs[:, 0], cs[:, 1], self.poly).all())

    def nearest(self, x, y, heading=None, turn_w=0.15):
        """(distance, (x, y, th)) of the closest pose that can flip, or (inf, None).
        With `heading`, the heading error costs `turn_w` m/rad on top of the distance, so the robot
        flips already facing where it drops next instead of spinning a loaded face afterwards next
        to the built piles (180 deg ~ 0.47 m)."""
        if self.poly is None:
            return 0.0, (x, y, 0.0 if heading is None else heading)
        if not len(self.poses):
            return math.inf, None
        d = np.hypot(self.poses[:, 0] - x, self.poses[:, 1] - y)
        cost = d
        if heading is not None:
            dth = np.abs((self.poses[:, 2] - heading + math.pi) % (2 * math.pi) - math.pi)
            cost = d + turn_w * dth
        i = int(np.argmin(cost))
        return float(d[i]), tuple(float(v) for v in self.poses[i])

    def best_between(self, a, b, heading=None):
        """Pose minimising |a -> pose| + |pose -> b| (a detour on the way from a to b).
        With `heading`, poses within 30 deg of it win ties (less turning)."""
        if self.poly is None:
            return (a[0], a[1], heading or 0.0)
        if not len(self.poses):
            return None
        P = self.poses
        cost = np.hypot(P[:, 0] - a[0], P[:, 1] - a[1]) + np.hypot(P[:, 0] - b[0], P[:, 1] - b[1])
        if heading is not None:
            dth = np.abs((P[:, 2] - heading + math.pi) % (2 * math.pi) - math.pi)
            cost = cost + 0.05 * dth
        i = int(np.argmin(cost))
        return tuple(float(v) for v in P[i])
