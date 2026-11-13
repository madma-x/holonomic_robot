"""Tiny grid A* planner for a holonomic robot on the table (no ROS)."""

import heapq
import math

import numpy as np

from .geometry import point_in_poly


class GridPlanner:
    def __init__(self, length, width, res=0.04, wall_margin=0.0):
        self.res = res
        self.nx = int(math.ceil(length / res))
        self.ny = int(math.ceil(width / res))
        xs = (np.arange(self.nx) + 0.5) * res
        ys = (np.arange(self.ny) + 0.5) * res
        self.cx, self.cy = np.meshgrid(xs, ys, indexing='ij')     # (nx, ny)
        self.length, self.width = length, width
        self.wall_margin = wall_margin
        self._static_key = None
        self._static = None

    # ---------------------------------------------------------- occupancy
    def _poly_mask(self, poly, radius):
        """Cells whose centre is within `radius` of the polygon."""
        xmin, ymin = poly.min(axis=0) - radius
        xmax, ymax = poly.max(axis=0) + radius
        i0, i1 = max(0, int(xmin / self.res)), min(self.nx, int(xmax / self.res) + 1)
        j0, j1 = max(0, int(ymin / self.res)), min(self.ny, int(ymax / self.res) + 1)
        mask = np.zeros((self.nx, self.ny), bool)
        if i0 >= i1 or j0 >= j1:
            return mask
        px = self.cx[i0:i1, j0:j1]
        py = self.cy[i0:i1, j0:j1]
        inside = np.ones_like(px, bool)
        dmin = np.full(px.shape, np.inf)
        n = len(poly)
        for k in range(n):
            a, b = poly[k], poly[(k + 1) % n]
            ex, ey = b - a
            inside &= (ex * (py - a[1]) - ey * (px - a[0])) >= 0
            t = np.clip(((px - a[0]) * ex + (py - a[1]) * ey) / (ex * ex + ey * ey), 0, 1)
            d = np.hypot(px - (a[0] + t * ex), py - (a[1] + t * ey))
            dmin = np.minimum(dmin, d)
        mask[i0:i1, j0:j1] = inside | (dmin <= radius)
        return mask

    def set_static(self, key, polys, radius, keepout=(), keepout_radius=0.08):
        """Static obstacles (cached on `key`). `keepout` polygons are forbidden areas inflated by their
        own (small) radius: the rule is about the robot's centre, not its loaded footprint."""
        if key == self._static_key:
            return
        occ = np.zeros((self.nx, self.ny), bool)
        m = radius + self.wall_margin
        occ |= self.cx < m
        occ |= self.cx > self.length - m
        occ |= self.cy < m
        occ |= self.cy > self.width - m
        for p in polys:
            occ |= self._poly_mask(p, radius)
        for p in keepout:
            occ |= self._poly_mask(p, keepout_radius)
        self._static, self._static_key = occ, key

    def occupancy(self, discs=()):
        occ = self._static.copy()
        for (x, y, r) in discs:
            occ |= (self.cx - x) ** 2 + (self.cy - y) ** 2 <= r * r
        return occ

    # ------------------------------------------------------------ search
    def cell(self, p):
        return (min(self.nx - 1, max(0, int(p[0] / self.res))),
                min(self.ny - 1, max(0, int(p[1] / self.res))))

    def center(self, c):
        return ((c[0] + 0.5) * self.res, (c[1] + 0.5) * self.res)

    def _component(self, occ, c):
        """Boolean mask of the free cells 4-connected to free cell c."""
        seen = np.zeros_like(occ)
        seen[c] = True
        stack = [c]
        while stack:
            i, j = stack.pop()
            for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if 0 <= n[0] < self.nx and 0 <= n[1] < self.ny and not occ[n] and not seen[n]:
                    seen[n] = True
                    stack.append(n)
        return seen

    def _nearest_free(self, occ, c, max_r=15, allowed=None):
        """Nearest free cell to c (c itself if free). `allowed`: optional mask the cell must be in -
        used so a start swallowed by an inflated obstacle snaps to the goal's region, not to an
        isolated free pocket next to it."""
        if not occ[c]:
            return c
        for r in range(1, max_r):
            best = None
            for i in range(c[0] - r, c[0] + r + 1):
                for j in (c[1] - r, c[1] + r):
                    if 0 <= i < self.nx and 0 <= j < self.ny and not occ[i, j] \
                            and (allowed is None or allowed[i, j]):
                        d = (i - c[0]) ** 2 + (j - c[1]) ** 2
                        if best is None or d < best[0]:
                            best = (d, (i, j))
            for j in range(c[1] - r + 1, c[1] + r):
                for i in (c[0] - r, c[0] + r):
                    if 0 <= i < self.nx and 0 <= j < self.ny and not occ[i, j] \
                            and (allowed is None or allowed[i, j]):
                        d = (i - c[0]) ** 2 + (j - c[1]) ** 2
                        if best is None or d < best[0]:
                            best = (d, (i, j))
            if best:
                return best[1]
        return None

    def line_free(self, occ, p, q):
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        n = max(2, int(d / (self.res * 0.5)))
        for k in range(n + 1):
            t = k / n
            c = self.cell((p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t))
            if occ[c]:
                return False
        return True

    def plan(self, start, goal, discs=(), occ=None):
        """Returns (waypoints, length) or (None, inf). Waypoints exclude start."""
        if occ is None:
            occ = self.occupancy(discs)
        if self.line_free(occ, start, goal):
            return [tuple(goal)], math.hypot(goal[0] - start[0], goal[1] - start[1])
        g = self._nearest_free(occ, self.cell(goal))
        if g is None:
            return None, math.inf
        s = self.cell(start)
        if occ[s]:      # start inside an inflated obstacle (just dropped beside a pile): escape toward the goal's region
            s = self._nearest_free(occ, s, allowed=self._component(occ, g))
        if s is None:
            return None, math.inf
        openq = [(0.0, 0.0, s)]
        came = {s: None}
        cost = {s: 0.0}
        sq2 = math.sqrt(2.0)
        nbrs = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, sq2), (1, -1, sq2), (-1, 1, sq2), (-1, -1, sq2)]
        found = False
        while openq:
            _, gc, c = heapq.heappop(openq)
            if c == g:
                found = True
                break
            if gc > cost[c] + 1e-9:
                continue
            for di, dj, w in nbrs:
                n = (c[0] + di, c[1] + dj)
                if not (0 <= n[0] < self.nx and 0 <= n[1] < self.ny) or occ[n]:
                    continue
                if di and dj and (occ[c[0] + di, c[1]] or occ[c[0], c[1] + dj]):
                    continue
                ng = gc + w
                if ng < cost.get(n, math.inf):
                    cost[n] = ng
                    came[n] = c
                    h = math.hypot(g[0] - n[0], g[1] - n[1])
                    heapq.heappush(openq, (ng + h, ng, n))
        if not found:
            return None, math.inf
        cells = []
        c = g
        while c is not None:
            cells.append(c)
            c = came[c]
        cells.reverse()
        pts = [tuple(start)] + [self.center(c) for c in cells[1:-1]] + [tuple(goal)]
        # line-of-sight shortcutting
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.line_free(occ, pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        length = sum(math.hypot(out[k + 1][0] - out[k][0], out[k + 1][1] - out[k][1])
                     for k in range(len(out) - 1))
        return out[1:], length


def path_length_estimate(planner, start, goal, discs=()):
    _, length = planner.plan(start, goal, discs)
    return length


__all__ = ['GridPlanner', 'point_in_poly']
