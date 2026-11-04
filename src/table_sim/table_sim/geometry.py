"""Small 2D geometry toolkit (pure python + numpy, no ROS).

Everything is in the map frame, metres / radians. Polygons are (N, 2) numpy
arrays of vertices in counter-clockwise order and are assumed convex.
"""

import math

import numpy as np


def wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def transform(points, x, y, th):
    """Local points (N,2) -> world, given pose (x, y, th)."""
    return np.asarray(points) @ rot(th).T + np.array([x, y])


def rect_poly(cx, cy, lx, ly, th=0.0):
    """Rectangle centred on (cx, cy), size lx * ly along its own axes."""
    hx, hy = lx / 2.0, ly / 2.0
    local = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
    return transform(local, cx, cy, th)


def regular_triangle(side):
    """Equilateral triangle centred on its centroid.

    Returns (vertices, face_normal_angles) with face 0 pointing along +x,
    faces numbered counter-clockwise.
    """
    r_out = side / math.sqrt(3.0)
    # vertices sit between faces: at +60, 180, -60 degrees when face 0 is at 0.
    verts = np.array([[r_out * math.cos(a), r_out * math.sin(a)]
                      for a in (math.radians(60), math.pi, math.radians(-60))])
    # order CCW: 60, 180, 300
    return verts, [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]


def polygon_from_face_angles(side, face_angles):
    """Equilateral triangle whose outward face normals are face_angles."""
    r_out = side / math.sqrt(3.0)
    a0 = face_angles[0]
    verts = [[r_out * math.cos(a0 + k * 2 * math.pi / 3 + math.pi / 3),
              r_out * math.sin(a0 + k * 2 * math.pi / 3 + math.pi / 3)] for k in range(3)]
    return np.array(verts)


def truncated_triangle_apothem(long_edge, short_edge):
    """Apothem (centre -> long-edge midpoint) of polygon_from_truncated_triangle.

    Same shape as cutting a small equilateral triangle of side `short_edge`
    off each 60deg corner of an equilateral triangle of side
    `long_edge + 2 * short_edge` - the remaining middle of each original
    edge is `long_edge` long, each cut is `short_edge` long. The apothem is
    unchanged by the cut (the long edges sit on the original edges' lines).
    """
    return (long_edge + 2.0 * short_edge) / (2.0 * math.sqrt(3.0))


def truncated_triangle_circumradius(long_edge, short_edge):
    """Distance centre -> any of the 6 vertices (all equal, see below)."""
    r_in = truncated_triangle_apothem(long_edge, short_edge)
    return math.hypot(r_in, long_edge / 2.0)


def polygon_from_truncated_triangle(long_edge, short_edge, face_angles):
    """Hexagon: 3 long edges (`long_edge`, outward normals `face_angles`)
    alternating with 3 short edges (`short_edge`) - an equilateral triangle
    with each 60deg corner symmetrically cut off. By the 60deg-corner/
    symmetric-cut geometry, all 6 vertices sit at the same distance from the
    centre (truncated_triangle_circumradius), so sorting them by angle
    around the centre is enough to get a valid CCW polygon regardless of
    face_angles' order.
    """
    r_in = truncated_triangle_apothem(long_edge, short_edge)
    pts = []
    for a in face_angles:
        nx, ny = math.cos(a), math.sin(a)
        tx, ty = -math.sin(a), math.cos(a)
        mx, my = r_in * nx, r_in * ny
        half = long_edge / 2.0
        pts.append((mx - half * tx, my - half * ty))
        pts.append((mx + half * tx, my + half * ty))
    pts.sort(key=lambda p: math.atan2(p[1], p[0]))
    return np.array(pts)


def point_in_poly(p, poly):
    """True if point p is inside convex CCW polygon (boundary counts)."""
    n = len(poly)
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < -1e-12:
            return False
    return True


def poly_inside_poly(inner, outer):
    return all(point_in_poly(v, outer) for v in inner)


def _axes(poly):
    edges = np.roll(poly, -1, axis=0) - poly
    normals = np.stack([edges[:, 1], -edges[:, 0]], axis=1)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    return normals / np.maximum(norms, 1e-12)


def sat_mtv(a, b):
    """Separating-axis test for two convex polygons.

    Returns None if they do not overlap, else (normal, depth) where moving
    polygon `a` by normal * depth separates it from `b`.
    """
    best_depth = float('inf')
    best_axis = None
    for axis in np.vstack([_axes(a), _axes(b)]):
        pa = a @ axis
        pb = b @ axis
        overlap = min(pa.max(), pb.max()) - max(pa.min(), pb.min())
        if overlap <= 0.0:
            return None
        if overlap < best_depth:
            best_depth = overlap
            best_axis = axis
    # orient the axis from b towards a
    if (a.mean(axis=0) - b.mean(axis=0)) @ best_axis < 0.0:
        best_axis = -best_axis
    return best_axis, best_depth


def closest_point_on_poly(p, poly):
    """Closest point to p on the boundary of poly, and its distance."""
    best = None
    best_d = float('inf')
    n = len(poly)
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        ab = b - a
        t = np.clip(((p - a) @ ab) / max(ab @ ab, 1e-12), 0.0, 1.0)
        q = a + t * ab
        d = float(np.linalg.norm(p - q))
        if d < best_d:
            best_d, best = d, q
    return best, best_d


def poly_segments(poly):
    """(M, 2, 2) array of the polygon edges."""
    return np.stack([poly, np.roll(poly, -1, axis=0)], axis=1)


def raycast(origin, angles, segments, max_range):
    """Vectorised ray / segment intersection.

    origin: (2,), angles: (N,) world angles, segments: (M, 2, 2).
    Returns (N,) distances, inf where nothing is hit within max_range.
    """
    n = len(angles)
    out = np.full(n, np.inf)
    if len(segments) == 0:
        return out
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)          # (N,2)
    p = segments[:, 0, :]                                             # (M,2)
    s = segments[:, 1, :] - segments[:, 0, :]                         # (M,2)
    q = p - origin                                                     # (M,2)
    denom = d[:, None, 0] * s[None, :, 1] - d[:, None, 1] * s[None, :, 0]   # (N,M)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (q[None, :, 0] * s[None, :, 1] - q[None, :, 1] * s[None, :, 0]) / denom
        u = (q[None, :, 0] * d[:, None, 1] - q[None, :, 1] * d[:, None, 0]) / denom
    valid = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0) & (t <= max_range)
    t = np.where(valid, t, np.inf)
    return t.min(axis=1)
