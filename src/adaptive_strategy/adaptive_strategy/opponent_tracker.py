"""Turns raw mast-lidar returns into a single opponent position estimate.

No ground truth is ever read here - only (ranges, angle_min, angle_inc,
lidar_yaw) from the same Observation the brain itself receives, exactly
mirroring what a live /scan subscription gives the real robot. The mast
reflects a small cluster of hits (~7cm wide); we average them to one point
and track a simple decaying-confidence belief of where the opponent is
between sightings, since the lidar only reports a detection when the
opponent happens to be within range (world.py's detect_range, ~2.8m).
"""

import math


class OpponentEstimate:
    __slots__ = ('x', 'y', 'sigma', 'visible', 'seen_ever', 'vx', 'vy')

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.sigma = float('inf')   # position uncertainty (m), grows when unseen
        self.visible = False        # detected on the most recent scan
        self.seen_ever = False
        self.vx = 0.0                # smoothed world-frame velocity (m/s),
        self.vy = 0.0                # held (not decayed) between sightings -
        #                              only ever trust it alongside sigma.

    def as_dict(self):
        return {'x': self.x, 'y': self.y, 'sigma': self.sigma,
                'visible': self.visible, 'seen_ever': self.seen_ever,
                'vx': self.vx, 'vy': self.vy}


class OpponentTracker:
    def __init__(self, sigma_growth=0.3, sigma_init=0.05, sigma_max=5.0, vel_smoothing=0.5):
        self.sigma_growth = sigma_growth   # m of added uncertainty per second unseen
        self.sigma_init = sigma_init
        self.sigma_max = sigma_max
        self.vel_smoothing = vel_smoothing  # 0..1, higher = trust the newest sample more
        self.opp = OpponentEstimate()
        self.last_points = []   # world-frame scan hits from the latest scan, for debug
        self._dt_since_sighting = 0.0

    def update(self, obs, dt):
        """Call every tick. `obs.ranges` is None on ticks without a fresh scan."""
        self._dt_since_sighting += dt
        if obs.ranges is None:
            if self.opp.seen_ever:
                self.opp.sigma = min(self.sigma_max, self.opp.sigma + self.sigma_growth * dt)
            self.opp.visible = False
            return self.opp

        x0, y0, th = obs.pose
        pts = []
        for i, r in enumerate(obs.ranges):
            if not math.isfinite(r):
                continue
            world_angle = th + obs.lidar_yaw + obs.angle_min + i * obs.angle_inc
            pts.append((x0 + r * math.cos(world_angle), y0 + r * math.sin(world_angle)))
        self.last_points = pts

        if not pts:
            if self.opp.seen_ever:
                self.opp.sigma = min(self.sigma_max, self.opp.sigma + self.sigma_growth * dt)
            self.opp.visible = False
            return self.opp

        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        # Velocity: finite difference against the last sighting (not the last
        # tick - the gap between two actual detections can span several
        # no-scan ticks), exponentially smoothed so one noisy mast-cluster
        # centroid doesn't swing the apparent heading wildly.
        if self.opp.seen_ever and self._dt_since_sighting > 1e-3:
            raw_vx = (cx - self.opp.x) / self._dt_since_sighting
            raw_vy = (cy - self.opp.y) / self._dt_since_sighting
            a = self.vel_smoothing
            self.opp.vx = a * raw_vx + (1 - a) * self.opp.vx
            self.opp.vy = a * raw_vy + (1 - a) * self.opp.vy
        self.opp.x, self.opp.y = cx, cy
        self.opp.sigma = self.sigma_init
        self.opp.visible = True
        self.opp.seen_ever = True
        self._dt_since_sighting = 0.0
        return self.opp
