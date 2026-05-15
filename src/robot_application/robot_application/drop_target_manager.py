"""Drop target manager for offset-aware base goal selection."""

import math
from typing import Any, Dict, List, Optional, Tuple


class DropTargetManager:
    """Compute feasible base goals for placing the arm over a drop target.

    Candidates are sampled on a ring of radius `drop_target_ring_radius_m`
    around the drop target. Each candidate heading points toward the target so
    the arm (offset `drop_target_arm_x_offset_m` along +X) lands as close as
    possible to it. Candidates whose arm tip would miss the target by more than
    `drop_target_arm_alignment_tolerance_m` are rejected. Among the remaining
    feasible candidates the one whose heading best matches the drop target
    theta is preferred; arm alignment error and path length are used as
    tiebreaks.
    """

    def __init__(self, node):
        self.node = node

        self.node.declare_parameter('drop_target_manager_enabled', False)
        self.node.declare_parameter('drop_target_arm_x_offset_m', 0.15)
        self.node.declare_parameter('drop_target_ring_radius_m', 0.15)
        self.node.declare_parameter('drop_target_ring_samples', 8)
        self.node.declare_parameter('drop_target_arm_alignment_tolerance_m', 0.02)
        self.node.declare_parameter('drop_target_planner_id', 'GridBased')

    def _is_enabled(self) -> bool:
        return bool(self.node.get_parameter('drop_target_manager_enabled').value)

    def select_navigation_goal(
        self,
        drop_id: str,
        drop_location: dict,
    ) -> Tuple[Optional[dict], Dict[str, Any]]:
        """Return the best feasible base pose for this drop location.

        'Best' means closest heading to the drop target theta, then smallest
        arm alignment error (closest to a perfect drop), with shortest Nav2
        path as a tiebreak.
        """
        normalized = self._normalize_location(drop_location)
        diagnostics: Dict[str, Any] = {
            'drop_id': drop_id,
            'tested_candidates': 0,
            'feasible_candidates': 0,
            'reject_reasons': {},
        }

        if not self._is_enabled():
            return normalized, diagnostics

        self.node.get_logger().info(
            "Drop target '%s' selected pose: x=%.3f, y=%.3f, theta=%.3f. Starting goal pose search."
            % (
                drop_id,
                float(normalized['x']),
                float(normalized['y']),
                float(normalized['theta']),
            )
        )

        planner_id = str(self.node.get_parameter('drop_target_planner_id').value)
        arm_alignment_tol = float(
            self.node.get_parameter('drop_target_arm_alignment_tolerance_m').value
        )

        candidates = self._generate_ring_candidates(normalized)
        if not candidates:
            self._count_reject(diagnostics, 'no_candidates')
            return None, diagnostics

        feasible: List[dict] = []
        for candidate in candidates:
            diagnostics['tested_candidates'] += 1

            if float(candidate['arm_alignment_error']) > arm_alignment_tol:
                self._count_reject(diagnostics, 'arm_alignment')
                continue

            ok, path_length, reason = self.node.compute_path_to_pose(
                float(candidate['x']),
                float(candidate['y']),
                float(candidate['theta']),
                planner_id=planner_id,
            )
            if not ok:
                self._count_reject(diagnostics, reason)
                continue

            candidate['path_length_m'] = float(path_length)
            feasible.append(candidate)

        diagnostics['feasible_candidates'] = len(feasible)
        if not feasible:
            return None, diagnostics

        best = min(
            feasible,
            key=lambda c: (
                float(c['theta_error_rad']),
                float(c['arm_alignment_error']),
                float(c['path_length_m']),
            ),
        )

        return {
            'x': float(best['x']),
            'y': float(best['y']),
            'theta': float(normalized['theta']),
        }, diagnostics

    def _normalize_location(self, location: dict) -> dict:
        return {
            'x': float(location.get('x', 0.0)),
            'y': float(location.get('y', 0.0)),
            'theta': float(location.get('theta', 0.0)),
        }

    def _generate_ring_candidates(self, drop_location: dict) -> List[dict]:
        """Sample `drop_target_ring_samples` poses on a ring around the drop target.

        Each pose faces the target. `arm_alignment_error` is the distance between
        the arm tip (base + arm_offset along heading) and the drop target center.
        This equals |ring_radius - arm_offset| for all candidates on a fixed ring,
        and can be reduced by tuning `drop_target_ring_radius_m` ≈ arm_offset.
        """
        radius = float(self.node.get_parameter('drop_target_ring_radius_m').value)
        arm_offset = float(self.node.get_parameter('drop_target_arm_x_offset_m').value)
        sample_count = max(4, int(self.node.get_parameter('drop_target_ring_samples').value))

        x_t = float(drop_location.get('x', 0.0))
        y_t = float(drop_location.get('y', 0.0))
        theta_t = float(drop_location.get('theta', 0.0))

        candidates: List[dict] = []
        for index in range(sample_count):
            angle = (2.0 * math.pi * float(index)) / float(sample_count)
            x_b = x_t - radius * math.cos(angle)
            y_b = y_t - radius * math.sin(angle)
            theta = math.atan2(y_t - y_b, x_t - x_b)
            theta_error_rad = abs(math.atan2(math.sin(theta - theta_t), math.cos(theta - theta_t)))

            arm_x = x_b + arm_offset * math.cos(theta)
            arm_y = y_b + arm_offset * math.sin(theta)
            arm_alignment_error = math.hypot(arm_x - x_t, arm_y - y_t)

            candidates.append({
                'x': x_b,
                'y': y_b,
                'theta': theta,
                'theta_error_rad': theta_error_rad,
                'arm_alignment_error': arm_alignment_error,
            })

        return candidates

    @staticmethod
    def _count_reject(diagnostics: Dict[str, Any], reason: str):
        reject_reasons = diagnostics.setdefault('reject_reasons', {})
        reject_reasons[reason] = int(reject_reasons.get(reason, 0)) + 1

