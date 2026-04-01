#!/usr/bin/env python3
"""
Nut pickup mission.

Picks up rectangular nut objects using vacuum pumps and deposits them in
scoring drop zones.

Sequence for each nut
---------------------
1. Try approaches in priority order (first listed = highest priority).
2. Nav2 to approach offset position.
3. Wait briefly for aruco tags to become visible.
4. If tags detected by the tag manager → run AlignToCluster visual servoing.
5. Activate vacuum pumps (sequence: activate → build-up delay).
6. Nav2 to drop zone robot position (arm offset pre-computed in config).
7. Deactivate pumps to release the nut.
8. Back up from the drop zone.

If a navigation step fails (e.g. enemy blocking), the next approach is tried.
If all approaches for a nut are blocked, that nut is skipped.
"""

import math
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from aruco_interfaces.action import AlignToCluster
from aruco_interfaces.msg import ClusterPickability
from rclpy.action import ActionClient
import yaml

from robot_application.mission_base import MissionBase, MissionState


class NutPickupMission(MissionBase):
    """Mission: pick up nuts and deposit them in drop zones."""

    def __init__(self):
        super().__init__('nut_pickup')

        # AlignToCluster visual-servoing action client
        self.align_client = ActionClient(self, AlignToCluster, 'align_to_cluster')

        # Latest pickability messages keyed by cluster_id
        self._pickability: dict = {}
        self._pickability_sub = self.create_subscription(
            ClusterPickability,
            '/cluster_pickability',
            self._on_pickability,
            10,
        )

        # Mission data (populated by load_mission_config)
        self.nuts: list = []
        self.drop_zones: list = []
        self.arm_offset_m: float = 0.30
        self.num_pumps: int = 4
        self.robot_side: str = 'yellow'
        self.tag_detection_wait_sec: float = 0.5
        self.alignment_threshold_m: float = 0.02
        self.pump_buildup_sec: float = 0.5
        self.backup_dist_m: float = 0.30

    # ── ROS subscription ─────────────────────────────────────────────────────

    def _on_pickability(self, msg: ClusterPickability) -> None:
        self._pickability[msg.cluster_id] = msg

    # ── Configuration loading ─────────────────────────────────────────────────

    def load_mission_config(self) -> None:
        """Load nut pickup configuration from YAML."""
        config_dir = get_package_share_directory('robot_application')
        config_file = os.path.join(config_dir, 'config', 'nut_pickup_config.yaml')

        with open(config_file, 'r') as fh:
            raw = yaml.safe_load(fh)

        cfg = raw['nut_pickup']

        # Allow robot_side override via ROS2 parameter
        self.declare_parameter('robot_side', cfg.get('robot_side', 'yellow'))
        self.robot_side = self.get_parameter('robot_side').value

        self.arm_offset_m = cfg.get('arm_offset_m', 0.30)
        self.num_pumps = cfg.get('num_pumps', 4)
        self.tag_detection_wait_sec = cfg.get('tag_detection_wait_sec', 0.5)
        self.alignment_threshold_m = cfg.get('alignment_threshold_m', 0.02)
        self.pump_buildup_sec = cfg.get('pump_buildup_sec', 0.5)
        self.backup_dist_m = cfg.get('backup_dist_m', 0.30)
        self.nuts = cfg.get('nuts', [])
        self.drop_zones = cfg.get('drop_zones', [])

        self.get_logger().info(
            f'Loaded {len(self.nuts)} nut(s), {len(self.drop_zones)} drop zone(s) '
            f'[side={self.robot_side}]'
        )

    # ── Top-level mission execution ───────────────────────────────────────────

    def execute_mission(self) -> None:
        """Execute the full nut pickup mission."""
        self.get_logger().info('=== Nut pickup mission START ===')
        drop_zone_idx = 0

        try:
            if not self.nuts:
                self.get_logger().warn('No nuts configured – mission complete')
                self.state = MissionState.COMPLETED
                self.progress = 100.0
                return

            if not self.drop_zones:
                self.get_logger().error('No drop zones configured – aborting')
                self.state = MissionState.FAILED
                return

            for nut_idx, nut in enumerate(self.nuts):
                if self.stop_requested:
                    break

                self.progress = (nut_idx / len(self.nuts)) * 100.0
                self.get_logger().info(
                    f'── Nut {nut_idx + 1}/{len(self.nuts)}: {nut["id"]} ──'
                )

                picked = self._execute_nut_pickup(nut)
                if not picked:
                    self.get_logger().warn(
                        f'All approaches blocked for {nut["id"]} – skipping'
                    )
                    continue

                # Deposit into the next available drop zone
                drop_zone = self.drop_zones[drop_zone_idx % len(self.drop_zones)]
                self.get_logger().info(
                    f'Depositing into drop zone: {drop_zone["id"]}'
                )
                dropped = self._execute_drop(drop_zone)
                if not dropped:
                    self.get_logger().error(
                        f'Failed to deposit into {drop_zone["id"]} – '
                        'releasing pumps and aborting mission'
                    )
                    self._release_all_pumps()
                    self.state = MissionState.FAILED
                    return

                drop_zone_idx += 1

            self.get_logger().info('=== Nut pickup mission COMPLETE ===')
            self.state = MissionState.COMPLETED
            self.progress = 100.0

        except Exception as exc:
            self.get_logger().error(f'Unhandled mission error: {exc}')
            self._release_all_pumps()
            self.state = MissionState.FAILED

    # ── Nut pickup sub-sequence ───────────────────────────────────────────────

    def _execute_nut_pickup(self, nut: dict) -> bool:
        """
        Try each approach for *nut* in order.

        Returns True if the nut was successfully picked up (pumps activated).
        Returns False if every approach was blocked or tag-detection failed.
        """
        for approach in nut.get('approaches', []):
            if self.stop_requested:
                return False

            name = approach['name']
            nav_pose = approach['nav2_pose']
            self.get_logger().info(
                f'  Trying approach "{name}": '
                f'nav2 → ({nav_pose["x"]:.3f}, {nav_pose["y"]:.3f}, '
                f'θ={nav_pose["theta"]:.3f})'
            )

            # 1 ── Navigate to approach offset position
            reached = self.navigate_to_pose(
                nav_pose['x'], nav_pose['y'], nav_pose['theta']
            )
            if not reached:
                self.get_logger().warn(
                    f'  Approach "{name}" blocked by obstacle – trying next'
                )
                continue

            # 2 ── Wait for tags to become visible
            time.sleep(self.tag_detection_wait_sec)

            # 3 ── Visual servoing alignment (if tags detected)
            cluster_id = nut.get('cluster_id', -1)
            if cluster_id >= 0:
                tags_visible = cluster_id in self._pickability
                if not tags_visible:
                    self.get_logger().warn(
                        f'  No tags detected for cluster {cluster_id} '
                        f'at approach "{name}" – trying next'
                    )
                    continue

                self.get_logger().info(
                    f'  Tags detected – running visual servoing '
                    f'(cluster {cluster_id})'
                )
                aligned = self._align_to_cluster(cluster_id)
                if not aligned:
                    self.get_logger().warn(
                        f'  Visual servoing failed for cluster {cluster_id} '
                        f'– trying next approach'
                    )
                    continue

            # 4 ── Activate vacuum pumps
            self.get_logger().info('  Activating vacuum pumps')
            if not self._activate_all_pumps():
                self.get_logger().error('  Pump activation failed – trying next approach')
                continue

            self.get_logger().info(f'  ✓ Nut {nut["id"]} picked up')
            return True

        return False

    # ── Drop sub-sequence ─────────────────────────────────────────────────────

    def _execute_drop(self, drop_zone: dict) -> bool:
        """
        Navigate to *drop_zone*, deactivate pumps, then back up.

        Returns True on success.
        """
        for approach in drop_zone.get('approaches', []):
            if self.stop_requested:
                return False

            name = approach['name']
            robot_pose = approach['robot_pose']
            self.get_logger().info(
                f'  Drop approach "{name}": '
                f'nav2 → ({robot_pose["x"]:.3f}, {robot_pose["y"]:.3f}, '
                f'θ={robot_pose["theta"]:.3f})'
            )

            reached = self.navigate_to_pose(
                robot_pose['x'], robot_pose['y'], robot_pose['theta']
            )
            if not reached:
                self.get_logger().warn(
                    f'  Drop approach "{name}" blocked – trying next'
                )
                continue

            # Release nuts
            self.get_logger().info('  Releasing pumps')
            self._release_all_pumps()
            time.sleep(0.3)

            # Back up away from the drop zone
            self.get_logger().info(
                f'  Backing up {self.backup_dist_m:.2f} m'
            )
            bx, by = self._backup_pose(robot_pose, self.backup_dist_m)
            if not self.navigate_to_pose(bx, by, robot_pose['theta']):
                self.get_logger().warn('  Backup navigation failed (non-critical)')

            return True

        return False

    # ── Visual servoing ───────────────────────────────────────────────────────

    def _align_to_cluster(self, cluster_id: int) -> bool:
        """
        Send an AlignToCluster goal and wait for the result.

        Returns True if the server is unavailable (skips alignment gracefully)
        or if alignment succeeded.  Returns False only on explicit failure.
        """
        if not self.align_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn(
                'AlignToCluster server not available – proceeding without '
                'visual servoing'
            )
            return True  # non-fatal: carry on without fine alignment

        goal = AlignToCluster.Goal()
        goal.cluster_id = cluster_id
        goal.alignment_threshold = self.alignment_threshold_m

        send_future = self.align_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f'AlignToCluster goal rejected (cluster {cluster_id})'
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)

        if result_future.result() is None:
            self.get_logger().error(
                f'AlignToCluster timed out (cluster {cluster_id})'
            )
            return False

        result = result_future.result().result
        if not result.success:
            self.get_logger().warn(
                f'AlignToCluster reported failure (cluster {cluster_id})'
            )
            return False

        self.get_logger().info(
            f'  Alignment OK – {len(result.arms_to_pick)} arm(s) assigned'
        )
        return True

    # ── Pump helpers ──────────────────────────────────────────────────────────

    def _activate_all_pumps(self) -> bool:
        """Activate all vacuum pumps and wait for vacuum build-up."""
        for pump_id in range(self.num_pumps):
            if not self.control_pump(pump_id, enable=True, duty_cycle=1.0):
                self.get_logger().error(f'Failed to activate pump {pump_id}')
                return False
        time.sleep(self.pump_buildup_sec)
        return True

    def _release_all_pumps(self) -> None:
        """Deactivate all vacuum pumps (best-effort)."""
        for pump_id in range(self.num_pumps):
            self.control_pump(pump_id, enable=False)

    # ── Geometry helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _backup_pose(robot_pose: dict, dist: float) -> tuple:
        """
        Return (x, y) for a backup position *dist* metres behind *robot_pose*.

        "Behind" means opposite to the heading (theta).
        """
        theta = robot_pose['theta']
        return (
            robot_pose['x'] - dist * math.cos(theta),
            robot_pose['y'] - dist * math.sin(theta),
        )


def main(args=None):
    rclpy.init(args=args)
    mission = NutPickupMission()
    mission.load_mission_config()

    try:
        rclpy.spin(mission)
    except KeyboardInterrupt:
        pass
    finally:
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
