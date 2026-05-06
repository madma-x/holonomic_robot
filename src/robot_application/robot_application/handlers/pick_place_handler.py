"""Pick-and-place mission handler for MissionExecutor."""

import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from aruco_interfaces.msg import ArmAssignment
from aruco_interfaces.msg import ClusterPickability
from aruco_interfaces.msg import DetectedTagArray
from aruco_interfaces.srv import AlignToCluster as AlignToClusterSrv
from robot_application.arm_sequences import ArmSequenceBuilder


class PickPlaceHandler:
    """Executes MOVE_OBJECT tasks with pickability and alignment checks."""

    def __init__(self, executor_node):
        self.node = executor_node
        self.latest_pickability: Optional[ClusterPickability] = None
        self.latest_detected_tags: Optional[DetectedTagArray] = None
        self._latest_detected_tags_stamp = 0.0
        self._cached_team_color: Optional[str] = None

        self.node.declare_parameter('pickability_topic', '/cluster_pickability')
        self.node.declare_parameter('align_service_name', '/align_to_cluster')
        self.node.declare_parameter('align_timeout_sec', 12.0)
        self.node.declare_parameter('align_threshold', 0.02)
        self.node.declare_parameter('pickability_wait_sec', 2.0)
        self.node.declare_parameter('priority_penalty', 1)
        self.node.declare_parameter('sticky_confirm_wait_sec', 0.4)
        self.node.declare_parameter('tag_manager_node_name', '/tag_manager_node')
        self.node.declare_parameter('game_state_manager_node_name', '/game_state_manager')
        self.node.declare_parameter('detected_tags_topic', '/findeeznuts/detected_tags')
        self.node.declare_parameter('detected_tags_max_age_sec', 0.75)
        self.node.declare_parameter('drop_clear_wait_sec', 1.0)

        pickability_topic = self.node.get_parameter('pickability_topic').value
        align_service_name = self.node.get_parameter('align_service_name').value
        tag_manager_node_name = self.node.get_parameter('tag_manager_node_name').value
        game_state_manager_node_name = self.node.get_parameter('game_state_manager_node_name').value
        detected_tags_topic = self.node.get_parameter('detected_tags_topic').value

        self.sequence_builder = ArmSequenceBuilder()

        self.align_client = self.node.create_client(AlignToClusterSrv, align_service_name)
        self.tag_manager_params = AsyncParameterClient(self.node, str(tag_manager_node_name))
        self.game_state_params = AsyncParameterClient(self.node, str(game_state_manager_node_name))
        self.pickability_sub = self.node.create_subscription(
            ClusterPickability,
            pickability_topic,
            self.pickability_callback,
            10
        )
        self.detected_tags_sub = self.node.create_subscription(
            DetectedTagArray,
            detected_tags_topic,
            self.detected_tags_callback,
            10
        )

    def can_handle(self, task: dict) -> bool:
        task_type = str(task.get('task_type', '')).lower()
        return task_type in ('move_object', 'pick_place', 'pick_and_place')

    def pickability_callback(self, msg: ClusterPickability):
        self.latest_pickability = msg

    def detected_tags_callback(self, msg: DetectedTagArray):
        self.latest_detected_tags = msg
        self._latest_detected_tags_stamp = time.time()
        
    def execute(self, task: dict) -> Dict[str, Any]:
        carry_object = bool(task.get('carry_object', False))
        source_pick_id = str(task.get('source_pick_id', ''))
        active_arm_indices = self._task_arm_indices(task)
        selected_arm_tag_ids = self._task_selected_arm_tag_ids(task)
        pick_ref: Optional[dict] = None
        team_color = self.get_team_color()

        self.node.get_logger().info(
            'PickPlace execute() start: '
            f"task_id={task.get('task_id', 'unknown')} "
            f"carry_object={carry_object} "
        )

        if not carry_object:
            self.node.get_logger().info('Step 1: preparing pickup sequence')
            self._set_sticky_assignment(False)
            try:
                approach_positions = self.extract_approach_positions(task)
                self.node.get_logger().info(
                    f'Step 2: extracted {len(approach_positions)} approach position(s)'
                )
                approached = False
                for index, approach_pose in enumerate(approach_positions[:2], start=1):
                    self.node.get_logger().info(
                        'Step 3: attempting approach '
                        f'#{index} at x={float(approach_pose.get("x", 0.0)):.2f}, '
                        f'y={float(approach_pose.get("y", 0.0)):.2f}, '
                        f'theta={float(approach_pose.get("theta", 0.0)):.2f}'
                    )
                    if self.node.navigate_to_pose(
                        float(approach_pose.get('x', 0.0)),
                        float(approach_pose.get('y', 0.0)),
                        float(approach_pose.get('theta', 0.0))
                    ):
                        self.node.get_logger().info(f'Step 3: approach #{index} succeeded')
                        approached = True
                        break

                    self.node.get_logger().warn(f'Step 3: approach #{index} failed')

                if not approached:
                    self.node.get_logger().error('Step 4: all pickup approaches failed')
                    self.lower_pick_priority(task, pick_ref, 'approach unreachable')
                    return self._build_outcome(task, 'FAILED', 'NAV_FAIL', False, source_pick_id=source_pick_id)


                self.node.get_logger().info('Step 6: waiting for pickability message')
                pickability = self.wait_for_pickability()
                if pickability is None or not bool(pickability.is_pickable):
                    self.node.get_logger().warn('Step 6: pickability check failed or timed out')
                    self.lower_pick_priority(task, pick_ref, 'not pickable')
                    return self._build_outcome(
                        task,
                        'FAILED',
                        'PICK_EMPTY',
                        False,
                        source_pick_id=source_pick_id,
                        active_arm_indices=active_arm_indices,
                    )

                self.node.get_logger().info(
                    'Step 6: pickability confirmed '
                    f'Step 6: pickability message: {pickability}'
                )
                active_arm_indices = self.select_pick_arm_indices(pickability)
                if not active_arm_indices:
                    self.node.get_logger().warn('Step 6: no arm is assigned to a pickable object')
                    self.lower_pick_priority(task, pick_ref, 'no assigned arm')
                    return self._build_outcome(
                        task,
                        'FAILED',
                        'PICK_EMPTY',
                        False,
                        source_pick_id=source_pick_id,
                        active_arm_indices=active_arm_indices,
                    )

                task['active_arm_indices'] = list(active_arm_indices)
                task['active_arm_index'] = active_arm_indices[0]
                assigned_arms = self.get_selected_arm_assignments(pickability, active_arm_indices)
                selected_tag_ids = [
                    int(getattr(arm, 'tag_id', 0))
                    for arm in assigned_arms
                ]
                selected_arm_tag_ids = {
                    int(getattr(arm, 'arm_index', -1)): int(getattr(arm, 'tag_id', 0))
                    for arm in assigned_arms
                    if int(getattr(arm, 'arm_index', -1)) >= 0
                }
                task['selected_arm_tag_ids'] = {
                    str(arm_index): int(tag_id)
                    for arm_index, tag_id in selected_arm_tag_ids.items()
                }
                self.node.get_logger().info(
                    f'Step 6: selected arm_indices={active_arm_indices} tag_ids={selected_tag_ids}'
                )

                # Keep arm->tag mapping stable only while robot is in the pick window.
                self.node.get_logger().info('Step 5: enabling sticky arm-tag assignment')
                self._set_sticky_assignment(True)

                if not self.wait_for_sticky_active():
                    self.node.get_logger().warn('Step 5: sticky confirmation timeout, proceeding anyway')


                cluster_id = int(getattr(pickability, 'cluster_id', 0))
                self.node.get_logger().info(f'Step 7: calling align_to_cluster for cluster_id={cluster_id}')
                if not self.align_to_cluster(cluster_id):
                    self.node.get_logger().error('Step 7: alignment failed')
                    self.lower_pick_priority(task, pick_ref, 'alignment failed')
                    return self._build_outcome(task, 'FAILED', 'ALIGN_FAIL', False, source_pick_id=source_pick_id)

                self.node.get_logger().info('Step 7: alignment succeeded')
                self.node.get_logger().info(
                    f'Step 8: executing pick sequence for arm_indices={active_arm_indices}'
                )
                if not self.execute_pick_sequence(active_arm_indices):
                    self.node.get_logger().error('Step 8: pick sequence failed')
                    self.lower_pick_priority(task, pick_ref, 'actuation failed')
                    return self._build_outcome(
                        task,
                        'FAILED',
                        'ACTUATION_FAIL',
                        False,
                        source_pick_id=source_pick_id,
                        active_arm_indices=active_arm_indices,
                    )

                self.node.get_logger().info('Step 8: pick sequence succeeded')
                carry_object = True
            finally:
                self.node.get_logger().info('Step 9: disabling sticky arm-tag assignment')
                self._set_sticky_assignment(False)

        excluded_ids = set(task.get('excluded_drop_ids', []))
        self.node.get_logger().info(
            f'Step 10: selecting drop location, excluded_ids={sorted(list(excluded_ids))}'
        )
        drop_info = self.select_drop_location(task, excluded_ids=excluded_ids)
        if drop_info is None:
            self.node.get_logger().warn('Step 10: no drop location available')
            if pick_ref is not None:
                self.lower_pick_priority(task, pick_ref, 'no drop location')
            return self._build_outcome(
                task,
                'REPLAN_REQUIRED',
                'DROP_FULL',
                carry_object,
                source_pick_id=source_pick_id,
                active_arm_indices=active_arm_indices,
            )

        drop_location, drop_ref = drop_info
        drop_id = str(drop_ref.get('id', task.get('target_drop_id', '')))
        self.node.get_logger().info(
            'Step 11: selected drop '
            f'id={drop_id} at x={float(drop_location.get("x", 0.0)):.2f}, '
            f'y={float(drop_location.get("y", 0.0)):.2f}, '
            f'theta={float(drop_location.get("theta", 0.0)):.2f}'
        )
        if self._is_drop_full(task, drop_ref):
            self.node.get_logger().warn(f'Step 11: drop {drop_id} is marked full')
            excluded_ids.add(drop_id)
            task['excluded_drop_ids'] = sorted(list(excluded_ids))
            return self._build_outcome(
                task,
                'REPLAN_REQUIRED',
                'DROP_FULL',
                carry_object,
                source_pick_id=source_pick_id,
                target_drop_id=drop_id,
                active_arm_indices=active_arm_indices,
                selected_arm_tag_ids=selected_arm_tag_ids,
            )

        self.node.get_logger().info(f'Step 12: navigating to drop {drop_id}')
        if not self.node.navigate_to_pose(
            float(drop_location.get('x', 0.0)),
            float(drop_location.get('y', 0.0)),
            float(drop_location.get('theta', 0.0))
        ):
            self.node.get_logger().error(f'Step 12: navigation to drop {drop_id} failed')
            if pick_ref is not None:
                self.lower_pick_priority(task, pick_ref, 'drop unreachable')
            return self._build_outcome(
                task,
                'FAILED',
                'NAV_FAIL',
                carry_object,
                source_pick_id=source_pick_id,
                target_drop_id=drop_id,
                active_arm_indices=active_arm_indices,
            )        
        if not active_arm_indices:
            self.node.get_logger().error('Step 13: no active arms available for drop sequence')
            return self._build_outcome(
                task,
                'FAILED',
                'ACTUATION_FAIL',
                carry_object,
                source_pick_id=source_pick_id,
                target_drop_id=drop_id,
                active_arm_indices=active_arm_indices,
            )

        occupancy_result = self.evaluate_drop_occupancy(team_color)
        if occupancy_result == 'drop_full':
            self.node.get_logger().warn(f'Step 13: drop {drop_id} appears full for team={team_color}, replanning')
            excluded_ids.add(drop_id)
            task['excluded_drop_ids'] = sorted(list(excluded_ids))
            return self._build_outcome(
                task,
                'REPLAN_REQUIRED',
                'DROP_FULL',
                carry_object,
                source_pick_id=source_pick_id,
                target_drop_id=drop_id,
                active_arm_indices=active_arm_indices,
                selected_arm_tag_ids=selected_arm_tag_ids,
            )

        if occupancy_result == 'clear_needed' and not self.clear_drop_position_placeholder(drop_id):
            return self._build_outcome(
                task,
                'FAILED',
                'DROP_CLEAR_FAIL',
                carry_object,
                source_pick_id=source_pick_id,
                target_drop_id=drop_id,
                active_arm_indices=active_arm_indices,
                selected_arm_tag_ids=selected_arm_tag_ids,
            )

        swap_arm_indices = self.swap_arm_indices_for_team(
            active_arm_indices,
            selected_arm_tag_ids,
            team_color,
        )
        self.node.get_logger().info(
            f'Step 13: executing drop sequence for arm_indices={active_arm_indices} '
            f'swap_arm_indices={swap_arm_indices}'
        )
        if not self.execute_place_sequence(active_arm_indices, swap_arm_indices=swap_arm_indices):
            self.node.get_logger().warn('Step 13: place sequence failed — object may not be released')

        if not self.node.move_relative(-0.08, 0.0, 0.0):
            self.node.get_logger().warn('Step 13b: post-drop retreat move failed, continuing')
        
        self.node.get_logger().info(f'Step 14: task completed successfully, placed at drop {drop_id}')
        return self._build_outcome(
            task,
            'COMPLETED',
            'PLACED',
            False,
            source_pick_id=source_pick_id,
            target_drop_id=drop_id,
            active_arm_indices=active_arm_indices,
            selected_arm_tag_ids=selected_arm_tag_ids,
        )


    def _build_outcome(
        self,
        task: dict,
        status: str,
        outcome_reason: str,
        carry_object: bool,
        source_pick_id: str = '',
        target_drop_id: str = '',
        active_arm_indices: Optional[List[int]] = None,
        selected_arm_tag_ids: Optional[Dict[int, int]] = None,
    ) -> Dict[str, Any]:
        arm_indices = list(active_arm_indices or [])
        selected_map = selected_arm_tag_ids or self._task_selected_arm_tag_ids(task)
        return {
            'task_id': str(task.get('task_id', 'unknown')),
            'task_type': str(task.get('task_type', 'unknown')),
            'status': status,
            'outcome_reason': outcome_reason,
            'carry_object': bool(carry_object),
            'source_pick_id': source_pick_id,
            'target_drop_id': target_drop_id,
            'active_arm_index': arm_indices[0] if arm_indices else None,
            'active_arm_indices': arm_indices,
            'selected_arm_tag_ids': {
                str(arm_index): int(tag_id)
                for arm_index, tag_id in selected_map.items()
            },
        }

    def wait_for_sticky_active(self) -> bool:
        timeout = float(self.node.get_parameter('sticky_confirm_wait_sec').value)
        deadline = self.node.get_clock().now() + Duration(seconds=timeout)

        while self.node.get_clock().now() < deadline and not self.node.stop_requested:
            if self.latest_pickability is not None and bool(getattr(self.latest_pickability, 'sticky_active', False)):
                return True
            time.sleep(0.02)

        return False

    def wait_for_pickability(self) -> Optional[ClusterPickability]:
        timeout = float(self.node.get_parameter('pickability_wait_sec').value)
        deadline = self.node.get_clock().now() + Duration(seconds=timeout)

        while self.node.get_clock().now() < deadline and not self.node.stop_requested:
            if self.latest_pickability is not None:
                return self.latest_pickability
            time.sleep(0.05)

        return None

    def align_to_cluster(self, cluster_id: int) -> bool:
        if not self.align_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error('AlignToCluster service not available')
            return False

        request = AlignToClusterSrv.Request()
        request.cluster_id = cluster_id
        request.alignment_threshold = float(self.node.get_parameter('align_threshold').value)
        request.max_wait_time = float(self.node.get_parameter('align_timeout_sec').value)

        timeout = float(self.node.get_parameter('align_timeout_sec').value)
        future = self.align_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=timeout + 1.0):
            self.node.get_logger().error('AlignToCluster service call timed out')
            return False
        
        if future.result() is None:
            self.node.get_logger().error('AlignToCluster service call timed out')
            return False

        response = future.result()
        if not response.success:
            self.node.get_logger().error(f'Alignment failed: {response.status_message}')
            return False

        self.node.get_logger().info(f'Alignment successful: {response.status_message}')
        return bool(response.success)

    def execute_pick_sequence(self, arm_indices: List[int]) -> bool:
        """Execute the configured pick sequence for the selected arms."""
        try:
            steps = self.sequence_builder.build_pick_sequence(arm_indices)
        except RuntimeError as exc:
            self.node.get_logger().error(str(exc))
            return False
        return self.node.execute_sequence(steps)

    def execute_swap_sequence(self, arm_indices: List[int]) -> bool:
        """Execute the configured swap sequence for selected arms."""
        try:
            steps = self.sequence_builder.build_swap_sequence(arm_indices)
        except RuntimeError as exc:
            self.node.get_logger().error(str(exc))
            return False
        if not steps:
            return True
        return self.node.execute_sequence(steps)

    def execute_place_sequence(
        self,
        arm_indices: List[int],
        swap_arm_indices: Optional[List[int]] = None,
    ) -> bool:
        """Execute the configured drop sequence for the selected arms."""
        try:
            steps = self.sequence_builder.build_place_sequence(
                arm_indices,
                swap_arm_indices=swap_arm_indices,
            )
        except RuntimeError as exc:
            self.node.get_logger().error(str(exc))
            return False
        return self.node.execute_sequence(steps)

    def select_pick_arm_indices(self, pickability: ClusterPickability) -> List[int]:
        return self.sequence_builder.select_arm_indices(pickability)

    def select_pick_arm_index(self, pickability: ClusterPickability) -> Optional[int]:
        return self.sequence_builder.select_arm_index(pickability)

    def get_selected_arm_assignment(
        self, pickability: ClusterPickability, arm_index: int
    ) -> Optional[ArmAssignment]:
        return self.sequence_builder.get_assigned_arm(pickability, arm_index)

    def get_selected_arm_assignments(
        self, pickability: ClusterPickability, arm_indices: List[int]
    ) -> List[ArmAssignment]:
        assigned = []
        for arm_index in arm_indices:
            arm = self.get_selected_arm_assignment(pickability, arm_index)
            if arm is not None:
                assigned.append(arm)
        return assigned

    def swap_arm_indices_for_team(
        self,
        active_arm_indices: List[int],
        selected_arm_tag_ids: Dict[int, int],
        team_color: str,
    ) -> List[int]:
        swap_tag_id = 47 if team_color == 'blue' else 36
        return [
            arm_index
            for arm_index in active_arm_indices
            if int(selected_arm_tag_ids.get(arm_index, -1)) == swap_tag_id
        ]

    def _task_arm_index(self, task: dict) -> Optional[int]:
        raw_value = task.get('active_arm_index')
        if raw_value is None or raw_value == '':
            return None
        try:
            return int(raw_value)
        except (TypeError, ValueError):
            return None

    def _task_arm_indices(self, task: dict) -> List[int]:
        raw_indices = task.get('active_arm_indices')
        if isinstance(raw_indices, list):
            return self._normalize_arm_index_list(raw_indices)
        single_arm_index = self._task_arm_index(task)
        return [] if single_arm_index is None else [single_arm_index]

    def _task_selected_arm_tag_ids(self, task: dict) -> Dict[int, int]:
        raw_map = task.get('selected_arm_tag_ids')
        if not isinstance(raw_map, dict):
            return {}

        normalized: Dict[int, int] = {}
        for raw_arm_index, raw_tag_id in raw_map.items():
            try:
                arm_index = int(raw_arm_index)
                tag_id = int(raw_tag_id)
            except (TypeError, ValueError):
                continue
            normalized[arm_index] = tag_id
        return normalized

    def _normalize_arm_index_list(self, values: List[Any]) -> List[int]:
        arm_indices = []
        for value in values:
            try:
                normalized = int(value)
            except (TypeError, ValueError):
                continue
            if normalized not in arm_indices:
                arm_indices.append(normalized)
        return arm_indices

    def lower_pick_priority(self, task: dict, pick_ref: Optional[dict], reason: str):
        penalty = int(self.node.get_parameter('priority_penalty').value)
        if isinstance(pick_ref, dict) and 'priority' in pick_ref:
            pick_ref['priority'] = max(0, int(pick_ref.get('priority', 0)) - penalty)
        elif 'priority' in task:
            task['priority'] = max(0, int(task.get('priority', 0)) - penalty)

        self.node.get_logger().warn(f'Pick priority lowered ({reason})')

    def _set_sticky_assignment(self, enabled: bool) -> None:
        if not self.tag_manager_params.wait_for_services(timeout_sec=0.2):
            return

        param = Parameter('sticky_assignment', Parameter.Type.BOOL, bool(enabled))
        future = self.tag_manager_params.set_parameters([param])
        self._wait_for_future(future, timeout_sec=0.5)

    def get_team_color(self) -> str:
        if self._cached_team_color:
            return self._cached_team_color

        if not self.game_state_params.wait_for_services(timeout_sec=0.3):
            self.node.get_logger().warn('Game state parameter service unavailable, defaulting team_color=blue')
            self._cached_team_color = 'blue'
            return self._cached_team_color

        future = self.game_state_params.get_parameters(['team_color'])
        if not self._wait_for_future(future, timeout_sec=0.5):
            self.node.get_logger().warn('team_color parameter request timed out, defaulting team_color=blue')
            self._cached_team_color = 'blue'
            return self._cached_team_color

        result = future.result()
        if not result or not getattr(result, 'values', None):
            self.node.get_logger().warn('team_color parameter missing, defaulting team_color=blue')
            self._cached_team_color = 'blue'
            return self._cached_team_color

        value = str(result.values[0].string_value)
        if not value:
            self.node.get_logger().warn('team_color empty, defaulting team_color=blue')
            value = 'blue'
        self._cached_team_color = value
        self.node.get_logger().info(f'Using persistent team_color={self._cached_team_color}')
        return self._cached_team_color

    def evaluate_drop_occupancy(self, team_color: str) -> str:
        max_age = float(self.node.get_parameter('detected_tags_max_age_sec').value)
        pickability = self.latest_pickability
        if pickability is None:
            self.node.get_logger().warn('Pickability unavailable; skipping drop occupancy gate')
            return 'unknown'

        stamp = getattr(pickability, 'header', None).stamp if getattr(pickability, 'header', None) else None
        if stamp is not None:
            stamp_sec = float(getattr(stamp, 'sec', 0)) + float(getattr(stamp, 'nanosec', 0)) * 1e-9
            if stamp_sec > 0.0 and (time.time() - stamp_sec) > max_age:
                self.node.get_logger().warn('Pickability stale; skipping drop occupancy gate')
                return 'unknown'

        majority_color = str(getattr(pickability, 'majority_color', 'none')).strip().lower()
        total_tags = int(getattr(pickability, 'total_tags', 0))
        team_color = str(team_color).strip().lower()

        if total_tags <= 0:
            self.node.get_logger().info(
                f'Drop occupancy: no occupancy evidence (majority={majority_color}, total_tags={total_tags}), proceed'
            )
            return 'proceed'

        if majority_color == 'equal':
            self.node.get_logger().info(
                f'Drop occupancy: equal team/non-team occupancy (total_tags={total_tags}), clear needed'
            )
            return 'clear_needed'

        if majority_color == team_color:
            self.node.get_logger().info(
                f'Drop occupancy: team majority={majority_color} (total_tags={total_tags}), drop full'
            )
            return 'drop_full'

        if majority_color in ('blue', 'yellow'):
            self.node.get_logger().info(
                f'Drop occupancy: non-team majority={majority_color} (total_tags={total_tags}), clear needed'
            )
            return 'clear_needed'

        self.node.get_logger().info(
            f'Drop occupancy: unrecognized majority={majority_color} (total_tags={total_tags}), proceed'
        )
        return 'proceed'

    def clear_drop_position_placeholder(self, drop_id: str) -> bool:
        """Temporary drop-clear hook for future implementation."""
        wait_sec = float(self.node.get_parameter('drop_clear_wait_sec').value)
        self.node.get_logger().warn(
            f'Drop {drop_id}: clear-drop placeholder invoked (TODO implementation), waiting {wait_sec:.2f}s'
        )
        time.sleep(max(0.0, wait_sec))
        return True

    def extract_approach_positions(self, task: dict) -> List[dict]:
        """Return priority-ordered approach poses for the selected pick target."""
        primary_pick = task.get('pick_location')
        if not primary_pick:
            return []

        approach_positions = list(primary_pick.get('approach_positions', []))
        if approach_positions:
            return sorted(
                approach_positions,
                key=lambda approach: int(approach.get('priority', 999))
            )

        location = primary_pick.get('location', {})
        if location:
            return [{
                'id': str(primary_pick.get('id', 'pick_location')),
                'priority': int(primary_pick.get('priority', 1)),
                'x': float(location.get('x', 0.0)),
                'y': float(location.get('y', 0.0)),
                'theta': float(location.get('theta', 0.0)),
            }]

        return []

    def select_drop_location(
        self,
        task: dict,
        excluded_ids: Optional[set] = None
    ) -> Optional[Tuple[dict, dict]]:
        """Select the first available drop location not excluded by replanning."""
        excluded_ids = excluded_ids or set()
        drop_ref = task.get('drop_location')
        if drop_ref:
            drop_positions = [drop_ref]
        else:
            drop_positions = task.get('drop_positions', []) or task.get('drop_locations', [])

        for drop_ref in drop_positions:
            drop_id = str(drop_ref.get('id', ''))
            if drop_id in excluded_ids:
                continue

            task['target_drop_id'] = drop_id
            location = drop_ref.get('location')
            if not location:
                approaches = drop_ref.get('approach_positions', [])
                location = approaches[0] if approaches else {'x': 0.0, 'y': 0.0, 'theta': 0.0}

            return location, drop_ref

        return None

    def _is_drop_full(self, task: dict, drop_ref: dict) -> bool:
        """Check whether the planner marked the drop target as full."""
        drop_id = str(drop_ref.get('id', ''))
        full_ids = {str(item) for item in task.get('full_drop_ids', [])}
        return drop_id in full_ids

    def _wait_for_future(self, future, timeout_sec: float, poll_interval: float = 0.05) -> bool:
        """Wait for an async future without re-entering the spinning executor."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not self.node.stop_requested:
            if future.done():
                return True
            time.sleep(poll_interval)
        return future.done()
