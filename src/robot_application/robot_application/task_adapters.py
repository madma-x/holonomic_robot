"""Task adapter layer to keep TaskPlanner generic across seasons."""

import json
from typing import Any, Callable, Dict, Optional

from std_msgs.msg import String

from robot_application.pick_place_outcome_processor import PickPlaceOutcomeProcessor
from robot_application.pick_place_payload_builder import PickPlacePayloadBuilder
from robot_application.task_definitions import Task


class BaseTaskAdapter:
    """Base adapter contract for task-type-specific execution behavior."""

    def execute(self, task: Task) -> bool:
        raise NotImplementedError

    def process_outcome(self, outcome: Dict[str, Any]) -> Dict[str, Any]:
        return {'replan_tasks': False, 'needs_replan': False}

    def handle_replan_required(self, outcome: Dict[str, Any]) -> bool:
        return False


class GotoPoseTaskAdapter(BaseTaskAdapter):
    """Adapter for GOTO_POSE: navigate to a target pose through mission_executor."""

    def __init__(
        self,
        logger,
        mission_assignment_pub=None,
        start_mission_executor: Callable[[], bool] = None,
        wait_for_mission_executor_result: Callable[[str, float], bool] = None,
    ):
        self._logger = logger
        self._mission_assignment_pub = mission_assignment_pub
        self._start_mission_executor = start_mission_executor
        self._wait_for_mission_executor_result = wait_for_mission_executor_result

    def execute(self, task: Task) -> bool:
        target_location = task.parameters.get('target_location', {})
        if not target_location:
            self._logger.error('GOTO_POSE task has no target_location parameter')
            return False

        if (self._mission_assignment_pub is None
                or self._start_mission_executor is None
                or self._wait_for_mission_executor_result is None):
            self._logger.warn('GOTO_POSE adapter not fully configured; treating as completed')
            return True

        if not self._start_mission_executor():
            self._logger.error('Failed to start mission_executor for GOTO_POSE')
            return False

        payload = {
            'task_id': task.task_id,
            'task_type': task.task_type.value,
            'task_name': task.name,
            'target_pose': {
                'x': float(target_location.get('x', 0.0)),
                'y': float(target_location.get('y', 0.0)),
                'theta': float(target_location.get('theta', 0.0)),
            },
        }

        assignment_msg = String()
        assignment_msg.data = json.dumps(payload)
        self._mission_assignment_pub.publish(assignment_msg)
        self._logger.info(
            f'Dispatched GOTO_POSE to mission_executor: task_id={task.task_id} '
            f'target=({payload["target_pose"]["x"]:.3f}, {payload["target_pose"]["y"]:.3f}, '
            f'{payload["target_pose"]["theta"]:.3f})'
        )

        timeout = max(120.0, float(task.time_estimate) + 60.0)
        return self._wait_for_mission_executor_result(task.task_id, timeout)


class ReturnBaseTaskAdapter(BaseTaskAdapter):
    """Adapter for RETURN_BASE: navigates the robot to the base pose."""

    def __init__(
        self,
        logger,
        mission_assignment_pub=None,
        start_mission_executor: Callable[[], bool] = None,
        wait_for_mission_executor_result: Callable[[str, float], bool] = None,
    ):
        self._logger = logger
        self._mission_assignment_pub = mission_assignment_pub
        self._start_mission_executor = start_mission_executor
        self._wait_for_mission_executor_result = wait_for_mission_executor_result

    def execute(self, task: Task) -> bool:
        target_location = task.parameters.get('target_location', {})
        if not target_location:
            self._logger.error('RETURN_BASE task has no target_location parameter')
            return False

        if (self._mission_assignment_pub is None
                or self._start_mission_executor is None
                or self._wait_for_mission_executor_result is None):
            self._logger.warn('RETURN_BASE adapter not fully configured; treating as completed')
            return True

        if not self._start_mission_executor():
            self._logger.error('Failed to start mission_executor for RETURN_BASE')
            return False

        payload = {
            'task_id': task.task_id,
            'task_type': task.task_type.value,
            'task_name': task.name,
            'target_pose': {
                'x': float(target_location.get('x', 0.0)),
                'y': float(target_location.get('y', 0.0)),
                'theta': float(target_location.get('theta', 0.0)),
            },
        }

        assignment_msg = String()
        assignment_msg.data = json.dumps(payload)
        self._mission_assignment_pub.publish(assignment_msg)
        self._logger.info(
            f'Dispatched RETURN_BASE to mission_executor: task_id={task.task_id} '
            f'target=({payload["target_pose"]["x"]:.3f}, {payload["target_pose"]["y"]:.3f}, '
            f'{payload["target_pose"]["theta"]:.3f})'
        )

        timeout = max(120.0, float(task.time_estimate) + 60.0)
        return self._wait_for_mission_executor_result(task.task_id, timeout)


class PickPlaceTaskAdapter(BaseTaskAdapter):
    """Adapter for MOVE_OBJECT pick-place task composition and outcome handling."""

    def __init__(
        self,
        *,
        logger,
        mission_assignment_pub,
        world_state,
        task_context_by_id: Dict[str, Dict[str, Any]],
        task_queue,
        task_pick_id_getter: Callable[[Task], str],
        task_drop_candidates_getter: Callable[[Task], list],
        start_mission_executor: Callable[[], bool],
        wait_for_mission_executor_result: Callable[[str, float], bool],
    ):
        self._logger = logger
        self._mission_assignment_pub = mission_assignment_pub
        self._world_state = world_state
        self._task_context_by_id = task_context_by_id
        self._task_queue = task_queue
        self._task_pick_id_getter = task_pick_id_getter
        self._task_drop_candidates_getter = task_drop_candidates_getter
        self._start_mission_executor = start_mission_executor
        self._wait_for_mission_executor_result = wait_for_mission_executor_result

        self._payload_builder = PickPlacePayloadBuilder()
        self._outcome_processor = PickPlaceOutcomeProcessor()

    def execute(self, task: Task) -> bool:
        full_drop_ids = self._world_state.get_full_drop_ids()
        pick_id = self._task_pick_id_getter(task)
        drop_candidates = self._task_drop_candidates_getter(task)

        start_ok = self._start_mission_executor()
        if not start_ok:
            self._logger.error('Failed to start mission_executor mission')
            return False

        payload = self._payload_builder.build_assignment_payload(
            task_name=task.name,
            task_id=task.task_id,
            task_type=task.task_type.value,
            task_priority=task.base_priority,
            task_parameters=task.parameters,
            pick_id=pick_id,
            drop_candidates=drop_candidates,
            full_drop_ids=full_drop_ids,
        )

        self._task_context_by_id[task.task_id] = payload

        assignment_msg = String()
        assignment_msg.data = json.dumps(payload)
        self._mission_assignment_pub.publish(assignment_msg)
        self._logger.info(f'Dispatched task to mission_executor: {task.task_id}')

        timeout = max(15.0, float(task.time_estimate) + 25.0)
        return self._wait_for_mission_executor_result(task.task_id, timeout)

    def process_outcome(self, outcome: Dict[str, Any]) -> Dict[str, Any]:
        self._sync_task_retry_state_from_outcome(outcome)
        return self._outcome_processor.process_outcome(
            outcome=outcome,
            world_state=self._world_state,
            task_queue=self._task_queue,
            task_context_by_id=self._task_context_by_id,
            task_pick_id_getter=self._task_pick_id_getter,
        )

    def _sync_task_retry_state_from_outcome(self, outcome: Dict[str, Any]) -> None:
        """Persist carry/drop context into queued task state for retry attempts."""
        task_id = str(outcome.get('task_id', ''))
        if not task_id:
            return

        carry_object = bool(outcome.get('carry_object', False))
        source_pick_id = str(outcome.get('source_pick_id', ''))
        active_arm_index = outcome.get('active_arm_index')
        active_arm_indices = outcome.get('active_arm_indices', [])
        selected_arm_tag_ids = outcome.get('selected_arm_tag_ids', {})

        task_obj = next((task for task in self._task_queue if task.task_id == task_id), None)
        if task_obj is not None:
            task_obj.parameters['carry_object'] = carry_object
            if source_pick_id:
                task_obj.parameters['source_pick_id'] = source_pick_id
            if active_arm_index is not None:
                task_obj.parameters['active_arm_index'] = active_arm_index
            if isinstance(active_arm_indices, list):
                task_obj.parameters['active_arm_indices'] = list(active_arm_indices)
            if isinstance(selected_arm_tag_ids, dict):
                task_obj.parameters['selected_arm_tag_ids'] = dict(selected_arm_tag_ids)

        context_payload = self._task_context_by_id.get(task_id)
        if isinstance(context_payload, dict):
            context_payload['carry_object'] = carry_object
            if source_pick_id:
                context_payload['source_pick_id'] = source_pick_id
            if active_arm_index is not None:
                context_payload['active_arm_index'] = active_arm_index
            if isinstance(active_arm_indices, list):
                context_payload['active_arm_indices'] = list(active_arm_indices)
            if isinstance(selected_arm_tag_ids, dict):
                context_payload['selected_arm_tag_ids'] = dict(selected_arm_tag_ids)

    def handle_replan_required(self, outcome: Dict[str, Any]) -> bool:
        reason = str(outcome.get('outcome_reason', '')).upper()
        carry_object = bool(outcome.get('carry_object', False))
        task_id = str(outcome.get('task_id', ''))

        if reason != 'DROP_FULL' or not carry_object or not task_id:
            return False

        original_payload = self._task_context_by_id.get(task_id)
        if not original_payload:
            return False

        failed_drop_id = str(outcome.get('target_drop_id', ''))
        excluded_ids = set(original_payload.get('excluded_drop_ids', []))
        excluded_ids.update(str(item) for item in outcome.get('excluded_drop_ids', []))
        if failed_drop_id:
            excluded_ids.add(failed_drop_id)

        candidate_drops = list(original_payload.get('drop_positions', []))
        next_drop = None
        for drop in sorted(candidate_drops, key=lambda item: int(item.get('priority', 999))):
            drop_id = str(drop.get('id', ''))
            if drop_id in excluded_ids:
                continue
            if self._world_state.is_drop_available(drop_id):
                next_drop = drop
                break

        if next_drop is None:
            return False

        continuation_payload = self._payload_builder.build_continuation_payload(
            original_payload=original_payload,
            outcome=outcome,
            next_drop=next_drop,
            excluded_ids=sorted(list(excluded_ids)),
            full_drop_ids=self._world_state.get_full_drop_ids(),
        )

        self._task_context_by_id[task_id] = continuation_payload
        assignment_msg = String()
        assignment_msg.data = json.dumps(continuation_payload)
        self._mission_assignment_pub.publish(assignment_msg)
        self._logger.info(
            f'Replanned drop for {task_id}: previous={failed_drop_id}, next={continuation_payload["target_drop_id"]}'
        )
        return True
