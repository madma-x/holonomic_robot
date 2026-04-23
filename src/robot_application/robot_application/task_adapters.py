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


class ReturnBaseTaskAdapter(BaseTaskAdapter):
    """Adapter for RETURN_BASE until mission-executor-level handler exists."""

    def __init__(self, logger):
        self._logger = logger

    def execute(self, task: Task) -> bool:
        self._logger.warn('RETURN_BASE has no dedicated executor handler yet; treating as completed')
        return True


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
        wait_for_mission_executor_result: Callable[[float], bool],
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

        start_ok = self._start_mission_executor()
        if not start_ok:
            self._logger.error('Failed to start mission_executor mission')
            return False

        timeout = max(15.0, float(task.time_estimate) + 25.0)
        return self._wait_for_mission_executor_result(timeout)

    def process_outcome(self, outcome: Dict[str, Any]) -> Dict[str, Any]:
        return self._outcome_processor.process_outcome(
            outcome=outcome,
            world_state=self._world_state,
            task_queue=self._task_queue,
            task_context_by_id=self._task_context_by_id,
            task_pick_id_getter=self._task_pick_id_getter,
        )

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
