"""Processes pick-place mission outcomes into planner directives."""

from typing import Any, Callable, Dict, List


class PickPlaceOutcomeProcessor:
    """Applies pick/drop world-state transitions and returns next-step hints."""

    def process_outcome(
        self,
        *,
        outcome: Dict[str, Any],
        world_state,
        task_queue,
        task_context_by_id: Dict[str, Dict[str, Any]],
        task_pick_id_getter: Callable[[Any], str],
    ) -> Dict[str, Any]:
        status = str(outcome.get('status', '')).upper()
        reason = str(outcome.get('outcome_reason', '')).upper()
        source_pick_id = str(outcome.get('source_pick_id', ''))
        target_drop_id = str(outcome.get('target_drop_id', ''))

        result = {
            'needs_replan': False,
            'replan_tasks': False,
            'failed_drop_id': target_drop_id,
        }

        if status == 'FAILED' and reason == 'PICK_EMPTY' and source_pick_id:
            world_state.mark_pick_empty(source_pick_id)
            task_queue[:] = [
                task for task in task_queue
                if task_pick_id_getter(task) != source_pick_id
            ]
            task_id = str(outcome.get('task_id', ''))
            if task_id and task_id in task_context_by_id:
                del task_context_by_id[task_id]
            result['replan_tasks'] = True
            return result

        if status == 'REPLAN_REQUIRED' and reason == 'DROP_FULL' and target_drop_id:
            world_state.mark_drop_full(target_drop_id)
            result['needs_replan'] = True
            return result

        if status == 'COMPLETED':
            if source_pick_id:
                world_state.mark_pick_empty(source_pick_id)
            if target_drop_id:
                world_state.mark_drop_occupied(target_drop_id)
            task_id = str(outcome.get('task_id', ''))
            if task_id and task_id in task_context_by_id:
                del task_context_by_id[task_id]

        return result
