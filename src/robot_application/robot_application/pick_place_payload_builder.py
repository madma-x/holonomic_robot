"""Builds canonical pick-place assignment payloads for MissionExecutor."""

from typing import Any, Dict, List


class PickPlacePayloadBuilder:
    """Composes initial and continuation payloads for MOVE_OBJECT tasks."""

    def build_assignment_payload(
        self,
        *,
        task_name: str,
        task_id: str,
        task_type: str,
        task_priority: int,
        task_parameters: Dict[str, Any],
        pick_id: str,
        drop_candidates: List[Dict[str, Any]],
        full_drop_ids: List[str],
    ) -> Dict[str, Any]:
        initial_drop_id = str(drop_candidates[0].get('id', '')) if drop_candidates else ''

        return {
            'name': task_name,
            'task_id': task_id,
            'task_type': task_type,
            'pick_location': task_parameters.get('pick_location', []),
            'drop_location': task_parameters.get('drop_location', []),
            'drop_positions': list(drop_candidates),
            'source_pick_id': pick_id,
            'target_drop_id': initial_drop_id,
            'excluded_drop_ids': [],
            'full_drop_ids': list(full_drop_ids),
            'priority': int(task_priority),
            'carry_object': bool(task_parameters.get('carry_object', False)),
        }

    def build_continuation_payload(
        self,
        *,
        original_payload: Dict[str, Any],
        outcome: Dict[str, Any],
        next_drop: Dict[str, Any],
        excluded_ids: List[str],
        full_drop_ids: List[str],
    ) -> Dict[str, Any]:
        candidate_drops = list(original_payload.get('drop_positions', []))
        continuation_payload = dict(original_payload)

        continuation_payload['carry_object'] = True
        continuation_payload['source_pick_id'] = str(
            outcome.get('source_pick_id', continuation_payload.get('source_pick_id', ''))
        )
        continuation_payload['target_drop_id'] = str(next_drop.get('id', ''))
        continuation_payload['active_arm_index'] = outcome.get(
            'active_arm_index',
            continuation_payload.get('active_arm_index'),
        )
        continuation_payload['active_arm_indices'] = outcome.get(
            'active_arm_indices',
            continuation_payload.get('active_arm_indices', []),
        )
        continuation_payload['excluded_drop_ids'] = list(excluded_ids)
        continuation_payload['full_drop_ids'] = list(full_drop_ids)
        continuation_payload['drop_positions'] = [next_drop] + [
            drop for drop in candidate_drops
            if str(drop.get('id', '')) != str(next_drop.get('id', ''))
        ]
        continuation_payload['object_color_before'] = str(
            outcome.get('object_color_before', continuation_payload.get('object_color_before', 'unknown'))
        )

        return continuation_payload
