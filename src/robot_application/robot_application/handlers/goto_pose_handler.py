"""Go-to-pose mission handler for MissionExecutor."""

from typing import Any, Dict


class GotoPoseHandler:
    """Executes GOTO_POSE tasks by navigating to a target pose."""

    def __init__(self, executor_node):
        self.node = executor_node

    def can_handle(self, task: dict) -> bool:
        task_type = str(task.get('task_type', '')).lower()
        return task_type in ('goto_pose', 'go_to_pose', 'navigate_to_pose')

    def execute(self, task: dict) -> Dict[str, Any]:
        task_id = str(task.get('task_id', 'unknown'))
        self.node.get_logger().info(f'GotoPose execute() start: task_id={task_id}')

        target = task.get('target_location') or task.get('target_pose') or {}
        x = float(target.get('x', 0.0))
        y = float(target.get('y', 0.0))
        theta = float(target.get('theta', 0.0))

        self.node.get_logger().info(
            f'Step 1: navigating to x={x:.3f}, y={y:.3f}, theta={theta:.3f}'
        )

        if not self.node.navigate_to_pose(x, y, theta):
            self.node.get_logger().error('Step 1: navigation failed')
            return self._build_outcome(task, 'FAILED', 'NAV_FAIL')

        self.node.get_logger().info('Step 1: navigation succeeded')
        return self._build_outcome(task, 'COMPLETED', 'ARRIVED')

    def _build_outcome(
        self,
        task: dict,
        status: str,
        outcome_reason: str,
    ) -> Dict[str, Any]:
        return {
            'task_id': str(task.get('task_id', 'unknown')),
            'task_type': str(task.get('task_type', 'unknown')),
            'status': status,
            'outcome_reason': outcome_reason,
            'carry_object': False,
            'source_pick_id': '',
            'target_drop_id': '',
            'active_arm_index': None,
            'active_arm_indices': [],
            'selected_arm_tag_ids': {},
        }
