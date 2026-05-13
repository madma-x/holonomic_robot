"""Return-to-base mission handler for MissionExecutor."""


class ReturnBaseHandler:
    """Executes RETURN_BASE tasks by navigating to the configured base pose."""

    def __init__(self, executor_node):
        self.node = executor_node

    def can_handle(self, task: dict) -> bool:
        return str(task.get('task_type', '')).lower() == 'return_base'

    def execute(self, task: dict) -> dict:
        target_pose = task.get('target_pose', {})
        x = float(target_pose.get('x', 0.0))
        y = float(target_pose.get('y', 0.0))

        self.node.get_logger().info(
            f'RETURN_BASE: navigating to base position (x={x:.3f}, y={y:.3f})'
        )

        success = self.node.navigate_to_pose(x, y, 0.0)

        task_id = str(task.get('task_id', ''))
        status = 'COMPLETED' if success else 'FAILED'
        reason = None if success else 'NAVIGATION_FAILED'

        result = {
            'task_id': task_id,
            'task_type': 'return_base',
            'status': status,
        }
        if reason:
            result['outcome_reason'] = reason

        return result
