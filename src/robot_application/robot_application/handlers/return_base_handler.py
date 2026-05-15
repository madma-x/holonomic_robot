"""Return-to-base mission handler for MissionExecutor."""

from robot_application.arm_layout import ARM_SELECTION_PRIORITY
from robot_application.arm_sequences import ArmSequenceBuilder


class ReturnBaseHandler:
    """Executes RETURN_BASE tasks by navigating to the configured base pose."""

    def __init__(self, executor_node):
        self.node = executor_node
        self._sequence_builder = ArmSequenceBuilder()

    def can_handle(self, task: dict) -> bool:
        return str(task.get('task_type', '')).lower() == 'return_base'

    def execute(self, task: dict) -> dict:
        target_pose = task.get('target_pose', {})
        x = float(target_pose.get('x', 0.0))
        y = float(target_pose.get('y', 0.0))

        self.node.get_logger().info('RETURN_BASE: resetting arms before navigating to base')
        if not self._execute_arm_reset():
            return {
                'task_id': str(task.get('task_id', '')),
                'task_type': 'return_base',
                'status': 'FAILED',
                'outcome_reason': 'ARM_RESET_FAILED',
            }

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

    def _execute_arm_reset(self) -> bool:
        try:
            steps = self._sequence_builder.build_reset_sequence(list(ARM_SELECTION_PRIORITY))
        except RuntimeError as exc:
            self.node.get_logger().error(f'RETURN_BASE: failed to build arm reset sequence: {exc}')
            return False

        if not steps:
            return True
        return bool(self.node.execute_sequence(steps))
