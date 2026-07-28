"""Return-to-base mission handler for MissionExecutor."""

from robot_application.arm_layout import ARM_SELECTION_PRIORITY
from robot_application.arm_sequences import ArmSequenceBuilder


class ReturnBaseHandler:
    """Executes RETURN_BASE tasks by navigating to the configured base pose."""

    def __init__(self, executor_node):
        self.node = executor_node
        self._sequence_builder = ArmSequenceBuilder()
        self.node.declare_parameter('blue_return_base_relative_move_x', -0.2)
        self.node.declare_parameter('blue_return_base_relative_move_y', 0.2)
        self.node.declare_parameter('blue_return_base_relative_move_theta', 0.0)
        self.node.declare_parameter('yellow_return_base_relative_move_x', 0.2)
        self.node.declare_parameter('yellow_return_base_relative_move_y', 0.2)
        self.node.declare_parameter('yellow_return_base_relative_move_theta', 0.0)

    def can_handle(self, task: dict) -> bool:
        return str(task.get('task_type', '')).lower() == 'return_base'

    def execute(self, task: dict) -> dict:
        target_pose = task.get('target_pose', {})
        team_color = self._normalize_team_color(task.get('team_color', 'blue'))
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
        if success:
            rel_x, rel_y, rel_theta = self._get_post_return_relative_move(team_color)
            self.node.get_logger().info(
                'RETURN_BASE: base reached, performing post-return relative move '
                f'team={team_color} x={rel_x:.3f} y={rel_y:.3f} theta={rel_theta:.3f}'
            )
            success = self.node.move_relative(rel_x, rel_y, rel_theta)

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

    def _normalize_team_color(self, value) -> str:
        normalized = str(value).strip().lower()
        return normalized if normalized in ('blue', 'yellow') else 'blue'

    def _get_post_return_relative_move(self, team_color: str):
        color = self._normalize_team_color(team_color)
        rel_x = float(self.node.get_parameter(f'{color}_return_base_relative_move_x').value)
        rel_y = float(self.node.get_parameter(f'{color}_return_base_relative_move_y').value)
        rel_theta = float(self.node.get_parameter(f'{color}_return_base_relative_move_theta').value)
        return rel_x, rel_y, rel_theta
