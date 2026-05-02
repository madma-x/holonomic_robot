#!/usr/bin/env python3
"""Run one arm actuator sequence via /execute_sequence.

Default behavior: run the pick sequence for arm index 1.
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_actuators.action import ExecuteSequence
from robot_application.arm_sequences import ArmSequenceBuilder


class ArmSequenceClient(Node):
    def __init__(self):
        super().__init__('arm_sequence_test_client')
        self._client = ActionClient(self, ExecuteSequence, 'execute_sequence')

    def run(self, arm_index: int, mode: str, timeout_sec: float) -> int:
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('execute_sequence action server not available')
            return 1

        builder = ArmSequenceBuilder()
        if mode == 'pick':
            steps = builder.build_pick_sequence([arm_index])
        elif mode == 'swap':
            steps = builder.build_swap_sequence([arm_index])
        elif mode in ('place', 'drop'):
            steps = builder.build_place_sequence([arm_index])
        else:
            self.get_logger().error(f'Unsupported mode: {mode}')
            return 2

        if not steps:
            self.get_logger().warn('No steps generated for this request')
            return 0

        self.get_logger().info(
            f'Sending {len(steps)} steps for arm {arm_index} in {mode} mode'
        )

        goal = ExecuteSequence.Goal()
        goal.steps = steps

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected by execute_sequence server')
            return 3

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error('Timeout waiting for sequence result')
            return 4

        result = wrapped_result.result
        if result.success:
            self.get_logger().info('Sequence completed successfully')
            return 0

        self.get_logger().error(
            f'Sequence failed at step {result.failed_step}: {result.message}'
        )
        return 5


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Execute sequence built by ArmSequenceBuilder.'
    )
    parser.add_argument('--arm', type=int, default=1, help='Arm index (default: 1)')
    parser.add_argument(
        '--mode',
        choices=['pick', 'swap', 'place', 'drop'],
        default='pick',
        help='Sequence mode (default: pick)',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=120.0,
        help='Result wait timeout in seconds (default: 120)',
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = ArmSequenceClient()
    try:
        exit_code = node.run(args.arm, args.mode, args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
