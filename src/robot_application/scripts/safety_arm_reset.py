#!/usr/bin/env python3
"""Trigger arm reset sequence on SAFE_OFF -> SAFE_ON transitions."""

import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from robot_actuators.action import ExecuteSequence
from robot_application.arm_layout import ARM_SELECTION_PRIORITY
from robot_application.arm_sequences import ArmSequenceBuilder
from robot_hw_interfaces.msg import SafetyState


_SAFETY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class SafetyArmResetNode(Node):
    def __init__(self):
        super().__init__('safety_arm_reset')

        self.declare_parameter('safety_topic', '/safety_state')
        self.declare_parameter('execute_server', 'execute_sequence')
        self.declare_parameter('action_server_wait_sec', 2.0)
        self.declare_parameter('action_result_timeout_sec', 30.0)

        self._safe_last = None
        self._reset_in_progress = False
        self._lock = threading.Lock()

        self._builder = ArmSequenceBuilder()
        self._client = ActionClient(
            self,
            ExecuteSequence,
            str(self.get_parameter('execute_server').value),
        )

        self._sub = self.create_subscription(
            SafetyState,
            str(self.get_parameter('safety_topic').value),
            self._safety_cb,
            _SAFETY_QOS,
        )

        self.get_logger().info('Safety arm reset node ready.')

    def _safety_cb(self, msg: SafetyState) -> None:
        safe_now = (msg.state == SafetyState.SAFE_ON)

        with self._lock:
            safe_prev = self._safe_last
            self._safe_last = safe_now

            # Trigger only on edge SAFE_OFF -> SAFE_ON.
            should_trigger = (safe_prev is False) and safe_now and (not self._reset_in_progress)
            if should_trigger:
                self._reset_in_progress = True

        if should_trigger:
            self.get_logger().info('Safety transitioned to SAFE_ON: triggering arm reset sequence.')
            threading.Thread(target=self._send_reset_goal, daemon=True).start()

    def _send_reset_goal(self) -> None:
        wait_sec = float(self.get_parameter('action_server_wait_sec').value)
        if not self._client.wait_for_server(timeout_sec=wait_sec):
            self.get_logger().warn('execute_sequence action server unavailable; skipping auto reset.')
            with self._lock:
                self._reset_in_progress = False
            return

        steps = self._builder.build_reset_sequence(list(ARM_SELECTION_PRIORITY))
        if not steps:
            self.get_logger().warn('Reset sequence produced no steps; skipping.')
            with self._lock:
                self._reset_in_progress = False
            return

        goal = ExecuteSequence.Goal()
        goal.steps = steps

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Auto reset goal was rejected.')
            with self._lock:
                self._reset_in_progress = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            if result.success:
                self.get_logger().info('Auto arm reset completed successfully after SAFE_ON.')
            else:
                self.get_logger().warn(
                    f'Auto arm reset failed at step {result.failed_step}: {result.message}'
                )
        finally:
            with self._lock:
                self._reset_in_progress = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyArmResetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
