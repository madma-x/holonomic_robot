"""actuator_sequencer.py — Orchestrates ordered actuator sequences.

Exposes an ExecuteSequence action server. For each step it calls:
  - MoveServo action client   (→ feetech_node, Feetech USB servo)
  - ControlPump action client (→ i2c_node)
  - MOVE_PWM_SERVO topic publish (→ i2c_node /pwm_servo_cmd, fire-and-forget)
"""

import threading
import time
from typing import Iterator

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from robot_actuators.action import ControlPump, ExecuteSequence, MoveServo
from robot_actuators.msg import ActuatorStep
from robot_hw_interfaces.msg import PwmServoCommand, SafetyState

_SAFETY_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ActuatorSequencerNode(Node):
    def __init__(self):
        super().__init__('actuator_sequencer')

        self.declare_parameter('move_servo_action', 'move_servo')
        self.declare_parameter('control_pump_action', 'control_pump')
        self.declare_parameter('pwm_servo_topic', '/pwm_servo_cmd')
        self.declare_parameter('step_timeout_sec', 15.0)
        self.declare_parameter('safety_timeout_sec', 0.5)

        self._step_timeout = self.get_parameter('step_timeout_sec').value
        self._safety_timeout = self.get_parameter('safety_timeout_sec').value

        self._move_client = ActionClient(
            self, MoveServo, self.get_parameter('move_servo_action').value,
        )
        self._pump_client = ActionClient(
            self, ControlPump, self.get_parameter('control_pump_action').value,
        )

        self._pwm_pub = self.create_publisher(
            PwmServoCommand,
            self.get_parameter('pwm_servo_topic').value,
            10,
        )

        self._system_safe = False
        self._last_safety_stamp = time.monotonic()
        self._safety_lock = threading.Lock()

        self._safety_sub = self.create_subscription(
            SafetyState, '/safety_state', self._safety_cb, _SAFETY_QOS
        )
        self._watchdog_timer = self.create_timer(0.1, self._safety_watchdog)

        self._server = ActionServer(
            self,
            ExecuteSequence,
            'execute_sequence',
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            execute_callback=self._execute_cb,
        )

        self.get_logger().info('actuator_sequencer ready.')

    def _safety_cb(self, msg: SafetyState):
        safe = (msg.state == SafetyState.SAFE_ON)
        with self._safety_lock:
            self._system_safe = safe
            self._last_safety_stamp = time.monotonic()

    def _safety_watchdog(self):
        with self._safety_lock:
            age = time.monotonic() - self._last_safety_stamp
            if age > self._safety_timeout:
                self._system_safe = False

    def _is_safe(self) -> bool:
        with self._safety_lock:
            return self._system_safe

    def _goal_cb(self, _goal_request):
        if not self._is_safe():
            self.get_logger().warn('ExecuteSequence rejected — system not safe.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle: ServerGoalHandle):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle: ServerGoalHandle):
        steps: list = list(goal_handle.request.steps)
        total = len(steps)
        result = ExecuteSequence.Result()
        feedback = ExecuteSequence.Feedback()
        feedback.total_steps = total

        self.get_logger().info(f'ExecuteSequence: starting {total} steps.')

        flat_idx = 0
        for batch in self._group_steps(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.failed_step = flat_idx
                result.message = f'Cancelled at step {flat_idx}.'
                return result

            if not self._is_safe():
                goal_handle.abort()
                result.success = False
                result.failed_step = flat_idx
                result.message = f'Safety cut at step {flat_idx}.'
                return result

            feedback.current_step = flat_idx
            feedback.step_description = ' | '.join(
                self._step_description(s) for s in batch
            )
            goal_handle.publish_feedback(feedback)
            tag = f'[parallel_group={batch[0].parallel_group}]' if len(batch) > 1 else ''
            self.get_logger().info(
                f'  Steps {flat_idx}–{flat_idx + len(batch) - 1}{tag}: '
                f'{feedback.step_description}'
            )

            ok, failed_offset = self._run_batch(batch)
            if not ok:
                goal_handle.abort()
                result.success = False
                result.failed_step = flat_idx + failed_offset
                result.message = (
                    f'Step {flat_idx + failed_offset} failed: '
                    f'{self._step_description(batch[failed_offset])}'
                )
                self.get_logger().error(result.message)
                return result

            flat_idx += len(batch)

        result.success = True
        result.failed_step = 0
        result.message = f'All {total} steps completed.'
        goal_handle.succeed()
        self.get_logger().info('ExecuteSequence: completed successfully.')
        return result

    def _group_steps(self, steps: list) -> Iterator[list]:
        i = 0
        while i < len(steps):
            group = steps[i].parallel_group
            if group == 0:
                yield [steps[i]]
                i += 1
            else:
                batch = []
                while i < len(steps) and steps[i].parallel_group == group:
                    batch.append(steps[i])
                    i += 1
                yield batch

    def _run_batch(self, batch: list) -> tuple:
        if len(batch) == 1:
            ok = self._run_step(batch[0])
            return (ok, 0) if not ok else (True, 0)

        results = [None] * len(batch)

        def worker(idx, step):
            results[idx] = self._run_step(step)

        threads = [
            threading.Thread(target=worker, args=(i, s), daemon=True)
            for i, s in enumerate(batch)
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=self._step_timeout + 5.0)

        for i, ok in enumerate(results):
            if not ok:
                return False, i
        return True, 0

    def _run_step(self, step: ActuatorStep) -> bool:
        if step.step_type == ActuatorStep.MOVE_SERVO:
            return self._run_move_servo(step)
        if step.step_type == ActuatorStep.CONTROL_PUMP:
            return self._run_control_pump(step)
        if step.step_type == ActuatorStep.MOVE_PWM_SERVO:
            return self._run_pwm_servo(step)
        self.get_logger().error(f'Unknown step_type {step.step_type}')
        return False

    def _run_move_servo(self, step: ActuatorStep) -> bool:
        if not self._move_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('MoveServo action server not available.')
            return False

        goal = MoveServo.Goal()
        goal.servo_id = step.servo_id
        goal.target_deg = step.target_deg
        goal.speed_deg_s = step.speed_deg_s

        future = self._move_client.send_goal_async(goal)
        if not self._wait(future, self._step_timeout):
            self.get_logger().error('MoveServo goal send timed out.')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveServo goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait(result_future, self._step_timeout):
            self.get_logger().error('MoveServo result timed out.')
            return False

        result = result_future.result()
        if result is None:
            return False
        return bool(result.result.success)

    def _run_control_pump(self, step: ActuatorStep) -> bool:
        if not self._pump_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('ControlPump action server not available.')
            return False

        goal = ControlPump.Goal()
        goal.pump_id = step.pump_id
        goal.enable = step.pump_enable
        goal.duty_cycle = 1.0
        goal.duration_sec = step.pump_duration_sec

        future = self._pump_client.send_goal_async(goal)
        if not self._wait(future, self._step_timeout):
            self.get_logger().error('ControlPump goal send timed out.')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('ControlPump goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        timeout = max(self._step_timeout, step.pump_duration_sec + 5.0)
        if not self._wait(result_future, timeout):
            self.get_logger().error('ControlPump result timed out.')
            return False

        result = result_future.result()
        if result is None:
            return False
        return bool(result.result.success)

    def _run_pwm_servo(self, step: ActuatorStep) -> bool:
        msg = PwmServoCommand()
        msg.channel = step.pwm_channel
        msg.enable = True
        msg.target_deg = step.pwm_target_deg
        self._pwm_pub.publish(msg)

        if step.pwm_settle_sec > 0.0:
            time.sleep(step.pwm_settle_sec)
        return True

    @staticmethod
    def _step_description(step: ActuatorStep) -> str:
        if step.step_type == ActuatorStep.MOVE_SERVO:
            return f'MOVE_SERVO servo={step.servo_id} → {step.target_deg:.1f}°'
        if step.step_type == ActuatorStep.MOVE_PWM_SERVO:
            return f'MOVE_PWM_SERVO ch={step.pwm_channel} → {step.pwm_target_deg:.1f}°'
        action = 'ON' if step.pump_enable else 'OFF'
        return f'CONTROL_PUMP pump={step.pump_id} {action}'

    def _wait(self, future, timeout_sec: float, poll: float = 0.02) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(poll)
        return future.done()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorSequencerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()