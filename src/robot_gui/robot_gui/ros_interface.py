"""ROS interface layer for the robot GUI."""

from __future__ import annotations

import threading
import time
import math
from dataclasses import dataclass, field
from typing import Dict

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger

from robot_hw_interfaces.msg import PumpState, SafetyState, ServoState


@dataclass
class GuiSnapshot:
    """Thread-safe snapshot of ROS-backed GUI data."""

    time_remaining: float = 0.0
    score: int = 0
    phase: str = "SETUP"
    match_active: bool = False
    current_task: str = "-"
    queue_size: int = 0
    mission_status: str = "UNKNOWN"

    safety_state: str = "UNKNOWN"
    servo_summary: str = "No data"
    pump_summary: str = "No data"

    service_feedback: str = "Ready"

    last_update: Dict[str, float] = field(default_factory=dict)


class RobotGuiRosInterface(Node):
    """ROS subscriptions and service clients for GUI controls."""

    def __init__(self):
        super().__init__('robot_gui')

        self.declare_parameter('window_title', 'Holonomic Robot Match GUI')
        self.declare_parameter('fullscreen', False)
        self.declare_parameter('min_width', 480)
        self.declare_parameter('min_height', 800)
        self.declare_parameter('poll_interval_ms', 100)

        self.declare_parameter('game_start_service', '/game/start_match')
        self.declare_parameter('game_stop_service', '/game/stop_match')
        self.declare_parameter('planner_start_service', '/planner/start')
        self.declare_parameter('planner_stop_service', '/planner/stop')
        self.declare_parameter('planner_replan_service', '/planner/replan')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('initial_pose_frame', 'map')
        self.declare_parameter('initial_pose_x', 0.2)
        self.declare_parameter('initial_pose_y', 0.2)
        self.declare_parameter('initial_pose_yaw', 0.0)

        self.declare_parameter('topic_time_remaining', '/game/time_remaining')
        self.declare_parameter('topic_score', '/game/score')
        self.declare_parameter('topic_phase', '/game/phase')
        self.declare_parameter('topic_match_active', '/game/match_active')
        self.declare_parameter('topic_current_task', '/planner/current_task')
        self.declare_parameter('topic_queue_size', '/planner/queue_size')
        self.declare_parameter('topic_mission_status', '/mission_executor/mission_status')
        self.declare_parameter('topic_safety_state', '/safety_state')
        self.declare_parameter('topic_servo_state', '/servo_state')
        self.declare_parameter('topic_pump_state', '/pump_state')

        self.declare_parameter('stale_timeout_sec', 1.5)

        self._lock = threading.Lock()
        self._snapshot = GuiSnapshot()
        self._stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)

        self._servo_online = 0
        self._servo_total = 0
        self._pump_on = 0
        self._pump_total = 0
        self._servo_states: Dict[int, int] = {}
        self._pump_states: Dict[int, int] = {}

        self._clients = {
            'start_match': self.create_client(Trigger, str(self.get_parameter('game_start_service').value)),
            'stop_match': self.create_client(Trigger, str(self.get_parameter('game_stop_service').value)),
            'planner_start': self.create_client(Trigger, str(self.get_parameter('planner_start_service').value)),
            'planner_stop': self.create_client(Trigger, str(self.get_parameter('planner_stop_service').value)),
            'planner_replan': self.create_client(Trigger, str(self.get_parameter('planner_replan_service').value)),
        }
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initial_pose_topic').value),
            10,
        )

        self.create_subscription(
            Float32,
            str(self.get_parameter('topic_time_remaining').value),
            self._time_cb,
            10,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter('topic_score').value),
            self._score_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('topic_phase').value),
            self._phase_cb,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('topic_match_active').value),
            self._active_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('topic_current_task').value),
            self._task_cb,
            10,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter('topic_queue_size').value),
            self._queue_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('topic_mission_status').value),
            self._mission_cb,
            10,
        )
        self.create_subscription(
            SafetyState,
            str(self.get_parameter('topic_safety_state').value),
            self._safety_cb,
            10,
        )
        self.create_subscription(
            ServoState,
            str(self.get_parameter('topic_servo_state').value),
            self._servo_cb,
            10,
        )
        self.create_subscription(
            PumpState,
            str(self.get_parameter('topic_pump_state').value),
            self._pump_cb,
            10,
        )

    def snapshot(self) -> GuiSnapshot:
        """Get a copy of current state, marking stale fields where needed."""
        with self._lock:
            snap = GuiSnapshot(
                time_remaining=self._snapshot.time_remaining,
                score=self._snapshot.score,
                phase=self._snapshot.phase,
                match_active=self._snapshot.match_active,
                current_task=self._snapshot.current_task,
                queue_size=self._snapshot.queue_size,
                mission_status=self._snapshot.mission_status,
                safety_state=self._snapshot.safety_state,
                servo_summary=self._snapshot.servo_summary,
                pump_summary=self._snapshot.pump_summary,
                service_feedback=self._snapshot.service_feedback,
                last_update=dict(self._snapshot.last_update),
            )
        self._apply_stale_flags(snap)
        return snap

    def call_named_service(self, service_key: str):
        """Call a known Trigger service asynchronously."""
        client = self._clients.get(service_key)
        if client is None:
            self._set_feedback(f'Unknown action: {service_key}')
            return

        if not client.wait_for_service(timeout_sec=0.2):
            self._set_feedback(f'Service unavailable: {service_key}')
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._on_service_result(service_key, f))
        self._set_feedback(f'Calling {service_key}...')

    def reset_to_initial_position(self):
        """Publish initial pose so odometry/localization returns to configured start."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('initial_pose_frame').value)

        x = float(self.get_parameter('initial_pose_x').value)
        y = float(self.get_parameter('initial_pose_y').value)
        yaw = float(self.get_parameter('initial_pose_yaw').value)

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        half_yaw = yaw * 0.5
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.09

        self._initial_pose_pub.publish(msg)
        self._set_feedback(
            f'Initial pose published: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad'
        )

    @property
    def poll_interval_ms(self) -> int:
        """UI refresh interval from parameters."""
        return int(self.get_parameter('poll_interval_ms').value)

    def _on_service_result(self, service_key: str, future):
        try:
            result = future.result()
        except Exception as error:  # noqa: BLE001
            self._set_feedback(f'{service_key} failed: {error}')
            return

        if result is None:
            self._set_feedback(f'{service_key} failed: no response')
            return

        prefix = 'OK' if result.success else 'FAIL'
        self._set_feedback(f'{prefix} {service_key}: {result.message}')

    def _mark_update(self, field: str):
        with self._lock:
            self._snapshot.last_update[field] = time.monotonic()

    def _set_feedback(self, message: str):
        with self._lock:
            self._snapshot.service_feedback = message

    def _time_cb(self, msg: Float32):
        with self._lock:
            self._snapshot.time_remaining = float(msg.data)
        self._mark_update('time')

    def _score_cb(self, msg: Int32):
        with self._lock:
            self._snapshot.score = int(msg.data)
        self._mark_update('score')

    def _phase_cb(self, msg: String):
        with self._lock:
            self._snapshot.phase = str(msg.data)
        self._mark_update('phase')

    def _active_cb(self, msg: Bool):
        with self._lock:
            self._snapshot.match_active = bool(msg.data)
        self._mark_update('match_active')

    def _task_cb(self, msg: String):
        with self._lock:
            self._snapshot.current_task = str(msg.data)
        self._mark_update('current_task')

    def _queue_cb(self, msg: Int32):
        with self._lock:
            self._snapshot.queue_size = int(msg.data)
        self._mark_update('queue_size')

    def _mission_cb(self, msg: String):
        with self._lock:
            self._snapshot.mission_status = str(msg.data)
        self._mark_update('mission_status')

    def _safety_cb(self, msg: SafetyState):
        if msg.state == SafetyState.SAFE_ON:
            state = 'SAFE_ON'
        elif msg.state == SafetyState.SAFE_OFF:
            state = 'SAFE_OFF'
        elif msg.state == SafetyState.RECOVERING:
            state = 'RECOVERING'
        else:
            state = 'UNKNOWN'

        with self._lock:
            self._snapshot.safety_state = state
        self._mark_update('safety')

    def _servo_cb(self, msg: ServoState):
        with self._lock:
            self._servo_states[int(msg.servo_id)] = int(msg.state)
            self._servo_total = len(self._servo_states)
            self._servo_online = sum(1 for state in self._servo_states.values() if state == ServoState.ONLINE)
            if self._servo_total > 0:
                self._snapshot.servo_summary = f'{self._servo_online}/{self._servo_total} online'
            else:
                self._snapshot.servo_summary = 'No data'
        self._mark_update('servo')

    def _pump_cb(self, msg: PumpState):
        with self._lock:
            self._pump_states[int(msg.pump_id)] = int(msg.state)
            self._pump_total = len(self._pump_states)
            self._pump_on = sum(1 for state in self._pump_states.values() if state == PumpState.ON)
            if self._pump_total > 0:
                self._snapshot.pump_summary = f'{self._pump_on}/{self._pump_total} active'
            else:
                self._snapshot.pump_summary = 'No data'
        self._mark_update('pump')

    def _apply_stale_flags(self, snap: GuiSnapshot):
        now = time.monotonic()
        stale = self._stale_timeout_sec
        last = snap.last_update

        if now - last.get('safety', 0.0) > stale:
            snap.safety_state = f'{snap.safety_state} (stale)'
        if now - last.get('servo', 0.0) > stale:
            snap.servo_summary = f'{snap.servo_summary} (stale)'
        if now - last.get('pump', 0.0) > stale:
            snap.pump_summary = f'{snap.pump_summary} (stale)'


def start_executor_thread(node: Node) -> tuple[SingleThreadedExecutor, threading.Thread]:
    """Spin a node in a background thread."""
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def _spin():
        executor.spin()

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    return executor, thread
