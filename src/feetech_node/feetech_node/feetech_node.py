"""
feetech_node.py — Feetech serial servo node (RPi4 USB).

Responsibilities
----------------
* Discover all configured servo IDs on SAFE_ON edge (staggered, 150 ms/servo).
* Serve MoveServo and ControlGripper action requests.
* Publish /servo_state at 20 Hz for each known servo.
* Subscribe to /safety_state — torque-disable all servos on SAFE_OFF.

Library used: scservo_sdk (pip install scservo_sdk)
This SDK supports both SMS/STS and SCS series Feetech servos.

Servo control
-------------
* SMS/STS series (our primary target):
    PortHandler + PacketHandler (protocol 0)
    or the higher-level STS class from scservo_sdk.
* We use the lower-level PacketHandler API so the driver works across
  the full SCS/SMS/STS range without needing a specific subclass.

Register map used (SMS_STS / SCS common subset):
    0x18  TORQUE_ENABLE  (1 byte)  1 = torque on, 0 = torque off
    0x2A  GOAL_POSITION  (2 bytes, little-endian)
    0x2C  GOAL_SPEED     (2 bytes)
    0x38  PRESENT_POSITION (2 bytes)
    0x3A  PRESENT_SPEED    (2 bytes)

Parameters (override via config YAML)
--------------------------------------
    port            str  '/dev/ttyUSB0'
    baud_rate       int  1000000
    servo_state_hz  float 20.0
    safety_timeout_sec float 0.5
    discovery_delay_ms int 500     — wait after SAFE_ON before scanning
    per_servo_delay_ms int 150     — interval between each servo's init ping

servo_ids: [0, 1, 2, ...]          — servo IDs to manage
gripper_map: {gripper_id: servo_id, ...}  — up to 8 entries via flat params

Each servo's full range in ticks and degree mapping is symmetric around
centre: ticks 0–4095 → 0°–300° for most Feetech STS servos.
Set degree_per_tick via param if your servo has a different spec.
"""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from robot_actuators.action import ControlGripper, MoveServo
from robot_hw_interfaces.msg import SafetyState, ServoState

try:
    from scservo_sdk import PacketHandler, PortHandler, COMM_SUCCESS
    _SDK_AVAILABLE = True
except ImportError:
    _SDK_AVAILABLE = False

_SAFETY_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Register addresses (SMS/STS series)
_REG_TORQUE_ENABLE = 0x18
_REG_GOAL_POSITION = 0x2A
_REG_GOAL_SPEED = 0x2C
_REG_PRESENT_POSITION = 0x38
_REG_PRESENT_SPEED = 0x3A

# Servo physical range
_TICK_MAX = 4095
_DEG_RANGE = 300.0       # full range in degrees (typical STS servo)
_TICKS_PER_DEG = _TICK_MAX / _DEG_RANGE


def _deg_to_ticks(deg: float) -> int:
    return max(0, min(_TICK_MAX, round(deg * _TICKS_PER_DEG)))


def _ticks_to_deg(ticks: int) -> float:
    return ticks / _TICKS_PER_DEG


def _speed_to_ticks(speed_deg_s: float) -> int:
    """Convert angular speed (deg/s) to servo speed register value."""
    ticks_per_s = abs(speed_deg_s) * _TICKS_PER_DEG
    # The STS speed register unit is approximately 0.732 deg/s per LSB.
    val = round(ticks_per_s / 0.732)
    return max(1, min(0x7FFF, val))


class ServoEntry:
    def __init__(self, servo_id: int):
        self.servo_id = servo_id
        self.online = False
        self.disabled_by_safety = False
        self.position_deg: float = 0.0
        self.speed_deg_s: float = 0.0
        self.torque_enabled: bool = False
        self.lock = threading.Lock()


class FeetechNode(Node):
    def __init__(self):
        super().__init__('feetech_node')

        # ---- params ----
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('servo_state_hz', 20.0)
        self.declare_parameter('safety_timeout_sec', 0.5)
        self.declare_parameter('discovery_delay_ms', 500)
        self.declare_parameter('per_servo_delay_ms', 150)
        self.declare_parameter('degree_per_tick', _DEG_RANGE / _TICK_MAX)

        port_name = self.get_parameter('port').value
        baud = self.get_parameter('baud_rate').value
        state_hz = self.get_parameter('servo_state_hz').value
        self._safety_timeout = self.get_parameter('safety_timeout_sec').value
        self._discovery_delay = self.get_parameter('discovery_delay_ms').value / 1000.0
        self._per_servo_delay = self.get_parameter('per_servo_delay_ms').value / 1000.0

        # servo_ids list: flat params servo_id_0 … servo_id_N
        servo_ids: list[int] = []
        for n in range(32):
            self.declare_parameter(f'servo_id_{n}', -1)
            sid = self.get_parameter(f'servo_id_{n}').value
            if sid >= 0:
                servo_ids.append(sid)

        # gripper_map: gripper_<g>_servo_id
        self._gripper_map: dict[int, int] = {}
        for g in range(8):
            self.declare_parameter(f'gripper_{g}_servo_id', -1)
            s = self.get_parameter(f'gripper_{g}_servo_id').value
            if s >= 0:
                self._gripper_map[g] = s

        self._servos: dict[int, ServoEntry] = {sid: ServoEntry(sid) for sid in servo_ids}
        self._servo_lock = threading.Lock()

        # ---- SDK / port ----
        self._port: Optional[object] = None
        self._ph: Optional[object] = None
        self._hw_ok = False

        if _SDK_AVAILABLE:
            try:
                self._port = PortHandler(port_name)
                self._ph = PacketHandler(0)  # protocol version 0 (Feetech)
                if self._port.openPort() and self._port.setBaudRate(baud):
                    self._hw_ok = True
                    self.get_logger().info(f'Feetech port {port_name} @ {baud} baud — OK')
                else:
                    self.get_logger().error(f'Failed to open {port_name}')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'SDK init error: {exc}')
        else:
            self.get_logger().warn('scservo_sdk not found — running in simulation mode.')

        # ---- safety ----
        self._system_safe = False
        self._prev_system_safe = False
        self._last_safety_stamp = time.monotonic()
        self._safety_lock = threading.Lock()

        self._safety_sub = self.create_subscription(
            SafetyState, '/safety_state', self._safety_cb, _SAFETY_QOS
        )

        # ---- publishers / timers ----
        self._state_pub = self.create_publisher(ServoState, '/servo_state', 10)
        self._state_timer = self.create_timer(1.0 / state_hz, self._publish_servo_states)
        self._watchdog_timer = self.create_timer(0.1, self._safety_watchdog)

        # ---- action servers ----
        self._move_server = ActionServer(
            self, MoveServo, 'move_servo',
            goal_callback=self._move_goal_cb,
            cancel_callback=self._cancel_cb,
            execute_callback=self._move_execute_cb,
        )
        self._gripper_server = ActionServer(
            self, ControlGripper, 'control_gripper',
            goal_callback=self._gripper_goal_cb,
            cancel_callback=self._cancel_cb,
            execute_callback=self._gripper_execute_cb,
        )

        self.get_logger().info('feetech_node initialised.')

    # ----------------------------------------------------------- Safety --

    def _safety_cb(self, msg: SafetyState):
        safe = (msg.state == SafetyState.SAFE_ON)
        with self._safety_lock:
            self._last_safety_stamp = time.monotonic()
            prev = self._system_safe
            self._system_safe = safe

        if prev and not safe:
            self.get_logger().warn('Safety SAFE_OFF — disabling all servo torque.')
            self._torque_all(False, disabled_by_safety=True)
        elif not prev and safe:
            self.get_logger().info('Safety SAFE_ON — starting servo discovery.')
            t = threading.Thread(target=self._discover_servos, daemon=True)
            t.start()

    def _safety_watchdog(self):
        with self._safety_lock:
            age = time.monotonic() - self._last_safety_stamp
            was_safe = self._system_safe
            if age > self._safety_timeout and was_safe:
                self._system_safe = False
                self.get_logger().warn(
                    f'Safety watchdog timeout ({age:.2f}s) — cutting torque.'
                )
                self._torque_all(False, disabled_by_safety=True)

    # ----------------------------------------------------------- Discovery --

    def _discover_servos(self):
        time.sleep(self._discovery_delay)
        for sid, entry in self._servos.items():
            time.sleep(self._per_servo_delay)
            if not self._is_safe():
                return
            ok = self._ping(sid)
            with entry.lock:
                entry.online = ok
                entry.disabled_by_safety = False
                if ok:
                    entry.torque_enabled = True
                    self._write_byte(sid, _REG_TORQUE_ENABLE, 1)
                    # Read current position.
                    pos, _ = self._read_position(sid)
                    if pos is not None:
                        entry.position_deg = pos
            status = 'online' if ok else 'offline'
            self.get_logger().info(f'Servo {sid}: {status}')

    # ----------------------------------------------------------- Low-level --

    def _ping(self, servo_id: int) -> bool:
        if not self._hw_ok:
            return True  # simulation: always online
        _, result, _ = self._ph.ping(self._port, servo_id)
        return result == COMM_SUCCESS

    def _write_byte(self, servo_id: int, addr: int, value: int) -> bool:
        if not self._hw_ok:
            return True
        result, error = self._ph.write1ByteTxRx(self._port, servo_id, addr, value)
        return result == COMM_SUCCESS and error == 0

    def _write_word(self, servo_id: int, addr: int, value: int) -> bool:
        if not self._hw_ok:
            return True
        result, error = self._ph.write2ByteTxRx(self._port, servo_id, addr, value)
        return result == COMM_SUCCESS and error == 0

    def _read_position(self, servo_id: int) -> tuple[Optional[float], Optional[float]]:
        """Returns (position_deg, speed_deg_s) or (None, None) on failure."""
        if not self._hw_ok:
            return None, None
        pos_ticks, result, _ = self._ph.read2ByteTxRx(
            self._port, servo_id, _REG_PRESENT_POSITION
        )
        if result != COMM_SUCCESS:
            return None, None
        spd_ticks, result, _ = self._ph.read2ByteTxRx(
            self._port, servo_id, _REG_PRESENT_SPEED
        )
        if result != COMM_SUCCESS:
            spd_ticks = 0
        return _ticks_to_deg(pos_ticks), _ticks_to_deg(spd_ticks) * 0.732

    def _torque_all(self, enable: bool, disabled_by_safety: bool = False):
        for sid, entry in self._servos.items():
            with entry.lock:
                if entry.online:
                    self._write_byte(sid, _REG_TORQUE_ENABLE, 1 if enable else 0)
                    entry.torque_enabled = enable
                    if disabled_by_safety:
                        entry.disabled_by_safety = True

    def _is_safe(self) -> bool:
        with self._safety_lock:
            return self._system_safe

    # ---------------------------------------------------- State publisher --

    def _publish_servo_states(self):
        for sid, entry in self._servos.items():
            with entry.lock:
                if entry.disabled_by_safety:
                    state_val = ServoState.DISABLED_BY_SAFETY
                elif entry.online:
                    state_val = ServoState.ONLINE
                else:
                    state_val = ServoState.OFFLINE

                if self._hw_ok and entry.online:
                    pos, spd = self._read_position(sid)
                    if pos is not None:
                        entry.position_deg = pos
                        entry.speed_deg_s = spd if spd is not None else 0.0

                msg = ServoState()
                msg.servo_id = sid
                msg.state = state_val
                msg.torque_enabled = entry.torque_enabled
                msg.position_deg = entry.position_deg
                msg.speed_deg_s = entry.speed_deg_s

            self._state_pub.publish(msg)

    # ------------------------------------------------------- MoveServo --

    def _move_goal_cb(self, goal_request):
        if goal_request.servo_id not in self._servos:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _gripper_goal_cb(self, goal_request):
        if goal_request.servo_id not in self._gripper_map.values():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _move_execute_cb(self, goal_handle: ServerGoalHandle):
        goal = goal_handle.request
        return self._execute_move(
            goal_handle,
            servo_id=goal.servo_id,
            target_deg=goal.target_deg,
            speed_deg_s=goal.speed_deg_s,
            result_cls=MoveServo.Result,
            feedback_cls=MoveServo.Feedback,
        )

    def _gripper_execute_cb(self, goal_handle: ServerGoalHandle):
        goal = goal_handle.request
        return self._execute_move(
            goal_handle,
            servo_id=goal.servo_id,
            target_deg=goal.target_deg,
            speed_deg_s=goal.speed_deg_s,
            result_cls=ControlGripper.Result,
            feedback_cls=ControlGripper.Feedback,
        )

    def _execute_move(
        self,
        goal_handle: ServerGoalHandle,
        servo_id: int,
        target_deg: float,
        speed_deg_s: float,
        result_cls,
        feedback_cls,
    ):
        result = result_cls()
        entry = self._servos.get(servo_id)

        if entry is None or not entry.online:
            result.success = False
            result.message = f'Servo {servo_id} not online.'
            goal_handle.abort()
            return result

        if not self._is_safe():
            result.success = False
            result.message = 'System not safe.'
            goal_handle.abort()
            return result

        # Send movement command.
        goal_ticks = _deg_to_ticks(target_deg)
        speed_ticks = _speed_to_ticks(speed_deg_s)

        self._write_word(servo_id, _REG_GOAL_SPEED, speed_ticks)
        self._write_word(servo_id, _REG_GOAL_POSITION, goal_ticks)

        feedback_msg = feedback_cls()
        tolerance_deg = 2.0
        timeout = abs(target_deg - entry.position_deg) / max(1.0, abs(speed_deg_s)) + 3.0
        start = time.monotonic()

        while time.monotonic() - start < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Cancelled.'
                return result

            if not self._is_safe():
                goal_handle.abort()
                result.success = False
                result.message = 'Safety cut during move.'
                return result

            pos, _ = self._read_position(servo_id)
            if pos is not None:
                with entry.lock:
                    entry.position_deg = pos
                feedback_msg.current_deg = pos
                goal_handle.publish_feedback(feedback_msg)

                if abs(pos - target_deg) <= tolerance_deg:
                    break

            time.sleep(0.02)

        with entry.lock:
            final = entry.position_deg

        result.success = True
        result.final_deg = final
        result.message = 'OK'
        goal_handle.succeed()
        return result

    # --------------------------------------------------------- Cleanup --

    def destroy_node(self):
        self._torque_all(False)
        if self._port and self._hw_ok:
            self._port.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FeetechNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
