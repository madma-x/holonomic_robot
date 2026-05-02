"""
safety_node.py — Safety authority node (runs on LattePanda).

Reads a GPIO line (e.g. a physical safety key/switch) and publishes
robot_hw_interfaces/SafetyState at a configurable rate plus on every
detected edge.

Downstream nodes (i2c_node, feetech_node) subscribe to /safety_state and
must immediately disable actuators when SafetyState.state == SAFE_OFF.

Parameters (declared, can be overridden via config YAML):
    gpio_chip     (str,  default '/dev/gpiochip0') — Linux character device
    gpio_line     (int,  default 18)               — GPIO line number
    active_low    (bool, default false)            — invert signal polarity
    debounce_ms   (int,  default 50)               — edge debounce window
    publish_rate_hz (float, default 20.0)          — periodic publish rate
    recovery_timeout_sec (float, default 2.0)      — how long RECOVERING lasts
    start_latch_enabled (bool, default true)       — enable match-start latch
    start_latch_port (str, default '/dev/ttyACM0') — MicroPython REPL port
    start_latch_baudrate (int, default 115200)
    start_latch_pin (int, default 28)              — RP2040 GPIO latch input
    start_latch_pullup (bool, default true)        — configure Pin.PULL_UP
    start_latch_trigger_value (int, default 0)     — trigger match when pin equals this value
    start_latch_debounce_ms (int, default 50)      — debounce for latch edge
    start_match_service (str, default '/game/start_match')
    match_active_topic (str, default '/game/match_active')
    require_safe_on_for_match_start (bool, default true)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from robot_hw_interfaces.msg import SafetyState

try:
    import gpiod
    _GPIOD_AVAILABLE = True
except ImportError:
    _GPIOD_AVAILABLE = False

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('gpio_chip', '/dev/gpiochip0')
        self.declare_parameter('gpio_line', 18)
        self.declare_parameter('active_low', False)
        self.declare_parameter('debounce_ms', 50)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('recovery_timeout_sec', 2.0)
        self.declare_parameter('start_latch_enabled', True)
        self.declare_parameter('start_latch_port', '/dev/ttyACM0')
        self.declare_parameter('start_latch_baudrate', 115200)
        self.declare_parameter('start_latch_pin', 28)
        self.declare_parameter('start_latch_pullup', True)
        self.declare_parameter('start_latch_trigger_value', 0)
        self.declare_parameter('start_latch_debounce_ms', 50)
        self.declare_parameter('start_match_service', '/game/start_match')
        self.declare_parameter('match_active_topic', '/game/match_active')
        self.declare_parameter('require_safe_on_for_match_start', True)

        self._chip_path = self.get_parameter('gpio_chip').value
        self._line_num = self.get_parameter('gpio_line').value
        self._active_low = self.get_parameter('active_low').value
        self._debounce_ms = self.get_parameter('debounce_ms').value
        self._rate_hz = self.get_parameter('publish_rate_hz').value
        self._recovery_timeout = self.get_parameter('recovery_timeout_sec').value
        self._start_latch_enabled = self.get_parameter('start_latch_enabled').value
        self._start_latch_port = self.get_parameter('start_latch_port').value
        self._start_latch_baudrate = self.get_parameter('start_latch_baudrate').value
        self._start_latch_pin_num = self.get_parameter('start_latch_pin').value
        self._start_latch_pullup = self.get_parameter('start_latch_pullup').value
        self._start_latch_trigger_value = int(
            self.get_parameter('start_latch_trigger_value').value
        )
        self._start_latch_debounce_ms = self.get_parameter('start_latch_debounce_ms').value
        self._start_match_service = self.get_parameter('start_match_service').value
        self._match_active_topic = self.get_parameter('match_active_topic').value
        self._require_safe_on = self.get_parameter('require_safe_on_for_match_start').value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(SafetyState, '/safety_state', qos)

        self._lock = threading.Lock()
        self._state = SafetyState.SAFE_OFF
        self._recovery_deadline: float | None = None
        self._last_edge_time = 0.0
        self._last_start_latch_edge_time = 0.0
        self._last_start_latch_value: bool | None = None
        self._match_active = False
        self._pending_start_future = None

        self._line = None
        self._gpio_ok = False
        self._start_latch_ok = False
        self._start_latch_serial = None

        self._start_match_client = self.create_client(
            Trigger,
            str(self._start_match_service),
        )
        self.create_subscription(
            Bool,
            str(self._match_active_topic),
            self._match_active_cb,
            10,
        )

        self._init_gpio()
        self._init_start_latch()

        period = 1.0 / self._rate_hz
        self._timer = self.create_timer(period, self._publish_state)

        if self._gpio_ok:
            self._monitor_thread = threading.Thread(
                target=self._gpio_monitor_loop,
                daemon=True,
            )
            self._monitor_thread.start()
        else:
            self.get_logger().warn('GPIO unavailable — publishing constant SAFE_OFF.')

        if self._start_latch_enabled:
            if self._start_latch_ok:
                self._start_latch_thread = threading.Thread(
                    target=self._start_latch_monitor_loop,
                    daemon=True,
                )
                self._start_latch_thread.start()
            else:
                self.get_logger().warn(
                    'Start latch unavailable — latch start trigger disabled.'
                )

    def _init_gpio(self):
        if not _GPIOD_AVAILABLE:
            self.get_logger().warn('gpiod Python library not found. GPIO disabled.')
            return

        try:
            self._chip = gpiod.Chip(self._chip_path)
            self._line = self._chip.get_line(self._line_num)
            self._line.request(
                consumer='safety_node',
                type=gpiod.LINE_REQ_EV_BOTH_EDGES,
            )
            self._gpio_ok = True
            raw = self._line.get_value()
            safe = (raw == 0) if self._active_low else (raw == 1)
            with self._lock:
                self._state = SafetyState.SAFE_ON if safe else SafetyState.SAFE_OFF
            self.get_logger().info(
                f'GPIO {self._chip_path}:{self._line_num} ready — '
                f'initial state {"SAFE_ON" if safe else "SAFE_OFF"}'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'GPIO init failed: {exc}')

    def _init_start_latch(self):
        if not self._start_latch_enabled:
            return

        if not _SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial not installed. Start latch disabled.')
            return

        try:
            self._start_latch_serial = serial.Serial(
                str(self._start_latch_port),
                int(self._start_latch_baudrate),
                timeout=0.2,
                write_timeout=0.2,
            )
            time.sleep(0.3)
            self._start_latch_serial.reset_input_buffer()

            pull_mode = 'machine.Pin.PULL_UP' if self._start_latch_pullup else 'None'
            self._micropython_write_line('import machine')
            self._micropython_write_line(
                f'_latch_pin=machine.Pin({int(self._start_latch_pin_num)}, machine.Pin.IN, {pull_mode})'
            )

            self._last_start_latch_value = self._micropython_read_latch_value()
            self._start_latch_ok = True
            self.get_logger().info(
                f'Start latch ready via MicroPython on {self._start_latch_port} '
                f'GPIO {self._start_latch_pin_num} — '
                f'initial level {self._last_start_latch_value}'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Start latch MicroPython init failed: {exc}')
            self._start_latch_ok = False

    def _gpio_monitor_loop(self):
        while rclpy.ok():
            try:
                has_event = self._line.event_wait(sec=0, nsec=100_000_000)
                if not has_event:
                    continue

                event = self._line.event_read()
                now = time.monotonic()
                if (now - self._last_edge_time) * 1000 < self._debounce_ms:
                    continue
                self._last_edge_time = now

                rising = event.type == gpiod.LineEvent.RISING_EDGE
                safe = (not rising) if self._active_low else rising

                with self._lock:
                    prev = self._state
                    if safe and prev == SafetyState.SAFE_OFF:
                        self._state = SafetyState.RECOVERING
                        self._recovery_deadline = time.monotonic() + self._recovery_timeout
                        self.get_logger().info('Safety: SAFE_OFF -> RECOVERING')
                    elif not safe and prev != SafetyState.SAFE_OFF:
                        self._state = SafetyState.SAFE_OFF
                        self._recovery_deadline = None
                        self.get_logger().warn('Safety: -> SAFE_OFF (safety cut)')

                self._publish_state()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'GPIO monitor error: {exc}')
                time.sleep(0.1)

    def _start_latch_monitor_loop(self):
        while rclpy.ok() and self._start_latch_ok:
            try:
                current_value = self._micropython_read_latch_value()
                if current_value is None:
                    time.sleep(0.02)
                    continue

                previous_value = self._last_start_latch_value
                self._last_start_latch_value = current_value
                now = time.monotonic()

                trigger_value = bool(self._start_latch_trigger_value)
                if (
                    previous_value is not None
                    and previous_value != current_value
                    and current_value == trigger_value
                ):
                    if (
                        (now - self._last_start_latch_edge_time) * 1000
                        >= self._start_latch_debounce_ms
                    ):
                        self._last_start_latch_edge_time = now
                        self._try_start_match_from_latch()

                time.sleep(0.02)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Start latch monitor error: {exc}')
                time.sleep(0.1)

    def _micropython_write_line(self, line: str):
        self._start_latch_serial.write((line + '\r\n').encode('utf-8'))
        time.sleep(0.03)
        if self._start_latch_serial.in_waiting:
            self._start_latch_serial.read(self._start_latch_serial.in_waiting)

    def _micropython_read_latch_value(self):
        self._start_latch_serial.reset_input_buffer()
        self._start_latch_serial.write(b'print(_latch_pin.value())\r\n')

        deadline = time.monotonic() + 0.2
        buffer = ''
        while time.monotonic() < deadline:
            waiting = self._start_latch_serial.in_waiting
            if waiting:
                buffer += self._start_latch_serial.read(waiting).decode('utf-8', errors='ignore')
                for line in reversed(buffer.splitlines()):
                    token = line.strip()
                    if token in ('0', '1'):
                        return token == '1'
            time.sleep(0.01)
        return None

    def _normalize_latch_value(self, value):
        if value is None:
            return None
        return bool(value)

    def _match_active_cb(self, msg: Bool):
        self._match_active = bool(msg.data)

    def _is_match_ready(self) -> bool:
        with self._lock:
            safe_state = self._state
        if self._match_active:
            return False
        if self._require_safe_on and safe_state != SafetyState.SAFE_ON:
            return False
        return True

    def _try_start_match_from_latch(self):
        if not self._is_match_ready():
            self.get_logger().info('Start latch falling edge ignored: match not ready')
            return

        if self._pending_start_future is not None and not self._pending_start_future.done():
            self.get_logger().info('Start latch ignored: start request already in progress')
            return

        if not self._start_match_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(
                f'Start latch trigger: service unavailable {self._start_match_service}'
            )
            return

        self.get_logger().info('Start latch falling edge detected: requesting match start')
        self._pending_start_future = self._start_match_client.call_async(Trigger.Request())
        self._pending_start_future.add_done_callback(self._on_start_match_response)

    def _on_start_match_response(self, future):
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Start latch trigger failed: {exc}')
            return

        if result is None:
            self.get_logger().warn('Start latch trigger: no response from start_match service')
            return

        if result.success:
            self.get_logger().info(f'Start latch trigger accepted: {result.message}')
        else:
            self.get_logger().warn(f'Start latch trigger rejected: {result.message}')

    def _publish_state(self):
        with self._lock:
            if (
                self._state == SafetyState.RECOVERING
                and self._recovery_deadline is not None
                and time.monotonic() >= self._recovery_deadline
            ):
                self._state = SafetyState.SAFE_ON
                self._recovery_deadline = None
                self.get_logger().info('Safety: RECOVERING -> SAFE_ON')
            state_val = self._state

        msg = SafetyState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state_val
        self._pub.publish(msg)

    def destroy_node(self):
        try:
            if self._start_latch_serial is not None:
                self._start_latch_serial.close()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Error while closing latch serial: {exc}')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
