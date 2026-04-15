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
    active_low    (bool, default false)             — invert signal polarity
    debounce_ms   (int,  default 50)                — edge debounce window
    publish_rate_hz (float, default 20.0)           — periodic publish rate
    recovery_timeout_sec (float, default 2.0)       — how long RECOVERING lasts
"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from robot_hw_interfaces.msg import SafetyState

try:
    import gpiod
    _GPIOD_AVAILABLE = True
except ImportError:
    _GPIOD_AVAILABLE = False


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('gpio_chip', '/dev/gpiochip0')
        self.declare_parameter('gpio_line', 18)
        self.declare_parameter('active_low', False)
        self.declare_parameter('debounce_ms', 50)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('recovery_timeout_sec', 2.0)

        self._chip_path = self.get_parameter('gpio_chip').value
        self._line_num = self.get_parameter('gpio_line').value
        self._active_low = self.get_parameter('active_low').value
        self._debounce_ms = self.get_parameter('debounce_ms').value
        self._rate_hz = self.get_parameter('publish_rate_hz').value
        self._recovery_timeout = self.get_parameter('recovery_timeout_sec').value

        # Latched transient QoS so late subscribers see the last state immediately.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(SafetyState, '/safety_state', qos)

        self._lock = threading.Lock()
        self._state = SafetyState.SAFE_OFF
        self._recovery_deadline: float | None = None
        self._last_edge_time: float = 0.0

        self._line = None
        self._gpio_ok = False
        self._init_gpio()

        period = 1.0 / self._rate_hz
        self._timer = self.create_timer(period, self._publish_state)

        # Separate thread monitors GPIO edges (blocking read).
        if self._gpio_ok:
            self._monitor_thread = threading.Thread(
                target=self._gpio_monitor_loop, daemon=True
            )
            self._monitor_thread.start()
        else:
            self.get_logger().warn(
                'GPIO unavailable — publishing constant SAFE_OFF.'
            )

    # ------------------------------------------------------------------ GPIO --

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
            # Bootstrap: read current level before any edge fires.
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

    def _gpio_monitor_loop(self):
        """Blocking loop — calls gpiod.Line.event_wait() then event_read()."""
        while rclpy.ok():
            try:
                has_event = self._line.event_wait(sec=0, nsec=100_000_000)  # 100 ms
                if not has_event:
                    continue
                event = self._line.event_read()
                now = time.monotonic()
                if (now - self._last_edge_time) * 1000 < self._debounce_ms:
                    continue
                self._last_edge_time = now

                # Determine physical level from event type.
                rising = (event.type == gpiod.LineEvent.RISING_EDGE)
                safe = (not rising) if self._active_low else rising

                with self._lock:
                    prev = self._state
                    if safe and prev == SafetyState.SAFE_OFF:
                        # Transition OFF→ON: brief RECOVERING period.
                        self._state = SafetyState.RECOVERING
                        self._recovery_deadline = (
                            time.monotonic() + self._recovery_timeout
                        )
                        self.get_logger().info('Safety: SAFE_OFF → RECOVERING')
                    elif not safe and prev != SafetyState.SAFE_OFF:
                        self._state = SafetyState.SAFE_OFF
                        self._recovery_deadline = None
                        self.get_logger().warn('Safety: → SAFE_OFF (safety cut)')

                # Publish immediately on edge.
                self._publish_state()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'GPIO monitor error: {exc}')
                time.sleep(0.1)

    # ----------------------------------------------------------- Timer/publish --

    def _publish_state(self):
        with self._lock:
            # Advance RECOVERING → SAFE_ON once deadline passes.
            if (
                self._state == SafetyState.RECOVERING
                and self._recovery_deadline is not None
                and time.monotonic() >= self._recovery_deadline
            ):
                self._state = SafetyState.SAFE_ON
                self._recovery_deadline = None
                self.get_logger().info('Safety: RECOVERING → SAFE_ON')
            state_val = self._state

        msg = SafetyState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state_val
        self._pub.publish(msg)


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
