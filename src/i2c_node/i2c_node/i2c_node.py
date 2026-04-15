"""
i2c_node.py — I2C actuator node (RPi4).

Responsibilities
----------------
* Drive a PCA9685 PWM board for servos (smbus2, no adafruit dependency).
* Drive an MCP23017 GPIO expander for on/off pump control.
* Expose a ControlPump action server per pump_id (timed, auto-off).
* Subscribe to /pwm_servo_cmd  → move a PWM servo to a target angle.
* Subscribe to /safety_state   → immediately cut all outputs on SAFE_OFF.
* Publish  /pump_state         at 5 Hz for each configured pump.

Pump wiring (MCP23017)
-----------------------
Each pump is wired to one pin on the MCP23017 GPIO expander (I2C address
typically 0x20–0x27).  The pin is driven HIGH to activate the pump relay.
    port 0 → MCP23017 port A, pins 0-7
    port 1 → MCP23017 port B, pins 0-7

PWM servo wiring (PCA9685)
---------------------------
Each servo is assigned a PCA9685 channel driven at 50 Hz.

Parameters (declared — override via config YAML)
-------------------------------------------------
    i2c_bus            int    1      Linux I2C bus (/dev/i2c-<n>)
    pca9685_address    int    0x40   PCA9685 I2C address
    mcp23017_address   int    0x20   MCP23017 I2C address
    servo_freq_hz      float  50.0   PWM frequency for servos
    pump_state_hz      float  5.0    Publish rate for /pump_state
    safety_timeout_sec float  0.5    Max gap between safety messages

Pump params (up to 8 pumps, flat numbered):
    pump_N_id    int   logical pump ID (-1 = slot unused)
    pump_N_port  int   MCP23017 port (0=A, 1=B)
    pump_N_pin   int   pin on that port (0-7)
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from robot_actuators.action import ControlPump
from robot_hw_interfaces.msg import PumpCommand, PumpState, PwmServoCommand, SafetyState

from .mcp23017_driver import MCP23017
from .pca9685_driver import PCA9685

_SAFETY_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class PumpEntry:
    """Runtime state for one pump (MCP23017 GPIO pin)."""

    def __init__(self, pump_id: int, port: int, pin: int):
        self.pump_id = pump_id
        self.port = port   # 0 = port A, 1 = port B
        self.pin = pin     # 0-7
        self.enabled = False
        self.disabled_by_safety = False
        self.lock = threading.Lock()


class I2CNode(Node):
    def __init__(self):
        super().__init__('i2c_node')

        # ---- parameters ----
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('pca9685_address', 0x40)
        self.declare_parameter('mcp23017_address', 0x20)
        self.declare_parameter('servo_freq_hz', 50.0)
        self.declare_parameter('pump_state_hz', 5.0)
        self.declare_parameter('safety_timeout_sec', 0.5)

        bus = self.get_parameter('i2c_bus').value
        pca_addr = self.get_parameter('pca9685_address').value
        mcp_addr = self.get_parameter('mcp23017_address').value
        self._servo_freq = self.get_parameter('servo_freq_hz').value
        pump_rate = self.get_parameter('pump_state_hz').value
        self._safety_timeout = self.get_parameter('safety_timeout_sec').value

        # ---- PCA9685 (PWM servos) ----
        try:
            self._pca = PCA9685(bus_number=bus, address=pca_addr)
            self._pca.set_pwm_freq(self._servo_freq)
            self._pca.all_channels_off()
            self._pca_ok = True
            self.get_logger().info(f'PCA9685 ready on I2C bus {bus}, addr 0x{pca_addr:02X}')
        except Exception as exc:  # noqa: BLE001
            self._pca = None
            self._pca_ok = False
            self.get_logger().error(f'PCA9685 init failed: {exc}')

        # ---- MCP23017 (pump GPIO) ----
        try:
            self._mcp = MCP23017(bus_number=bus, address=mcp_addr)
            self._mcp_ok = True
            self.get_logger().info(f'MCP23017 ready on I2C bus {bus}, addr 0x{mcp_addr:02X}')
        except Exception as exc:  # noqa: BLE001
            self._mcp = None
            self._mcp_ok = False
            self.get_logger().error(f'MCP23017 init failed: {exc}')

        # ---- PWM servo registry (channel → pulse limits) ----
        # Flat params: pwm_servo_N_channel, pwm_servo_N_min_pulse_us, pwm_servo_N_max_pulse_us
        # Maps channel number → (min_pulse_us, max_pulse_us)
        self._pwm_servo_pulses: dict[int, tuple] = {}
        for n in range(16):
            self.declare_parameter(f'pwm_servo_{n}_channel', -1)
            self.declare_parameter(f'pwm_servo_{n}_min_pulse_us', 500.0)
            self.declare_parameter(f'pwm_servo_{n}_max_pulse_us', 2500.0)
            ch = self.get_parameter(f'pwm_servo_{n}_channel').value
            if ch >= 0:
                self._pwm_servo_pulses[ch] = (
                    float(self.get_parameter(f'pwm_servo_{n}_min_pulse_us').value),
                    float(self.get_parameter(f'pwm_servo_{n}_max_pulse_us').value),
                )

        # ---- pump registry ----
        # Flat params: pump_N_id, pump_N_port (0=A/1=B), pump_N_pin (0-7)
        self._pumps: dict[int, PumpEntry] = {}
        for n in range(8):
            self.declare_parameter(f'pump_{n}_id', -1)
            self.declare_parameter(f'pump_{n}_port', -1)
            self.declare_parameter(f'pump_{n}_pin', -1)
            pid = self.get_parameter(f'pump_{n}_id').value
            port = self.get_parameter(f'pump_{n}_port').value
            pin = self.get_parameter(f'pump_{n}_pin').value
            if pid >= 0 and port >= 0 and pin >= 0:
                self._pumps[pid] = PumpEntry(pump_id=pid, port=port, pin=pin)

        # ---- safety ----
        self._system_safe = False
        self._last_safety_stamp = time.monotonic()
        self._safety_lock = threading.Lock()

        self._safety_sub = self.create_subscription(
            SafetyState, '/safety_state', self._safety_cb, _SAFETY_QOS
        )

        # ---- pump direct command subscriber ----
        self._pump_cmd_sub = self.create_subscription(
            PumpCommand, '/pump_cmd', self._pump_cmd_cb, 10
        )

        # ---- pwm servo command subscriber ----
        self._servo_cmd_sub = self.create_subscription(
            PwmServoCommand, '/pwm_servo_cmd', self._servo_cmd_cb, 10
        )

        # ---- pump state publisher ----
        self._pump_state_pub = self.create_publisher(PumpState, '/pump_state', 10)
        self._pump_state_timer = self.create_timer(
            1.0 / pump_rate, self._publish_pump_states
        )

        # ---- ControlPump action server ----
        self._pump_action_server = ActionServer(
            self,
            ControlPump,
            'control_pump',
            goal_callback=self._pump_goal_cb,
            cancel_callback=self._pump_cancel_cb,
            execute_callback=self._pump_execute_cb,
        )

        # ---- safety watchdog ----
        self._watchdog_timer = self.create_timer(0.1, self._safety_watchdog)

        self.get_logger().info('i2c_node initialised.')

    # --------------------------------------------------------------- Safety --

    def _safety_cb(self, msg: SafetyState):
        safe = (msg.state == SafetyState.SAFE_ON)
        with self._safety_lock:
            prev = self._system_safe
            self._system_safe = safe
            self._last_safety_stamp = time.monotonic()

        if prev and not safe:
            self.get_logger().warn('Safety SAFE_OFF — cutting all outputs.')
            self._cut_all_outputs()
        elif not prev and safe:
            self.get_logger().info('Safety SAFE_ON — outputs available.')

    def _safety_watchdog(self):
        """If no safety message received within timeout → assume SAFE_OFF."""
        with self._safety_lock:
            age = time.monotonic() - self._last_safety_stamp
            was_safe = self._system_safe
            if age > self._safety_timeout and was_safe:
                self._system_safe = False
                self.get_logger().warn(
                    f'Safety watchdog timeout ({age:.2f}s) — cutting outputs.'
                )
                self._cut_all_outputs()

    def _cut_all_outputs(self):
        for entry in self._pumps.values():
            with entry.lock:
                entry.enabled = False
                entry.disabled_by_safety = True
        if self._mcp and self._mcp_ok:
            self._mcp.all_off()
        if self._pca and self._pca_ok:
            self._pca.all_channels_off()

    # -------------------------------------------------------------- Pumps --

    def _hw_set_pump(self, entry: 'PumpEntry', on: bool):
        if not self._mcp_ok or self._mcp is None:
            return
        self._mcp.set_pin(entry.port, entry.pin, on)

    def _pump_cmd_cb(self, msg: PumpCommand):
        entry = self._pumps.get(msg.pump_id)
        if entry is None:
            self.get_logger().warn(f'Unknown pump_id {msg.pump_id}')
            return
        with self._safety_lock:
            safe = self._system_safe
        with entry.lock:
            if not safe and msg.enable:
                self.get_logger().warn(
                    f'Pump {msg.pump_id} ON rejected — system not safe.'
                )
                return
            entry.disabled_by_safety = False
            entry.enabled = msg.enable
            self._hw_set_pump(entry, msg.enable)

    def _publish_pump_states(self):
        for entry in self._pumps.values():
            with entry.lock:
                if entry.disabled_by_safety:
                    state = PumpState.DISABLED_BY_SAFETY
                elif entry.enabled:
                    state = PumpState.ON
                else:
                    state = PumpState.OFF
                pid = entry.pump_id

            msg = PumpState()
            msg.pump_id = pid
            msg.state = state
            self._pump_state_pub.publish(msg)

    # ------------------------------------------------------------- Servo --

    def _servo_cmd_cb(self, msg: PwmServoCommand):
        if not self._pca_ok or self._pca is None:
            return
        with self._safety_lock:
            safe = self._system_safe
        if not safe:
            self.get_logger().warn(
                f'Servo ch{msg.channel} command rejected — system not safe.'
            )
            return
        if not msg.enable:
            self._pca.set_channel_full_off(msg.channel)
        else:
            min_us, max_us = self._pwm_servo_pulses.get(msg.channel, (500.0, 2500.0))
            self._pca.set_servo_angle(
                msg.channel, msg.target_deg,
                min_pulse_us=min_us, max_pulse_us=max_us,
                freq_hz=self._servo_freq,
            )

    # --------------------------------------------------------- ControlPump action --

    def _pump_goal_cb(self, goal_request):
        if goal_request.pump_id not in self._pumps:
            self.get_logger().warn(
                f'ControlPump: unknown pump_id {goal_request.pump_id}'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _pump_cancel_cb(self, goal_handle: ServerGoalHandle):
        return CancelResponse.ACCEPT

    def _pump_execute_cb(self, goal_handle: ServerGoalHandle):
        goal = goal_handle.request
        entry = self._pumps[goal.pump_id]
        result = ControlPump.Result()

        with self._safety_lock:
            safe = self._system_safe
        if not safe and goal.enable:
            result.success = False
            result.message = 'System not safe — pump command rejected.'
            goal_handle.abort()
            return result

        # Set initial pump state.
        with entry.lock:
            entry.enabled = goal.enable
            entry.disabled_by_safety = False
            self._hw_set_pump(entry, goal.enable)

        if goal.enable and goal.duration_sec > 0.0:
            start = time.monotonic()
            deadline = start + goal.duration_sec
            feedback_msg = ControlPump.Feedback()
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    with entry.lock:
                        entry.enabled = False
                        self._hw_set_pump(entry, False)
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Cancelled.'
                    return result

                # Abort if safety lost.
                with self._safety_lock:
                    safe = self._system_safe
                if not safe:
                    with entry.lock:
                        entry.enabled = False
                        entry.disabled_by_safety = True
                        self._hw_set_pump(entry, False)
                    goal_handle.abort()
                    result.success = False
                    result.message = 'Safety cut during pump run.'
                    return result

                feedback_msg.elapsed_sec = time.monotonic() - start
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.05)

            # Auto-off after duration.
            with entry.lock:
                entry.enabled = False
                self._hw_set_pump(entry, False)

        result.success = True
        result.message = 'OK'
        goal_handle.succeed()
        return result

    # --------------------------------------------------------------- Cleanup --

    def destroy_node(self):
        if self._mcp and self._mcp_ok:
            self._mcp.all_off()
            self._mcp.close()
        if self._pca and self._pca_ok:
            self._pca.all_channels_off()
            self._pca.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = I2CNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
