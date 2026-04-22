"""
pca9685_driver.py — Minimal PCA9685 PWM driver using smbus2 only.

Datasheet reference: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

Register map (relevant):
    0x00  MODE1  — sleep/auto-increment/all-call bits
    0x01  MODE2  — output drive configuration
    0xFA  ALL_LED_ON_L  — broadcast on low byte
    0xFE  PRE_SCALE     — divider for PWM frequency

Per-channel registers (base = 0x06 + channel * 4):
    +0  LED_ON_L
    +1  LED_ON_H
    +2  LED_OFF_L
    +3  LED_OFF_H
"""

import time
import smbus2

_MODE1 = 0x00
_MODE2 = 0x01
_PRE_SCALE = 0xFE
_LED0_ON_L = 0x06
_ALL_LED_OFF_H = 0xFD
_OSC_CLOCK = 25_000_000   # internal oscillator (Hz)
_RESOLUTION = 4096         # 12-bit


class PCA9685:
    """Thin PCA9685 driver. No adafruit dependency."""

    def __init__(self, bus_number: int = 1, address: int = 0x40):
        self._bus = smbus2.SMBus(bus_number)
        self._addr = address
        self._reset()

    # ------------------------------------------------------------------ Init --

    def _reset(self):
        # Sleep mode, auto-increment enabled.
        self._write(_MODE1, 0x10)
        time.sleep(0.005)

    def set_pwm_freq(self, freq_hz: float):
        """Set PWM frequency (24–1526 Hz). Typical servo: 50 Hz."""
        prescale = round(_OSC_CLOCK / (_RESOLUTION * freq_hz)) - 1
        prescale = max(3, min(255, prescale))

        old_mode = self._read(_MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10   # sleep bit high
        self._write(_MODE1, sleep_mode)
        self._write(_PRE_SCALE, prescale)
        self._write(_MODE1, old_mode)
        time.sleep(0.005)
        # Restart PWM with auto-increment.
        self._write(_MODE1, old_mode | 0xA1)

    def set_pwm(self, channel: int, on: int, off: int):
        """Set raw ON/OFF 12-bit tick counts for a channel (0–15)."""
        base = _LED0_ON_L + 4 * channel
        self._bus.write_i2c_block_data(
            self._addr, base,
            [on & 0xFF, on >> 8, off & 0xFF, off >> 8],
        )

    def set_servo_angle(self, channel: int, angle_deg: float,
                        min_pulse_us: float = 500.0,
                        max_pulse_us: float = 2500.0,
                        freq_hz: float = 50.0):
        """Convert angle (0–180°) to PWM pulse and write."""
        angle_deg = max(0.0, min(180.0, angle_deg))
        period_us = 1_000_000.0 / freq_hz
        pulse_us = min_pulse_us + (max_pulse_us - min_pulse_us) * angle_deg / 180.0
        off_tick = round(pulse_us / period_us * _RESOLUTION)
        self.set_pwm(channel, 0, off_tick)

    def set_channel_full_off(self, channel: int):
        """Force a channel completely off (bit 4 of OFF_H)."""
        base = _LED0_ON_L + 4 * channel
        self._bus.write_i2c_block_data(self._addr, base, [0, 0, 0, 0x10])

    def all_channels_off(self):
        """Broadcast full-off to all 16 channels simultaneously."""
        self._write(_ALL_LED_OFF_H, 0x10)

    # ----------------------------------------------------------------- Util --

    def _write(self, reg: int, value: int):
        self._bus.write_byte_data(self._addr, reg, value)

    def _read(self, reg: int) -> int:
        return self._bus.read_byte_data(self._addr, reg)

    def close(self):
        self._bus.close()
