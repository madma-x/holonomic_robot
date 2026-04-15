"""
mcp23017_driver.py — Minimal MCP23017 16-bit GPIO expander driver via smbus2.

Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf

Register map (IOCON.BANK = 0, power-on default):
    0x00  IODIRA  — direction port A (1=input, 0=output)
    0x01  IODIRB  — direction port B
    0x12  GPIOA   — read port A GPIO
    0x13  GPIOB   — read port B GPIO
    0x14  OLATA   — output latch port A
    0x15  OLATB   — output latch port B

Usage: each pump maps to one (port, pin) pair.
    port 0 → port A,  pin 0-7
    port 1 → port B,  pin 0-7
"""

import smbus2

_IODIRA = 0x00
_IODIRB = 0x01
_OLATA  = 0x14
_OLATB  = 0x15

_PORT_REGS = {
    0: (_IODIRA, _OLATA),
    1: (_IODIRB, _OLATB),
}


class MCP23017:
    """Thin MCP23017 driver. All pins initialised as outputs, latches cleared."""

    def __init__(self, bus_number: int = 1, address: int = 0x20):
        self._bus = smbus2.SMBus(bus_number)
        self._addr = address
        # Shadow latch registers so we can do bit-level updates without a read.
        self._latch = [0x00, 0x00]   # [port_A, port_B]
        # Set all 16 pins as outputs.
        self._write(_IODIRA, 0x00)
        self._write(_IODIRB, 0x00)
        # Clear all outputs.
        self._write(_OLATA, 0x00)
        self._write(_OLATB, 0x00)

    def set_pin(self, port: int, pin: int, value: bool):
        """Set a single output pin (port 0=A, 1=B; pin 0-7)."""
        if port not in _PORT_REGS:
            raise ValueError(f'MCP23017: invalid port {port}')
        if not 0 <= pin <= 7:
            raise ValueError(f'MCP23017: invalid pin {pin}')
        if value:
            self._latch[port] |= (1 << pin)
        else:
            self._latch[port] &= ~(1 << pin) & 0xFF
        _, latch_reg = _PORT_REGS[port]
        self._write(latch_reg, self._latch[port])

    def all_off(self):
        """Drive all output pins low."""
        self._latch = [0x00, 0x00]
        self._write(_OLATA, 0x00)
        self._write(_OLATB, 0x00)

    def close(self):
        self._bus.close()

    def _write(self, reg: int, value: int):
        self._bus.write_byte_data(self._addr, reg, value)
