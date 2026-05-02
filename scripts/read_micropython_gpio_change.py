#!/usr/bin/env python3
"""Read a MicroPython GPIO pin over serial and print state changes.

Example:
  python3 scripts/read_micropython_gpio_change.py --port /dev/ttyACM0 --pin 28 --pull-up
"""

import argparse
import sys
import time

try:
    import serial
except ImportError as exc:
    print("pyserial is required: sudo apt install python3-serial", file=sys.stderr)
    raise SystemExit(1) from exc


def write_line(ser: serial.Serial, line: str) -> None:
    ser.write((line + "\r\n").encode("utf-8"))


def read_pin_value(ser: serial.Serial, timeout_s: float = 0.25):
    ser.reset_input_buffer()
    write_line(ser, "print(_host_pin.value())")

    deadline = time.monotonic() + timeout_s
    buf = ""
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting:
            buf += ser.read(waiting).decode("utf-8", errors="ignore")
            for line in reversed(buf.splitlines()):
                token = line.strip()
                if token in ("0", "1"):
                    return int(token)
        time.sleep(0.01)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor MicroPython GPIO pin changes")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--pin", type=int, default=28, help="RP2040 GPIO pin number")
    parser.add_argument("--interval", type=float, default=0.05, help="Poll interval in seconds")
    parser.add_argument("--pull-up", action="store_true", help="Configure input with pull-up")
    parser.add_argument("--print-all", action="store_true", help="Print every sample, not only changes")
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baudrate}...")
    ser = serial.Serial(args.port, args.baudrate, timeout=0.2, write_timeout=0.2)

    try:
        time.sleep(0.3)
        ser.reset_input_buffer()

        write_line(ser, "import machine")
        pull_mode = "machine.Pin.PULL_UP" if args.pull_up else "None"
        write_line(
            ser,
            f"_host_pin = machine.Pin({args.pin}, machine.Pin.IN, {pull_mode})",
        )
        time.sleep(0.05)

        print("Monitoring pin values. Press Ctrl+C to stop.")
        last = None

        while True:
            value = read_pin_value(ser)
            now = time.strftime("%H:%M:%S")
            if value is None:
                print(f"[{now}] read timeout")
            elif args.print_all or value != last:
                state = "HIGH" if value == 1 else "LOW"
                print(f"[{now}] GPIO{args.pin} = {value} ({state})")
                last = value

            time.sleep(max(0.0, args.interval))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
