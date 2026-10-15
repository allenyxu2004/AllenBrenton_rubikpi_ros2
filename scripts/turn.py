#!/usr/bin/env python3
import argparse
import json
import math
import time
import serial
def send_command(connection, left, right):
    command = {"T": 1, "L": left, "R": right}
    connection.write((json.dumps(command) + "\n").encode("utf-8"))
    connection.flush()
def main():
    parser = argparse.ArgumentParser(description="Test timed pivot turns.")
    parser.add_argument("direction", choices=["left", "right"])
    parser.add_argument(
        "duration", type=float, nargs="?", default=0.5,
        help="Turn duration in seconds (default: 0.5)",
    )
    parser.add_argument("--left-speed", type=float, default=0.2)
    parser.add_argument("--right-speed", type=float, default=0.2)
    parser.add_argument("-t", "--times", type=int, default=1)
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be finite and greater than zero")
    if args.times < 1:
        parser.error("times must be at least 1")
    for speed in (args.left_speed, args.right_speed):
        if not math.isfinite(speed) or not 0 < speed <= 0.5:
            parser.error("motor magnitudes must be greater than 0 and at most 0.5")
    if args.direction == "left":
        left, right = -args.left_speed, args.right_speed
    else:
        left, right = args.left_speed, -args.right_speed
    try:
        with serial.Serial(args.port, 115200, timeout=1) as connection:
            try:
                send_command(connection, 0, 0)
                for trial in range(1, args.times + 1):
                    input(
                        f"Trial {trial}/{args.times}: mark the starting heading, "
                        "then press Enter to turn..."
                    )
                    print(
                        f"Turning {args.direction}: L={left}, R={right}, "
                        f"duration={args.duration:.2f} s",
                        flush=True,
                    )
                    start = time.monotonic()
                    deadline = start + args.duration
                    while time.monotonic() < deadline:
                        send_command(connection, left, right)
                        remaining = deadline - time.monotonic()
                        if remaining > 0:
                            time.sleep(min(0.05, remaining))
                    send_command(connection, 0, 0)
                    elapsed = time.monotonic() - start
                    print(f"Stopped. Actual elapsed time: {elapsed:.3f} s")
                    measured = input(
                        "Enter measured angle magnitude in degrees "
                        "(or Enter to skip): "
                    ).strip()
                    if measured:
                        try:
                            angle = float(measured)
                            if not math.isfinite(angle) or angle < 0:
                                raise ValueError
                            print(f"Average turn rate: {angle / elapsed:.2f} deg/s")
                        except ValueError:
                            print("Invalid angle; measurement skipped.")
            finally:
                send_command(connection, 0, 0)
    except (KeyboardInterrupt, EOFError):
        print("\nTest ended.")
    except (serial.SerialException, OSError) as error:
        print(f"Serial error: {error}")
if __name__ == "__main__":
    main()
