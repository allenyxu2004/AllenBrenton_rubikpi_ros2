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
    parser = argparse.ArgumentParser(
        description="Send timed motion commands to the robot."
    )
    parser.add_argument("L", type=float, nargs="?", default=0.2)
    parser.add_argument("R", type=float, nargs="?", default=0.2)
    parser.add_argument("duration", type=float, nargs="?", default=2.0)
    parser.add_argument("-t", "--times", type=int, default=1)
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be finite and greater than zero")
    if args.times < 1:
        parser.error("times must be at least 1")
    for speed in (args.L, args.R):
        if not math.isfinite(speed) or abs(speed) > 0.5:
            parser.error("L and R must be finite and within [-0.5, 0.5]")
    try:
        with serial.Serial(args.port, 115200, timeout=1) as connection:
            try:
                for trial in range(1, args.times + 1):
                    print(
                        f"Run {trial}/{args.times}: "
                        f"L={args.L}, R={args.R}, "
                        f"duration={args.duration:.2f} s",
                        flush=True,
                    )
                    send_command(connection, args.L, args.R)
                    time.sleep(args.duration)
                    send_command(connection, 0, 0)
                    response = connection.read(300)
                    print(response.decode("utf-8", errors="replace"))
                    if trial < args.times:
                        time.sleep(0.5)
            finally:
                send_command(connection, 0, 0)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except (serial.SerialException, OSError) as error:
        print(f"Serial error: {error}")
if __name__ == "__main__":
    main()
