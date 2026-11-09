#!/usr/bin/env python3
"""Check forward motion in measured increments through the ROS motor bridge.

Stop SLAM and teleop; run motor_control separately. Commands use the same 20 Hz
pulse/pause pattern as SLAM. Distances printed here are predictions, not sensors.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray


class ForwardCheck(Node):
    """Move one increment, stop and settle, then let the operator measure."""

    def __init__(self, args):
        super().__init__('ros_forward_check')
        self.args = args
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 1)
        self.timer = self.create_timer(0.05, self.tick)
        self.active = False
        self.running = False
        self.failed = False
        self.distance = 0.0
        self.total_distance = 0.0
        self.powered_seconds = 0.0
        self.pulses = 0
        self.previous = time.monotonic()
        self.next_pulse = self.previous
        self.pulse_end = 0.0

    def send(self, left=0.0, right=0.0):
        self.publisher.publish(Float32MultiArray(data=[float(left), float(right)]))

    def begin_increment(self):
        self.distance = 0.0
        self.powered_seconds = 0.0
        self.pulses = 0
        self.previous = time.monotonic()
        self.next_pulse = self.previous
        self.running = True

    def stop(self, reason, failed=False):
        self.active = False
        self.running = False
        self.failed = failed
        self.send()
        self.get_logger().info(reason)

    def tick(self):
        now = time.monotonic()
        dt = now - self.previous
        self.previous = now
        if not self.running:
            self.send()
            return
        if self.active:
            elapsed = min(dt, 0.15)
            self.powered_seconds += elapsed
            displacement = self.args.speed * elapsed
            self.distance += displacement
            self.total_distance += displacement
            if dt > 0.15:
                self.stop('Timing gap: discard this trial', failed=True)
                return
        if self.count_publishers('motor_commands') > 1:
            self.stop('Another command publisher is active: stop its launch', failed=True)
            return
        if self.publisher.get_subscription_count() == 0:
            if self.active:
                self.stop('Motor subscriber disappeared: discard this trial', failed=True)
            else:
                self.send()
            return
        if self.active and now >= self.pulse_end:
            self.active = False
            self.next_pulse = now + self.args.pause
        if not self.active and now >= self.next_pulse:
            remaining = self.args.distance_ft * 0.3048 - self.distance
            if remaining <= 0:
                self.stop(
                    f'Increment complete: {self.pulses} pulses, '
                    f'{self.powered_seconds:.3f} s powered; '
                    f'predicted increment={self.distance / 0.3048:.3f} ft, '
                    f'predicted total={self.total_distance / 0.3048:.3f} ft. '
                    'Now measure the actual robot center position.')
                return
            self.active = True
            self.pulses += 1
            duration = max(0.05, min(self.args.pulse, remaining / self.args.speed))
            self.pulse_end = now + duration
        if self.active:
            self.send(self.args.left_command, self.args.right_command)
        else:
            self.send()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance-ft', type=float, default=1.0, help='Nominal increment, feet')
    parser.add_argument('--increments', type=int, default=1, help='Number of manual measurement stops')
    parser.add_argument('--speed', type=float, default=0.29845, help='Assumed forward speed, m/s')
    parser.add_argument('--left-command', type=float, default=0.25)
    parser.add_argument('--right-command', type=float, default=0.20)
    parser.add_argument('--pulse', type=float, default=0.15, help='Maximum movement pulse, seconds')
    parser.add_argument('--pause', type=float, default=0.6, help='Stopped time between pulses, seconds')
    args = parser.parse_args()
    for name, value in vars(args).items():
        if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
            parser.error(f'{name} must be finite and positive')
    if args.increments < 1:
        parser.error('increments must be at least 1')
    if args.pulse < 0.05:
        parser.error('pulse must be at least one 0.05 s command tick')
    if max(args.left_command, args.right_command) > 0.5:
        parser.error('motor commands must not exceed 0.5')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ForwardCheck(args)
    try:
        print('Mark the initial robot center and forward direction. Do not reset between increments.')
        print(f'Commands L={args.left_command}, R={args.right_command}; assumed speed={args.speed} m/s.')
        for increment in range(1, args.increments + 1):
            input(f'Increment {increment}/{args.increments}: clear the route, then press Enter to move...')
            node.begin_increment()
            while rclpy.ok() and node.running:
                rclpy.spin_once(node, timeout_sec=0.05)
            if node.failed or not rclpy.ok():
                break
            measurement = input(
                'Measured TOTAL forward distance from the original mark, inches '
                '(or Enter to skip): ').strip()
            if measurement:
                try:
                    inches = float(measurement)
                    if not math.isfinite(inches) or inches < 0:
                        raise ValueError
                    print(f'MEASUREMENT increment={increment}, actual_total_inches={inches:.3f}, '
                          f'predicted_total_inches={node.total_distance / 0.0254:.3f}', flush=True)
                except ValueError:
                    print('Invalid measurement; skipped. Robot remains stopped until the next Enter.')
    except (KeyboardInterrupt, EOFError):
        node.stop('Interrupted: discard any incomplete increment', failed=True)
    finally:
        node.running = False
        node.active = False
        stop_until = time.monotonic() + 0.3
        while rclpy.ok() and time.monotonic() < stop_until:
            node.send()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if node.failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
