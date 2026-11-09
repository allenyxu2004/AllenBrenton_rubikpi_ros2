#!/usr/bin/env python3
"""Check a nominal 90-degree turn using motor_commands and the ROS bridge.

Stop SLAM/teleop first. Run motor_control separately. No camera or filter is used.
The printed predicted angle is a command-based estimate, not a sensor reading.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray


class TurnCheck(Node):
    """Repeat SLAM's 20 Hz pulse and stationary-pause pattern."""

    def __init__(self, args):
        super().__init__('ros_turn_check')
        self.args = args
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 1)
        self.predicted_angle = 0.0
        self.active = False
        self.done = False
        self.failed = False
        self.previous = time.monotonic()
        self.next_pulse = self.previous + 2.0
        self.pulse_end = 0.0
        self.pulses = 0
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'{args.direction} turn check: target={args.angle:g} deg, '
            f'assumed rate={args.rate:.6f} rad/s. Starts after 2 s and a motor subscriber.')

    def send(self, left=0.0, right=0.0):
        self.publisher.publish(Float32MultiArray(data=[float(left), float(right)]))

    def finish(self, reason, failed=False):
        self.active = False
        self.done = True
        self.failed = failed
        self.send()
        self.get_logger().info(
            f'{reason}; {self.pulses} pulses; '
            f'command-predicted angle={math.degrees(self.predicted_angle):.2f} deg. '
            'Measure the physical angle against the tape reference.')

    def tick(self):
        now = time.monotonic()
        dt = now - self.previous
        self.previous = now
        if self.done:
            self.send()
            return
        if self.active:
            self.predicted_angle += self.args.rate * min(dt, 0.15)
            if dt > 0.15:
                self.finish('Timing gap: discard this trial', failed=True)
                return
        if self.count_publishers('motor_commands') > 1:
            self.finish('Another command publisher is active: stop its launch', failed=True)
            return
        if self.publisher.get_subscription_count() == 0:
            if self.active:
                self.finish('Motor subscriber disappeared: discard this trial', failed=True)
            else:
                self.send()
            return
        if self.active and now >= self.pulse_end:
            self.active = False
            self.next_pulse = now + self.args.pause
        if not self.active and now >= self.next_pulse:
            remaining = math.radians(self.args.angle) - self.predicted_angle
            if remaining <= 0:
                self.finish('Turn complete')
                return
            self.active = True
            self.pulses += 1
            # Match SLAM: at least one 50 ms tick, capped by pulse_s.
            duration = max(0.05, min(self.args.pulse, remaining / self.args.rate))
            self.pulse_end = now + duration
        if self.active:
            sign = 1 if self.args.direction == 'right' else -1
            self.send(sign * self.args.left_speed, -sign * self.args.right_speed)
        else:
            self.send()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('direction', choices=['left', 'right'])
    parser.add_argument('--rate', type=float, required=True, help='Assumed turn rate, rad/s')
    parser.add_argument('--angle', type=float, default=90.0, help='Nominal total angle, degrees')
    parser.add_argument('--pulse', type=float, default=0.15, help='Maximum pulse duration, s')
    parser.add_argument('--pause', type=float, default=0.6, help='Stopped interval between pulses, s')
    parser.add_argument('--left-speed', type=float, default=0.45)
    parser.add_argument('--right-speed', type=float, default=0.30)
    args = parser.parse_args()
    for name, value in vars(args).items():
        if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
            parser.error(f'{name} must be finite and positive')
    if args.pulse < 0.05:
        parser.error('pulse must be at least one 0.05 s command tick')
    if max(args.left_speed, args.right_speed) > 0.5:
        parser.error('motor command magnitudes must not exceed 0.5')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TurnCheck(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.finish('Interrupted: discard this trial', failed=True)
    finally:
        # Refresh zeros long enough for the serial bridge to transmit a stop.
        node.done = True
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
