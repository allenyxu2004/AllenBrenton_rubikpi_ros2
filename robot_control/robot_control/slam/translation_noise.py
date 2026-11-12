"""Repeat calibrated forward travel and measure translation noise (feet input)."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

from robot_control.version import __version__


METERS_PER_FOOT = 0.3048
TRIALS = 10
COMMAND_PERIOD_S = 0.05


def parse_options(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('feet', type=float, help='Requested forward distance per trial, in feet')
    opts = parser.parse_args(args)
    if not math.isfinite(opts.feet) or opts.feet <= 0:
        parser.error('feet must be finite and positive')
    return opts


def load_settings(path):
    """Use the installed SLAM calibration without duplicating speed constants."""
    import yaml

    with path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    names = ('forward_left', 'forward_right', 'forward_speed_mps', 'pulse_s',
             'settle_s', 'observe_s', 'translation_noise', 'route_timeout_s')
    try:
        settings = {name: document['hw3_slam']['ros__parameters'][name] for name in names}
    except (KeyError, TypeError) as exc:
        raise ValueError('Expected hw3_slam.ros__parameters with motion settings') from exc
    for name, value in settings.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
    for name in ('forward_left', 'forward_right'):
        if not 0 < settings[name] <= 0.5:
            raise ValueError(f'{name} must be in (0, 0.5] for forward travel')
    for name in ('forward_speed_mps', 'route_timeout_s'):
        if settings[name] <= 0:
            raise ValueError(f'{name} must be positive')
    if settings['pulse_s'] < COMMAND_PERIOD_S:
        raise ValueError('pulse_s must be at least 0.05 s')
    for name in ('settle_s', 'observe_s', 'translation_noise'):
        if settings[name] < 0:
            raise ValueError(f'{name} must be nonnegative')
    return settings


def pulse_plan(feet, settings):
    """Match SLAM's capped forward pulses, including its minimum 50 ms pulse."""
    total = feet * METERS_PER_FOOT / settings['forward_speed_mps']
    cap = settings['pulse_s']
    count = int(total // cap)
    remainder = total - count * cap
    extra = remainder > 1e-9
    moving_s = count * cap + (max(COMMAND_PERIOD_S, remainder) if extra else 0)
    pause_s = settings['settle_s'] + settings['observe_s']
    if moving_s + (count + int(extra)) * pause_s > settings['route_timeout_s']:
        raise ValueError('Requested distance exceeds the SLAM route timeout; use fewer feet')
    pulses = [cap] * count
    if extra:
        pulses.append(max(COMMAND_PERIOD_S, remainder))
    if not pulses:
        raise ValueError('Requested distance is too small')
    return pulses


def summarize(rows, moving_s):
    """Separate systematic bias from sample scatter; one trial has no std."""
    errors = [row['error_m'] for row in rows]
    std = statistics.stdev(errors) if len(errors) >= 2 else None
    return {
        'trials': len(errors),
        'mean_error_m': statistics.mean(errors) if errors else None,
        'error_std_m': std,
        'error_std_ft': std / METERS_PER_FOOT if std is not None else None,
        'translation_noise_m_per_sqrt_s': std / math.sqrt(moving_s) if std is not None else None,
        'commanded_moving_s': moving_s,
        'covariance_denominator': len(errors) - 1 if len(errors) >= 2 else None,
    }


def display_statistics(logger, result):
    if result['error_std_m'] is None:
        logger.info('At least two measured trials are required for standard deviation.')
        return
    logger.info(
        f'{result["trials"]} trials: distance error std = {result["error_std_m"]:.6f} m '
        f'({result["error_std_ft"]:.6f} ft); mean error = {result["mean_error_m"]:+.6f} m; '
        f'translation_noise estimate = {result["translation_noise_m_per_sqrt_s"]:.6f} m/sqrt(s)')


class MotionDriver:
    """Publish at 20 Hz, stop between pulses, and leave serial ownership to the bridge."""

    def __init__(self, ros, node, message_type):
        self.ros = ros
        self.node = node
        self.message_type = message_type
        self.publisher = node.create_publisher(message_type, 'motor_commands', 1)

    def send(self, wheels):
        self.publisher.publish(self.message_type(data=list(wheels)))

    def check(self):
        if not self.ros.ok():
            raise RuntimeError('ROS shutdown')
        if self.node.count_publishers('motor_commands') > 1:
            raise RuntimeError('Another motor_commands publisher is active; stop other controllers')
        if self.publisher.get_subscription_count() == 0:
            raise RuntimeError('No motor_commands subscriber; start motor_control')

    def hold(self, wheels, duration_s, check=True):
        started = time.monotonic()
        deadline = started + duration_s
        next_send = started
        while self.ros.ok():
            now = time.monotonic()
            if now >= deadline:
                break
            if now >= next_send:
                if check:
                    self.check()
                self.send(wheels)
                next_send = now + COMMAND_PERIOD_S
            self.ros.spin_once(self.node, timeout_sec=max(
                0.0, min(next_send, deadline) - time.monotonic()))
        return time.monotonic() - started

    def stop(self):
        if self.ros.ok():
            self.send((0.0, 0.0))
            self.hold((0.0, 0.0), 0.2, check=False)

    def connect(self):
        deadline = time.monotonic() + 5.0
        # Allow discovery even if an unrelated subscriber appears immediately.
        self.hold((0.0, 0.0), 0.5, check=False)
        while self.ros.ok() and self.publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                break
            self.ros.spin_once(self.node, timeout_sec=0.05)
        self.check()

    def drive(self, pulses, settings):
        elapsed = []
        try:
            for pulse in pulses:
                self.check()
                elapsed.append(self.hold(
                    (settings['forward_left'], settings['forward_right']), pulse))
                self.send((0.0, 0.0))
                self.hold((0.0, 0.0), settings['settle_s'] + settings['observe_s'])
            self.check()
        finally:
            self.stop()
        return elapsed


def measured_distance():
    while True:
        answer = input('Physical forward distance traveled (feet), or q to finish: ').strip()
        if answer.lower() == 'q':
            return None
        try:
            value = float(answer)
            if math.isfinite(value) and value >= 0:
                return value
        except ValueError:
            pass
        print('Enter a finite, nonnegative distance in feet.')


def main(args=None):
    opts = parse_options(args)
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Float32MultiArray

    config = Path(get_package_share_directory('robot_control')) / 'config' / 'hw3_slam.yaml'
    settings = load_settings(config)
    pulses = pulse_plan(opts.feet, settings)
    moving_s = math.fsum(pulses)
    predicted_m = moving_s * settings['forward_speed_mps']
    directory = Path('~/.ros/translation_noise').expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    log_path = directory / f'translation_noise_{time.time_ns()}.jsonl'
    rows = []
    status = 'stopped_early'
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('translation_noise_sampler')
    driver = MotionDriver(rclpy, node, Float32MultiArray)
    try:
        with log_path.open('x', encoding='utf-8') as stream:
            def record(event, **values):
                stream.write(json.dumps(dict(
                    event=event, created_utc=datetime.now(timezone.utc).isoformat(),
                    **values), allow_nan=False) + '\n')
                stream.flush()

            record('config', software_version=__version__, config_path=str(config),
                   settings=settings, requested_distance_ft=opts.feet, requested_trials=TRIALS,
                   pulse_durations_s=pulses, commanded_moving_s=moving_s,
                   predicted_distance_m=predicted_m)
            node.get_logger().info(
                f'Translation noise v{__version__}; log: {log_path}\n'
                f'{TRIALS} trials, requested {opts.feet:g} ft each; '
                f'predicted {predicted_m / METERS_PER_FOOT:.6f} ft in {moving_s:.6f} moving s.\n'
                f'SLAM calibration: L={settings["forward_left"]}, R={settings["forward_right"]}, '
                f'speed={settings["forward_speed_mps"]} m/s; {len(pulses)} pulses per trial.')
            try:
                driver.connect()
                for number in range(1, TRIALS + 1):
                    answer = input(
                        f'Trial {number}/{TRIALS}: reset to the same starting pose, '
                        'clear the path, then press Enter to drive (q to finish): ').strip()
                    while answer and answer.lower() != 'q':
                        answer = input('Press Enter to drive, or q to finish: ').strip()
                    if answer.lower() == 'q':
                        break
                    record('trial_start', trial=number)
                    elapsed = driver.drive(pulses, settings)
                    record('motion_complete', trial=number, elapsed_pulse_s=elapsed)
                    actual_ft = measured_distance()
                    if actual_ft is None:
                        break
                    row = dict(trial=number, actual_distance_ft=actual_ft,
                               actual_distance_m=actual_ft * METERS_PER_FOOT,
                               predicted_distance_m=predicted_m,
                               error_m=actual_ft * METERS_PER_FOOT - predicted_m)
                    rows.append(row)
                    record('trial', **row)
                    result = summarize(rows, moving_s)
                    record('statistics', **result)
                    node.get_logger().info(f'Trial {number} error: {row["error_m"]:+.6f} m')
                    display_statistics(node.get_logger(), result)
                if len(rows) == TRIALS:
                    status = 'complete'
            except (KeyboardInterrupt, EOFError, ExternalShutdownException):
                status = 'interrupted'
            except Exception as exc:
                status = 'error'
                record('error', message=str(exc))
                node.get_logger().error(str(exc))
            finally:
                try:
                    driver.stop()
                finally:
                    result = summarize(rows, moving_s)
                    record('summary', status=status, **result)
                    display_statistics(node.get_logger(), result)
                    node.get_logger().info(f'Status: {status}. Saved log: {log_path}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if status == 'complete' else (130 if status == 'interrupted' else 1)


if __name__ == '__main__':
    raise SystemExit(main())
