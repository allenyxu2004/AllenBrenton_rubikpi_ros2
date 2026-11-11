"""Repeat calibrated pivots and estimate rotation noise from measured angles."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

from robot_control.version import __version__
from robot_control.slam.translation_noise import MotionDriver, COMMAND_PERIOD_S


def parse_options(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('direction', choices=['left', 'right'])
    parser.add_argument('--angle', type=float, default=90.0,
                        help='Requested angle magnitude in degrees (default: 90)')
    parser.add_argument('--trials', type=int, default=10)
    opts = parser.parse_args(args)
    if not math.isfinite(opts.angle) or opts.angle <= 0:
        parser.error('angle must be finite and positive')
    if opts.trials < 2:
        parser.error('trials must be at least two')
    return opts


def load_settings(path, direction):
    import yaml

    with path.open(encoding='utf-8') as stream:
        params = yaml.safe_load(stream)['hw3_slam']['ros__parameters']
    names = (f'{direction}_left', f'{direction}_right', f'{direction}_rate_rps',
             'pulse_s', 'settle_s', 'observe_s', 'rotation_noise', 'route_timeout_s')
    settings = {name: params[name] for name in names}
    for name, value in settings.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
    left, right = settings[f'{direction}_left'], settings[f'{direction}_right']
    if not (0 < abs(left) <= 0.5 and 0 < abs(right) <= 0.5):
        raise ValueError('Turn commands must have magnitudes in (0, 0.5]')
    if not ((left < 0 < right) if direction == 'left' else (right < 0 < left)):
        raise ValueError('Turn command signs do not match the selected direction')
    if settings[f'{direction}_rate_rps'] <= 0 or settings['route_timeout_s'] <= 0:
        raise ValueError('Turn rate and timeout must be positive')
    if settings['pulse_s'] < COMMAND_PERIOD_S:
        raise ValueError('pulse_s must be at least 0.05 seconds')
    if min(settings['settle_s'], settings['observe_s'], settings['rotation_noise']) < 0:
        raise ValueError('Pauses and noise must be nonnegative')
    settings['turn_rate_rps'] = settings[f'{direction}_rate_rps']
    # MotionDriver accepts wheel commands under these keys for either primitive.
    settings['forward_left'] = left
    settings['forward_right'] = right
    return settings


def pulse_plan(angle_deg, settings):
    total = math.radians(angle_deg) / settings['turn_rate_rps']
    cap = settings['pulse_s']
    count = int(total // cap)
    remainder = total - count * cap
    pulses = [cap] * count
    if remainder > 1e-9:
        pulses.append(max(COMMAND_PERIOD_S, remainder))
    if not pulses:
        raise ValueError('Requested angle is too small')
    duration = math.fsum(pulses) + len(pulses) * (settings['settle_s'] + settings['observe_s'])
    if duration > settings['route_timeout_s']:
        raise ValueError('Requested turn exceeds the route timeout')
    return pulses


def summarize(rows, moving_s):
    errors = [row['error_rad'] for row in rows]
    std = statistics.stdev(errors) if len(errors) >= 2 else None
    mean = statistics.mean(errors) if errors else None
    return {
        'trials': len(errors),
        'mean_error_deg': math.degrees(mean) if mean is not None else None,
        'error_std_deg': math.degrees(std) if std is not None else None,
        'error_std_rad': std,
        'rotation_noise_rad_per_sqrt_s': std / math.sqrt(moving_s) if std is not None else None,
        'commanded_moving_s': moving_s,
        'covariance_denominator': len(errors) - 1 if len(errors) >= 2 else None,
    }


def display_statistics(logger, result):
    if result['error_std_rad'] is None:
        logger.info('At least two measured trials are required for standard deviation.')
        return
    logger.info(
        f'{result["trials"]} trials: angle error std = {result["error_std_deg"]:.3f} deg; '
        f'mean error = {result["mean_error_deg"]:+.3f} deg; '
        f'rotation_noise estimate = {result["rotation_noise_rad_per_sqrt_s"]:.6f} rad/sqrt(s)')


def measured_angle():
    while True:
        answer = input('Actual turn magnitude (degrees; e.g. 88 or 92), or q to finish: ').strip()
        if answer.lower() == 'q':
            return None
        try:
            value = float(answer)
            if math.isfinite(value) and value >= 0:
                return value
        except ValueError:
            pass
        print('Enter a finite, nonnegative angle magnitude in degrees.')


def main(args=None):
    opts = parse_options(args)
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Float32MultiArray

    config = Path(get_package_share_directory('robot_control')) / 'config' / 'hw3_slam.yaml'
    settings = load_settings(config, opts.direction)
    pulses = pulse_plan(opts.angle, settings)
    moving_s = math.fsum(pulses)
    predicted_deg = math.degrees(moving_s * settings['turn_rate_rps'])
    directory = Path('~/.ros/rotation_noise').expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    log_path = directory / f'rotation_noise_{time.time_ns()}.jsonl'
    rows = []
    status = 'stopped_early'
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('rotation_noise_sampler')
    driver = MotionDriver(rclpy, node, Float32MultiArray)
    try:
        with log_path.open('x', encoding='utf-8') as stream:
            def record(event, **values):
                stream.write(json.dumps(dict(
                    event=event, created_utc=datetime.now(timezone.utc).isoformat(),
                    **values), allow_nan=False) + '\n')
                stream.flush()

            record('config', software_version=__version__, config_path=str(config),
                   settings=settings, direction=opts.direction, requested_angle_deg=opts.angle, requested_trials=opts.trials,
                   pulse_durations_s=pulses, commanded_moving_s=moving_s,
                   predicted_angle_deg=predicted_deg)
            node.get_logger().info(
                f'Rotation noise v{__version__}; log: {log_path}\n'
                f'{opts.trials} {opts.direction} trials, requested {opts.angle:g} degrees each; '
                f'predicted {predicted_deg:.6f} degrees in {moving_s:.6f} moving s.\n'
                f'Commands L={settings["forward_left"]}, R={settings["forward_right"]}; '
                f'rate={settings["turn_rate_rps"]} rad/s; {len(pulses)} pulses per trial.')
            try:
                driver.connect()
                for number in range(1, opts.trials + 1):
                    answer = input(
                        f'Trial {number}/{opts.trials}: reset to the same starting pose, '
                        'clear the path, then press Enter to drive (q to finish): ').strip()
                    while answer and answer.lower() != 'q':
                        answer = input('Press Enter to drive, or q to finish: ').strip()
                    if answer.lower() == 'q':
                        break
                    record('trial_start', trial=number)
                    elapsed = driver.drive(pulses, settings)
                    record('motion_complete', trial=number, elapsed_pulse_s=elapsed)
                    actual_deg = measured_angle()
                    if actual_deg is None:
                        break
                    row = dict(trial=number, actual_angle_deg=actual_deg,
                               predicted_angle_deg=predicted_deg,
                               error_deg=actual_deg - predicted_deg,
                               error_rad=math.radians(actual_deg - predicted_deg))
                    rows.append(row)
                    record('trial', **row)
                    result = summarize(rows, moving_s)
                    record('statistics', **result)
                    node.get_logger().info(f'Trial {number} error: {row["error_deg"]:+.3f} degrees')
                    display_statistics(node.get_logger(), result)
                if len(rows) == opts.trials:
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
