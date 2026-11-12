"""Measure stationary AprilTag scatter from distinct camera-frame timestamps."""

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

from robot_control.slam.observations import planar_observation, tag_id_from_frame
from robot_control.version import __version__


METERS_PER_FOOT = 0.3048
SAMPLE_FIELDS = ('source_stamp_ns', 'received_stamp_ns', 'forward_m', 'left_m',
                 'camera_x_m', 'camera_y_m', 'camera_z_m')


def parse_options(args=None):
    """Parse robot-relative tag position: x is left and y is forward, in feet."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag-id', default=4, type=int, help='AprilTag ID to sample (default: 4)')
    parser.add_argument('--x-ft', required=True, type=float,
                        help='Tag offset LEFT of robot center, feet (right is negative)')
    parser.add_argument('--y-ft', required=True, type=float,
                        help='Tag distance FORWARD of robot center, feet')
    parser.add_argument('--samples', type=int, default=200)
    parser.add_argument('--timeout-s', type=float, default=120.0)
    parser.add_argument('--config', type=Path,
                        help='SLAM YAML; default: installed config/hw3_slam.yaml')
    parser.add_argument('--output-dir', type=Path, default=Path('~/.ros/camera_noise'))
    parser.add_argument('--tf-topic', default='/tf')
    parser.add_argument('--command-topic', default='/motor_commands')
    opts = parser.parse_args(args)
    if opts.tag_id < 0 or opts.samples < 2:
        parser.error('--tag-id must be nonnegative and --samples must be at least 2')
    if not all(math.isfinite(v) for v in (opts.x_ft, opts.y_ft, opts.timeout_s)):
        parser.error('position and timeout must be finite')
    if opts.y_ft <= 0 or opts.timeout_s <= 0:
        parser.error('--y-ft and --timeout-s must be positive')
    return opts


def load_settings(path):
    """Read the same camera geometry and noise assumptions as the SLAM node."""
    import yaml

    with path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    try:
        params = document['hw3_slam']['ros__parameters']
        names = ('camera_frame', 'camera_forward_m', 'camera_left_m', 'camera_yaw_rad',
                 'measurement_noise_m', 'range_noise_fraction',
                 'max_observation_age_s', 'settle_s')
        settings = {name: params[name] for name in names}
    except (KeyError, TypeError) as exc:
        raise ValueError('Expected hw3_slam.ros__parameters with SLAM camera settings') from exc
    if not isinstance(settings['camera_frame'], str) or not settings['camera_frame']:
        raise ValueError('camera_frame must be a nonempty string')
    for name in names[1:]:
        value = settings[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
    for name in ('measurement_noise_m', 'max_observation_age_s'):
        if settings[name] <= 0:
            raise ValueError(f'{name} must be positive')
    for name in ('range_noise_fraction', 'settle_s'):
        if settings[name] < 0:
            raise ValueError(f'{name} must be nonnegative')
    return settings


class ObservationCollector:
    """Select one tag and fresh, increasing image stamps without a noise gate."""

    def __init__(self, tag_id, count, settings, started_ns):
        self.tag_id = tag_id
        self.count = count
        self.settings = settings
        self.minimum_stamp_ns = started_ns + round(settings['settle_s'] * 1e9)
        self.last_stamp_ns = None
        self.rows = []
        self.skipped = Counter()

    @property
    def complete(self):
        return len(self.rows) >= self.count

    def accept(self, transform, now_ns):
        """Return an accepted row or None; identical values at new stamps count."""
        if self.complete or tag_id_from_frame(transform.child_frame_id) != self.tag_id:
            return None
        c = self.settings
        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age_ns = now_ns - stamp_ns
        reason = None
        if transform.header.frame_id != c['camera_frame']:
            reason = 'wrong_camera_frame'
        elif stamp_ns < self.minimum_stamp_ns:
            reason = 'before_start_or_settling'
        elif age_ns < 0 or age_ns > round(c['max_observation_age_s'] * 1e9):
            reason = 'stale_or_future'
        elif self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            reason = 'duplicate_or_out_of_order'
        if reason:
            self.skipped[reason] += 1
            return None
        p = transform.transform.translation
        try:
            forward, left = planar_observation(
                p.x, p.y, p.z, c['camera_forward_m'], c['camera_left_m'], c['camera_yaw_rad'])
        except ValueError:
            self.skipped['invalid_translation'] += 1
            return None
        if not all(math.isfinite(v) for v in (forward, left)):
            self.skipped['invalid_translation'] += 1
            return None
        self.last_stamp_ns = stamp_ns
        row = dict(zip(SAMPLE_FIELDS, (stamp_ns, now_ns, forward, left, p.x, p.y, p.z)))
        self.rows.append(row)
        return row


def summarize(rows, x_ft, y_ft):
    """Separate sample scatter (N-1 covariance) from ground-truth bias and RMSE."""
    if len(rows) < 2:
        return None
    forward = [row['forward_m'] for row in rows]
    left = [row['left_m'] for row in rows]
    mean_f, mean_l = statistics.mean(forward), statistics.mean(left)
    var_f, var_l = statistics.variance(forward), statistics.variance(left)
    cross = math.fsum((f - mean_f) * (l - mean_l) for f, l in zip(forward, left))
    cross /= len(rows) - 1
    truth_f, truth_l = y_ft * METERS_PER_FOOT, x_ft * METERS_PER_FOOT
    sigma = math.sqrt((var_f + var_l) / 2)
    return {
        'coordinate_order': ['forward', 'left'],
        'mean_m': [mean_f, mean_l],
        'std_m': [math.sqrt(var_f), math.sqrt(var_l)],
        'std_ft': [math.sqrt(var_f) / METERS_PER_FOOT, math.sqrt(var_l) / METERS_PER_FOOT],
        'sample_covariance_m2': [[var_f, cross], [cross, var_l]],
        'isotropic_std_m': sigma,
        'isotropic_std_ft': sigma / METERS_PER_FOOT,
        'isotropic_covariance_m2': [[sigma**2, 0.0], [0.0, sigma**2]],
        'bias_m': [mean_f - truth_f, mean_l - truth_l],
        'rmse_m': [math.sqrt(statistics.mean((v - truth_f)**2 for v in forward)),
                   math.sqrt(statistics.mean((v - truth_l)**2 for v in left))],
        'covariance_denominator': len(rows) - 1,
    }


def save_report(directory, opts, settings, collector, status, elapsed_s):
    """Save complete or explicitly partial results beside the raw sample CSV."""
    distance_m = math.hypot(opts.x_ft, opts.y_ft) * METERS_PER_FOOT
    report = {
        'software_version': __version__,
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'status': status,
        'requested_samples': opts.samples,
        'collected_samples': len(collector.rows),
        'elapsed_s': elapsed_s,
        'tag_id': opts.tag_id,
        'input_position_ft': {'x_left': opts.x_ft, 'y_forward': opts.y_ft},
        'ground_truth_forward_left_m': [opts.y_ft * METERS_PER_FOOT,
                                       opts.x_ft * METERS_PER_FOOT],
        'ground_truth_planar_distance_m': distance_m,
        'configured_model_std_m_at_distance': (
            settings['measurement_noise_m'] + settings['range_noise_fraction'] * distance_m),
        'settings': settings,
        'config_path': str(opts.config),
        'tf_topic': opts.tf_topic,
        'command_topic': opts.command_topic,
        'skipped': dict(collector.skipped),
        'statistics': summarize(collector.rows, opts.x_ft, opts.y_ft),
        'notes': [
            'Measured scatter is around the sample mean; bias is mean minus supplied position.',
            'Unique image timestamps do not guarantee statistically independent errors.',
            'One distance estimates one noise level, not both range-model parameters.',
            'Robot and tag must remain stationary; command monitoring cannot detect manual motion.',
            'No EKF innovation gate or distance-based outlier trimming is applied.',
        ],
    }
    destination = directory / 'summary.json'
    with destination.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    return report


def main(args=None):
    opts = parse_options(args)
    if opts.config is None:
        from ament_index_python.packages import get_package_share_directory
        opts.config = Path(get_package_share_directory('robot_control')) / 'config' / 'hw3_slam.yaml'
    opts.config = opts.config.expanduser().resolve()
    settings = load_settings(opts.config)

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Float32MultiArray
    from tf2_msgs.msg import TFMessage

    directory = opts.output_dir.expanduser() / f'tag_{opts.tag_id}_{time.time_ns()}'
    directory.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('camera_noise_sampler')
    collector = ObservationCollector(
        opts.tag_id, opts.samples, settings, node.get_clock().now().nanoseconds)
    started = time.monotonic()
    status = 'collecting'
    try:
        with (directory / 'observations.csv').open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS)
            writer.writeheader()
            stream.flush()

            def on_tf(message):
                if status != 'collecting':
                    return
                now_ns = node.get_clock().now().nanoseconds
                for transform in message.transforms:
                    row = collector.accept(transform, now_ns)
                    if row is not None:
                        writer.writerow(row)
                        stream.flush()
                        if len(collector.rows) % 25 == 0 or collector.complete:
                            node.get_logger().info(f'Collected {len(collector.rows)}/{opts.samples}')

            def on_command(message):
                nonlocal status
                if any(not math.isfinite(value) or value != 0 for value in message.data):
                    status = 'motion_command_detected'

            node.create_subscription(TFMessage, opts.tf_topic, on_tf, qos_profile_sensor_data)
            node.create_subscription(
                Float32MultiArray, opts.command_topic, on_command, qos_profile_sensor_data)
            node.get_logger().info(
                f'Keep robot and tag {opts.tag_id} still. Collecting {opts.samples} frames; '
                f'x={opts.x_ft:g} ft LEFT, y={opts.y_ft:g} ft FORWARD. Saving to {directory}')
            next_progress = started + 10
            try:
                while rclpy.ok() and status == 'collecting' and not collector.complete:
                    if time.monotonic() - started >= opts.timeout_s:
                        status = 'timeout'
                        break
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if time.monotonic() >= next_progress:
                        node.get_logger().info(
                            f'Waiting: {len(collector.rows)}/{opts.samples}; '
                            f'skipped={dict(collector.skipped)}')
                        next_progress = time.monotonic() + 10
                if status == 'collecting':
                    status = 'complete' if collector.complete else 'shutdown'
            except (KeyboardInterrupt, ExternalShutdownException):
                status = 'interrupted'
    finally:
        try:
            if status == 'collecting':
                status = 'error'
            report = save_report(directory, opts, settings, collector, status, time.monotonic() - started)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    print(f'Status: {status}; collected {len(collector.rows)}/{opts.samples}')
    result = report['statistics']
    if result is not None:
        print(f'Measured forward/left std: {result["std_m"]} m')
        print(f'Measured camera std (isotropic): {result["isotropic_std_m"]:.6f} m '
              f'({result["isotropic_std_ft"]:.6f} ft)')
        print(f'Mean error (forward/left): {result["bias_m"]} m')
        print('Configured model std at supplied distance: '
              f'{report["configured_model_std_m_at_distance"]:.6f} m')
    else:
        print('Fewer than two samples: standard deviation cannot be estimated.')
    print(f'Saved: {directory / "observations.csv"}\nSaved: {directory / "summary.json"}')
    if status != 'complete':
        print('INCOMPLETE run: repeat while stationary with a visible tag and fresh camera timestamps.')
        return 130 if status == 'interrupted' else 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
