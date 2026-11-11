"""Offline checks for camera-noise sampling; ROS is not required."""

import contextlib
import csv
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from robot_control.slam.camera_noise import (
    METERS_PER_FOOT, ObservationCollector, main, parse_options, save_report, summarize,
)


SETTINGS = dict(camera_frame='camera_frame', camera_forward_m=0.0889,
                camera_left_m=0.0, camera_yaw_rad=0.0,
                measurement_noise_m=0.04, range_noise_fraction=0.02,
                max_observation_age_s=0.6, settle_s=0.35)
START = 10_000_000_000


def transform(stamp_ns, tag='tag36h11:7', parent='camera_frame', x=0.0, y=0.0, z=1.0):
    return SimpleNamespace(
        child_frame_id=tag,
        header=SimpleNamespace(frame_id=parent, stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)),
        transform=SimpleNamespace(translation=SimpleNamespace(x=x, y=y, z=z)))


class CameraNoiseTests(unittest.TestCase):
    def test_200_distinct_stamps_allow_identical_coordinates_and_stop_at_limit(self):
        collector = ObservationCollector(7, 200, SETTINGS, START)
        for index in range(205):
            stamp = START + 400_000_000 + index * 100_000_000
            observation = transform(stamp)
            collector.accept(observation, stamp + 10_000_000)
            collector.accept(observation, stamp + 20_000_000)
        self.assertTrue(collector.complete)
        self.assertEqual(len(collector.rows), 200)
        self.assertEqual(len({r['source_stamp_ns'] for r in collector.rows}), 200)
        self.assertEqual(collector.skipped['duplicate_or_out_of_order'], 199)
        result = summarize(collector.rows, 0, 6)
        self.assertEqual(result['isotropic_std_m'], 0)
        self.assertNotEqual(result['bias_m'][0], 0)

    def test_frame_time_and_validity_filters(self):
        collector = ObservationCollector(7, 200, SETTINGS, START)
        stamp = START + 1_000_000_000
        cases = [
            (transform(stamp, tag='tag_8'), stamp, None),
            (transform(stamp, parent='map'), stamp, 'wrong_camera_frame'),
            (transform(START + 100_000_000), stamp, 'before_start_or_settling'),
            (transform(stamp), stamp + 700_000_000, 'stale_or_future'),
            (transform(stamp), stamp - 1, 'stale_or_future'),
            (transform(stamp, z=-1), stamp, 'invalid_translation'),
            (transform(stamp, x=float('nan')), stamp, 'invalid_translation'),
        ]
        for tf, now, reason in cases:
            with self.subTest(reason=reason):
                previous = collector.skipped[reason]
                self.assertIsNone(collector.accept(tf, now))
                if reason:
                    self.assertEqual(collector.skipped[reason], previous + 1)
        self.assertEqual(len(collector.rows), 0)
        self.assertIsNotNone(collector.accept(transform(stamp, tag='tag_7'), stamp))
        self.assertIsNone(collector.accept(transform(stamp - 1), stamp))
        self.assertEqual(collector.skipped['duplicate_or_out_of_order'], 1)

    def test_conversion_includes_mounting_yaw_and_offsets(self):
        settings = dict(SETTINGS, camera_yaw_rad=math.pi / 2, camera_left_m=0.2)
        collector = ObservationCollector(7, 2, settings, START)
        stamp = START + 1_000_000_000
        row = collector.accept(transform(stamp, x=0.1, z=2), stamp)
        self.assertAlmostEqual(row['forward_m'], 0.1889)
        self.assertAlmostEqual(row['left_m'], 2.2)

    def test_sample_covariance_and_bias_are_separate(self):
        # Known sample variance: [1, 2, 3] -> 1; [2, 4, 6] -> 4, covariance 2.
        rows = [dict(forward_m=f, left_m=2 * f) for f in (1.0, 2.0, 3.0)]
        result = summarize(rows, 4 / METERS_PER_FOOT, 1 / METERS_PER_FOOT)
        self.assertEqual(result['sample_covariance_m2'], [[1, 2], [2, 4]])
        self.assertEqual(result['std_m'], [1, 2])
        self.assertAlmostEqual(result['isotropic_std_m'], math.sqrt(2.5))
        self.assertAlmostEqual(result['bias_m'][0], 1)
        self.assertAlmostEqual(result['bias_m'][1], 0)
        self.assertEqual(result['covariance_denominator'], 2)
        self.assertAlmostEqual(result['rmse_m'][0], math.sqrt(5 / 3))
        self.assertIsNone(summarize(rows[:1], 0, 6))
        self.assertIsNone(summarize([], 0, 6))

    def test_six_foot_truth_conversion_and_partial_report(self):
        opts = parse_options(['--tag-id', '7', '--x-ft', '0', '--y-ft', '6'])
        opts.config = Path('hw3_slam.yaml')
        collector = ObservationCollector(7, opts.samples, SETTINGS, START)
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            report = save_report(directory, opts, SETTINGS, collector, 'timeout', 120)
            loaded = json.loads((directory / 'summary.json').read_text())
            self.assertEqual(report, loaded)
            self.assertEqual(report['status'], 'timeout')
            self.assertEqual(report['collected_samples'], 0)
            self.assertIsNone(report['statistics'])
            self.assertAlmostEqual(report['ground_truth_forward_left_m'][0], 1.8288)
            self.assertAlmostEqual(report['configured_model_std_m_at_distance'], 0.076576)
            with self.assertRaises(FileExistsError):
                save_report(directory, opts, SETTINGS, collector, 'timeout', 120)

    def test_cli_rejects_invalid_trials(self):
        for extra in (['--samples', '1'], ['--timeout-s', '0'], ['--x-ft', 'nan'],
                      ['--y-ft', '-1'], ['--tag-id', '-1']):
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    parse_options(['--tag-id', '7', '--x-ft', '0', '--y-ft', '6'] + extra)
                self.assertEqual(error.exception.code, 2)

    def test_ros_adapter_saves_csv_and_summary_for_complete_and_interrupted_runs(self):
        # Simulated ROS delivery tests the actual command entry point and file lifecycle.
        for ending in ('complete', 'interrupted', 'motion_command_detected'):
            with self.subTest(ending=ending), tempfile.TemporaryDirectory() as temporary:
                callbacks = {}
                stamp = START
                node = Mock()
                node.get_clock.return_value.now.side_effect = lambda: SimpleNamespace(
                    nanoseconds=stamp)

                def subscribe(message_type, topic, callback, qos):
                    callbacks[topic] = callback

                def spin_once(unused_node, timeout_sec):
                    nonlocal stamp
                    stamp += 100_000_000
                    if stamp >= START + 1_000_000_000 and ending != 'complete':
                        if ending == 'interrupted':
                            raise KeyboardInterrupt
                        callbacks['/motor_commands'](SimpleNamespace(data=[0.2, 0.2]))
                        return
                    # Each message repeats its transform: it must count only once.
                    tf = transform(stamp, z=6 * METERS_PER_FOOT - 0.0889)
                    callbacks['/tf'](SimpleNamespace(transforms=[tf, tf]))

                node.create_subscription.side_effect = subscribe
                ros = SimpleNamespace(init=Mock(), ok=lambda: True, shutdown=Mock(),
                                      spin_once=spin_once)
                modules = {
                    'rclpy': ros,
                    'rclpy.node': SimpleNamespace(Node=Mock(return_value=node)),
                    'rclpy.executors': SimpleNamespace(ExternalShutdownException=RuntimeError),
                    'rclpy.qos': SimpleNamespace(qos_profile_sensor_data=object()),
                    'rclpy.signals': SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)),
                    'std_msgs': SimpleNamespace(),
                    'std_msgs.msg': SimpleNamespace(Float32MultiArray=object),
                    'tf2_msgs': SimpleNamespace(),
                    'tf2_msgs.msg': SimpleNamespace(TFMessage=object),
                }
                with patch.dict('sys.modules', modules), patch(
                        'robot_control.slam.camera_noise.load_settings', return_value=SETTINGS
                ), contextlib.redirect_stdout(io.StringIO()):
                    result = main(['--tag-id', '7', '--x-ft', '0', '--y-ft', '6',
                                   '--config', 'unused.yaml', '--output-dir', temporary])
                expected_code = {'complete': 0, 'interrupted': 130, 'motion_command_detected': 1}
                self.assertEqual(result, expected_code[ending])
                output, = Path(temporary).iterdir()
                report = json.loads((output / 'summary.json').read_text())
                with (output / 'observations.csv').open(newline='') as stream:
                    samples = list(csv.DictReader(stream))
                self.assertEqual(report['status'], ending)
                self.assertEqual(report['collected_samples'], len(samples))
                self.assertEqual(len({r['source_stamp_ns'] for r in samples}), len(samples))
                if ending == 'complete':
                    self.assertEqual(len(samples), 200)
                    self.assertEqual(report['statistics']['isotropic_std_m'], 0)
                    self.assertAlmostEqual(report['statistics']['bias_m'][0], 0)
                else:
                    self.assertGreater(len(samples), 1)
                    self.assertLess(len(samples), 200)
                node.destroy_node.assert_called_once()
                ros.shutdown.assert_called_once()


if __name__ == '__main__':
    unittest.main()
