"""Check optical/world conversion and mixed ROS launch log extraction."""

import importlib.util
import json
import math
from pathlib import Path

import unittest


SCRIPT = Path(__file__).resolve().parents[2] / 'scripts' / 'parse_route_log.py'
SPEC = importlib.util.spec_from_file_location('parse_route_log', SCRIPT)
report = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(report)


class ReportTests(unittest.TestCase):
    def test_automatic_file_is_unique_flushed_and_parseable(self):
        import tempfile
        helper_path = SCRIPT.parents[1] / 'robot_control/robot_control/report_log.py'
        spec = importlib.util.spec_from_file_location('report_log', helper_path)
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        with tempfile.TemporaryDirectory() as directory:
            first = helper.ReportLog(Path(directory) / 'nested', 'tag_route', 'abcd1234')
            second = helper.ReportLog(Path(directory) / 'nested', 'tag_route', 'abcd1234')
            try:
                self.assertNotEqual(first.path, second.path)
                self.assertRegex(first.path.name,
                                 r'^tag_route_\d{8}T\d{6}\.\d{6}-0[78]00_abcd1234\.log$')
                record = dict(version=1, run='abcd1234', event='start')
                first.write('HW2_REPORT ' + json.dumps(record))
                self.assertEqual(report.read_records(first.path), [record])
            finally:
                first.close()
                second.close()

    def test_front_tag_recovers_rotated_robot_pose(self):
        for yaw in (0, 90, -90, 180):
            angle = math.radians(yaw)
            record = dict(translation_m=[0.2, 0, 1], quaternion_xyzw=[0, 1, 0, 0])
            forward = 1.0889
            tag = dict(x_m=3 + math.cos(angle)*forward + math.sin(angle)*0.2,
                       y_m=4 + math.sin(angle)*forward - math.cos(angle)*0.2,
                       normal_yaw_deg=yaw+180)
            x, y, theta = report.robot_pose(record, tag, 0.0889)
            self.assertAlmostEqual(x, 3)
            self.assertAlmostEqual(y, 4)
            self.assertAlmostEqual(math.cos(theta), math.cos(angle))
            self.assertAlmostEqual(math.sin(theta), math.sin(angle))

    def test_extract_launch_prefix_and_ansi_suffix(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'run.log'
            record = dict(version=1, run='abc', event='start')
            path.write_text('noise\n[node-3] [INFO] HW2_REPORT '
                            + json.dumps(record) + '\x1b[0m\n')
            self.assertEqual(report.read_records(path), [record])

    def test_parser_outputs(self):
        import subprocess
        import sys
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            common = dict(version=1, run='abc', stage=1, tag='tag_9')
            records = [dict(common, event='start', elapsed_s=0, camera_forward_m=0.0889),
                       dict(common, event='observation', elapsed_s=0.1, source_stamp_s=100,
                            observed_tag='tag_9', translation_m=[0, 0, 1],
                            quaternion_xyzw=[0, 1, 0, 0]),
                       dict(common, event='arrival', elapsed_s=1, source_stamp_s=101,
                            position_error_m=0.02, heading_error_deg=1),
                       dict(common, event='complete', elapsed_s=1.1)]
            log = root / 'run.log'
            log.write_text('\n'.join('prefix HW2_REPORT ' + json.dumps(r) for r in records))
            tag_map = root / 'tags.json'
            tag_map.write_text(json.dumps(dict(tags=dict(tag_9=dict(
                x_m=1.0889, y_m=0, normal_yaw_deg=180)))))
            subprocess.run([sys.executable, str(SCRIPT), str(log), '--tag-map',
                            str(tag_map), '--output', str(root / 'report')], check=True)
            self.assertIn('0.02', (root / 'report/waypoint_errors.csv').read_text())
            self.assertIn('<circle', (root / 'report/trajectory.svg').read_text())
            self.assertIn('0.0,0.0,0.0', (root / 'report/trajectory.csv').read_text())
