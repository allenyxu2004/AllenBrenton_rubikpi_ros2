"""Exercise route startup and transitions without ROS or motor hardware."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from robot_control.approach_policy import Outcome
from robot_control.control_config import NODE_DEFAULTS
from robot_control.route_config import RouteStep
from robot_control.square_approach import SquareApproachPolicy
from robot_control.tag_route_steps import AlignTagPolicy


class RouteConfigurationTest(unittest.TestCase):
    def setUp(self):
        # Replace only ROS plumbing; use the real stage and policy implementations.
        fake_base = ModuleType('robot_control.approach_tag')
        fake_base.ApproachTag = type('ApproachTag', (), {})
        path = Path(__file__).parents[1] / 'robot_control/tag_route.py'
        spec = importlib.util.spec_from_file_location('route_under_test', path)
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {
                'rclpy': ModuleType('rclpy'),
                'robot_control.approach_tag': fake_base}):
            spec.loader.exec_module(self.module)
        self.route = self.module.TagRoute.__new__(self.module.TagRoute)
        self.settings = {key: NODE_DEFAULTS[key] for key in (
            'target_distance', 'max_command', 'max_duration', 'min_command',
            'drive_seconds', 'pause_seconds', 'pivot_command', 'use_hw1_turns',
            'stop_margin', 'loss_grace_seconds')}
        self.route.policy_settings = lambda: dict(self.settings)
        self.route.make_policy = lambda target_distance=None: SquareApproachPolicy(
            **dict(self.settings, target_distance=(target_distance if target_distance
                   is not None else self.settings['target_distance'])))
        self.route.reset_tracking = Mock()
        self.route.get_parameter = lambda name: SimpleNamespace(value=0.3)
        self.route.get_logger = Mock()
        self.route.report_event = Mock()
        self.route.stage_index = 0
        self.route.finished = False

    def test_first_stage_honors_tag_action_and_distance(self):
        for action, policy_type in (
                ('align', AlignTagPolicy), ('approach', SquareApproachPolicy)):
            with self.subTest(action=action), patch.object(
                    self.module, 'STAGES', [RouteStep(action, 42, 1.7)]):
                self.route.initialize_policy()
                self.assertEqual(self.route.tag_frame, 'tag_42')
                self.assertIsInstance(self.route.policy, policy_type)
                self.assertEqual(self.route.policy.target, 1.7)
                self.route.reset_tracking.assert_called()

    def test_empty_route_is_rejected(self):
        with patch.object(self.module, 'STAGES', []):
            with self.assertRaises(ValueError):
                self.route.initialize_policy()

    def test_success_is_independent_of_log_wording(self):
        self.route.policy = SimpleNamespace(
            outcome=Outcome.SUCCEEDED, reason='Different arrival wording')
        self.assertTrue(self.route._step_succeeded())
        self.route.policy.outcome = Outcome.FAILED
        self.route.policy.reason = 'Square-on goal reached: misleading text'
        self.assertFalse(self.route._step_succeeded())

    def test_transition_and_completion_use_configured_stage(self):
        stages = [RouteStep('align', 42), RouteStep('approach', 43, 2.1)]
        with patch.object(self.module, 'STAGES', stages):
            self.route.initialize_policy()
            self.route._advance_route()
            self.assertEqual(self.route.tag_frame, 'tag_43')
            self.assertEqual(self.route.policy.target, 2.1)
            self.assertFalse(self.route.finished)
            self.route._advance_route()
            self.assertTrue(self.route.finished)
            self.route.report_event.assert_called_once_with('complete')
            message = self.route.get_logger().info.call_args.args[0]
            self.assertIn('tag_43', message)
            self.assertIn('2.10 m', message)
