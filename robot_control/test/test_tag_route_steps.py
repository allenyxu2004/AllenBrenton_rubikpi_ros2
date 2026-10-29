"""Test turn-only steps added around the existing approach controller."""
import math
import unittest
from robot_control.tag_route_steps import AlignTagPolicy, STAGES, ALIGN_COMPLETE
from robot_control.kinematic_control import LEFT_TURN_LEFT, LEFT_TURN_RIGHT


class RouteStepsTest(unittest.TestCase):
    def make_policy(self):
        return AlignTagPolicy(max_command=.15, min_command=.08,
                              max_duration=30., use_hw1_turns=True,
                              loss_grace_seconds=1.5)

    def test_sequence(self):
        self.assertEqual(STAGES, [
            ('approach', 9, None), ('align', 8, None), ('approach', 8, None),
            ('align', 7, None), ('approach', 7, None), ('align', 5, None),
            ('approach', 5, None), ('align', 9, None), ('approach', 9, 1.5)])

    def test_search_pulse_and_actual_pause(self):
        p = self.make_policy()
        self.assertEqual(p.step(0), (LEFT_TURN_LEFT, LEFT_TURN_RIGHT))
        self.assertEqual(p.step(.3), (0., 0.))
        self.assertEqual(p.step(.45), (0., 0.))
        self.assertEqual(p.step(.51), (LEFT_TURN_LEFT, LEFT_TURN_RIGHT))

    def test_alignment_only_turns_then_completes(self):
        p = self.make_policy()
        left, right = p.step(0, 2., math.radians(10), True)
        self.assertGreater(left, 0)
        self.assertLess(right, 0)
        self.assertEqual(p.step(.1, 2., 0., True), (0., 0.))
        self.assertIsNone(p.reason)
        p.step(.61, 2., 0., True)
        self.assertEqual(p.reason, ALIGN_COMPLETE)
        self.assertEqual(p.step(1., 2., 1., True), (0., 0.))

    def test_loss_recovers_then_latches(self):
        p = self.make_policy()
        p.step(0., 2., 0., True)
        self.assertEqual(p.step(.2), (0., 0.))
        p.step(.5, 2., 0., True)
        self.assertIsNone(p.reason)
        p.step(.7)
        p.step(2.3)
        self.assertIn('lost', p.reason)

    def test_search_deadline(self):
        p = self.make_policy()
        p.step(0.)
        self.assertEqual(p.step(30.), (0., 0.))
        self.assertIsNotNone(p.reason)
