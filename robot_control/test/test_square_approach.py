"""Check square-on waypoint geometry and multi-stage arrival behaviour."""

import math
import unittest

from robot_control.square_approach import SquareApproachPolicy, square_goal
from robot_control.approach_policy import Outcome


class SquareApproachTest(unittest.TestCase):
    def test_front_facing_goal(self):
        self.assertEqual(square_goal(0., 2., (1., 0., 0., 0.), 1.), (0., 1., 0.))

    def test_rotated_plane_changes_goal_and_heading(self):
        angle = math.radians(15)
        q = (math.cos(angle / 2), 0., -math.sin(angle / 2), 0.)
        x, z, heading = square_goal(0., 2., q, 1.)
        self.assertAlmostEqual(x, -math.sin(angle))
        self.assertAlmostEqual(z, 2 - math.cos(angle))
        self.assertAlmostEqual(heading, angle)
        self.assertEqual(square_goal(0., 2., q, 1.),
                         square_goal(0., 2., tuple(-v for v in q), 1.))

    def test_invalid_or_wrong_facing_normal(self):
        for q in [(0., 0., 0., 0.), (0., 0., 0., 1.), (math.nan, 0., 0., 1.)]:
            with self.assertRaises(ValueError):
                square_goal(0., 2., q, 1.)

    def test_distance_alone_does_not_finish(self):
        policy = SquareApproachPolicy()
        policy.update_goal(0.15, 1.05, (1., 0., 0., 0.))
        policy.step(0., 1.06, math.atan2(0.15, 1.05), True)
        self.assertIsNone(policy.reason)
        self.assertEqual(policy.phase, 'POSITION')

    def test_orient_then_settle_and_latch(self):
        policy = SquareApproachPolicy(use_hw1_turns=True)
        angle = math.radians(10)
        q = (math.cos(angle / 2), 0., -math.sin(angle / 2), 0.)
        policy.update_goal(math.sin(angle), math.cos(angle), q)
        left, right = policy.step(0., 1., angle, True)
        self.assertGreater(left, 0.)
        self.assertLess(right, 0.)
        self.assertEqual(policy.phase, 'ORIENTATION')
        policy.update_goal(0., 1., (1., 0., 0., 0.))
        self.assertEqual(policy.step(0.1, 1., 0., True), (0., 0.))
        self.assertIsNone(policy.reason)
        self.assertEqual(policy.step(0.7, 1., 0., True), (0., 0.))
        self.assertIn('Square-on goal reached', policy.reason)
        policy.update_goal(0., 2., (1., 0., 0., 0.))
        self.assertEqual(policy.step(1., 2., 0., True), (0., 0.))

    def test_coast_outside_tolerance_does_not_report_success(self):
        policy = SquareApproachPolicy()
        policy.update_goal(0., 1., (1., 0., 0., 0.))
        policy.step(0., 1., 0., True)
        policy.update_goal(0., 0.85, (1., 0., 0., 0.))
        self.assertEqual(policy.step(0.6, 0.85, 0., True), (0., 0.))
        self.assertIn('behind robot', policy.reason)

    def test_tracking_loss_during_final_settle(self):
        policy = SquareApproachPolicy()
        policy.update_goal(0., 1., (1., 0., 0., 0.))
        policy.step(0., 1., 0., True)
        self.assertEqual(policy.step(0.1, fresh=False), (0., 0.))
        self.assertIn('lost', policy.reason)

    def test_visibility_boundary_and_outcome(self):
        for degrees, expected in [(20, Outcome.RUNNING), (20.1, Outcome.FAILED)]:
            with self.subTest(degrees=degrees):
                policy = SquareApproachPolicy()
                policy.update_goal(0., 2., (1., 0., 0., 0.))
                policy.step(0., 2., math.radians(degrees), True)
                self.assertEqual(policy.outcome, expected)
        policy = SquareApproachPolicy()
        policy.update_goal(0., 1., (1., 0., 0., 0.))
        policy.step(0., 1., 0., True)
        policy.step(.5, 1., 0., True)
        self.assertEqual(policy.outcome, Outcome.SUCCEEDED)
        policy.stop('Later error cannot overwrite latched success')
        self.assertEqual(policy.outcome, Outcome.SUCCEEDED)

    def test_deadline_and_visibility(self):
        policy = SquareApproachPolicy(max_duration=1.)
        policy.update_goal(0., 2., (1., 0., 0., 0.))
        policy.step(0., 2., 0., True)
        self.assertEqual(policy.step(1., 2., 0., True), (0., 0.))
        self.assertIn('time limit', policy.reason)
        policy = SquareApproachPolicy()
        policy.update_goal(1., 2., (1., 0., 0., 0.))
        self.assertEqual(policy.step(0., 2., math.radians(21), True), (0., 0.))


if __name__ == '__main__':
    unittest.main()
