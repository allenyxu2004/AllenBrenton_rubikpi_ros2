"""Check steering direction and stop conditions without robot hardware."""

import math
import unittest

from robot_control.approach_policy import ApproachPolicy


class ApproachPolicyTest(unittest.TestCase):
    def test_waits_for_tag(self):
        self.assertEqual(ApproachPolicy().step(0), (0., 0.))

    def test_drives_forward_and_corrects_both_directions(self):
        for bearing in (-0.1, 0., 0.1):
            left, right = ApproachPolicy().step(0, 1.5, bearing, True)
            self.assertTrue(0 < left <= 0.08 and 0 < right <= 0.08)
            self.assertAlmostEqual(math.copysign(1, left - right) if bearing else 0,
                                   math.copysign(1, bearing) if bearing else 0)

    def test_slows_near_target(self):
        far = ApproachPolicy().step(0, 1.5, 0., True)[0]
        near = ApproachPolicy().step(0, 1.05, 0., True)[0]
        self.assertLess(near, far)

    def test_arrival_never_restarts_or_reverses(self):
        policy = ApproachPolicy()
        self.assertEqual(policy.step(0, 0.95, 0., True), (0., 0.))
        self.assertEqual(policy.step(1, 1.5, 0., True), (0., 0.))

    def test_loss_latches_stop(self):
        policy = ApproachPolicy()
        policy.step(0, 1.5, 0., True)
        self.assertEqual(policy.step(1), (0., 0.))
        self.assertEqual(policy.step(2, 1.5, 0., True), (0., 0.))

    def test_excessive_bearing_and_invalid_pose(self):
        for distance, bearing in [(1.5, math.radians(21)), (math.nan, 0.)]:
            policy = ApproachPolicy()
            self.assertEqual(policy.step(0, distance, bearing, True), (0., 0.))
            self.assertIsNotNone(policy.reason)

    def test_deadline(self):
        policy = ApproachPolicy(max_duration=2)
        policy.step(0, 1.5, 0., True)
        self.assertEqual(policy.step(2, 1.5, 0., True), (0., 0.))

    def test_bad_configuration(self):
        for kw in [{'max_command': 0.5}, {'target_distance': -1},
                   {'max_duration': math.nan}]:
            with self.assertRaises(ValueError):
                ApproachPolicy(**kw)


if __name__ == '__main__':
    unittest.main()
