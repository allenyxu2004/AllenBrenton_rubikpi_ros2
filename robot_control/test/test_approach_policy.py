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
        near = ApproachPolicy().step(0, 1.10, 0., True)[0]
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

    def test_deadband_floor_preserves_steering_and_stop(self):
        for bearing in [-0.1, 0., 0.1]:
            policy = ApproachPolicy(max_command=0.15, min_command=0.08)
            left, right = policy.step(0, 1.06, bearing, True)
            self.assertTrue(0.08 <= left <= 0.15 and 0.08 <= right <= 0.15)
            self.assertAlmostEqual((left - right) * bearing, abs((left - right) * bearing))
            self.assertEqual(policy.step(1, 1.0, bearing, True), (0., 0.))

    def test_requested_command_limit(self):
        left, right = ApproachPolicy(max_command=0.3, min_command=0.08).step(
            0, 3., 0.1, True)
        self.assertTrue(0.08 <= right < left <= 0.3)

    def test_interval_pause_and_resume(self):
        policy = ApproachPolicy(min_command=0.08, max_command=0.15)
        self.assertGreater(policy.step(0, 2., 0., True)[0], 0)
        self.assertEqual(policy.step(3.5, 2., 0., True), (0., 0.))
        self.assertEqual(policy.step(3.7, 2., 0., True), (0., 0.))
        self.assertGreater(policy.step(3.81, 2., 0., True)[0], 0)

    def test_delayed_tick_does_not_skip_pause(self):
        policy = ApproachPolicy()
        policy.step(0, 2., 0., True)
        self.assertEqual(policy.step(5, 2., 0., True), (0., 0.))
        self.assertEqual(policy.step(5.2, 2., 0., True), (0., 0.))

    def test_arrival_or_loss_during_pause_never_resumes(self):
        for fresh, distance in [(True, 0.99), (False, 2.)]:
            policy = ApproachPolicy()
            policy.step(0, 2., 0., True)
            policy.step(3.5, 2., 0., True)
            self.assertEqual(policy.step(3.6, distance, 0., fresh), (0., 0.))
            self.assertEqual(policy.step(4, 2., 0., True), (0., 0.))

    def test_large_bearing_aligns_in_place_both_directions(self):
        for sign in [-1, 1]:
            policy = ApproachPolicy(max_command=0.15, min_command=0.08)
            left, right = policy.step(0, 1.8, sign * math.radians(15), True)
            self.assertEqual((left, right), (sign * 0.2, -sign * 0.2))
            self.assertTrue(policy.aligning)
            policy.step(0.1, 1.8, sign * math.radians(5), True)
            self.assertTrue(policy.aligning)
            left, right = policy.step(0.2, 1.8, sign * math.radians(2), True)
            self.assertFalse(policy.aligning)
            self.assertGreater(min(left, right), 0)

    def test_alignment_respects_pause_and_tracking_stop(self):
        policy = ApproachPolicy()
        policy.step(0, 1.8, math.radians(15), True)
        self.assertEqual(policy.step(3.5, 1.8, math.radians(15), True), (0., 0.))
        self.assertEqual(policy.step(3.6, fresh=False), (0., 0.))
        self.assertEqual(policy.step(4., 1.8, 0., True), (0., 0.))

    def test_pivot_strength_independent_of_forward_speed(self):
        policy = ApproachPolicy(max_command=0.15, pivot_command=0.25)
        self.assertEqual(policy.step(0, 2., math.radians(10), True), (0.25, -0.25))
        left, right = policy.step(0.1, 2., 0., True)
        self.assertTrue(0 < left <= 0.15 and 0 < right <= 0.15)
        self.assertEqual(policy.step(0.2, 1., 0., True), (0., 0.))

    def test_hw1_pivots_are_pulsed_and_stop_on_arrival(self):
        for sign in [-1, 1]:
            policy = ApproachPolicy(use_hw1_turns=True)
            expected = (0.45, -0.3) if sign > 0 else (-0.45, 0.3)
            bearing = sign * math.radians(15)
            self.assertEqual(policy.step(0, 2., bearing, True), expected)
            self.assertEqual(policy.step(0.15, 2., bearing, True), (0., 0.))
            self.assertEqual(policy.step(0.3, 2., bearing, True), (0., 0.))
            self.assertEqual(policy.step(0.36, 2., bearing, True), expected)
            self.assertEqual(policy.step(0.4, 1., bearing, True), (0., 0.))

    def test_hw1_pivot_stops_on_tracking_loss(self):
        policy = ApproachPolicy(use_hw1_turns=True)
        policy.step(0, 2., math.radians(15), True)
        self.assertEqual(policy.step(0.05, fresh=False), (0., 0.))

    def test_coast_margin_stops_early_and_latches(self):
        policy = ApproachPolicy(target_distance=1., stop_margin=0.05)
        self.assertGreater(policy.step(0, 1.06, 0., True)[0], 0)
        self.assertEqual(policy.step(0.1, 1.05, 0., True), (0., 0.))
        self.assertEqual(policy.step(0.2, 1.2, 0., True), (0., 0.))
        no_margin = ApproachPolicy(stop_margin=0.)
        self.assertGreater(no_margin.step(0, 1.01, 0., True)[0], 0)

    def test_bad_configuration(self):
        for kw in [{'max_command': 0.5}, {'target_distance': -1},
                   {'max_duration': math.nan}, {'drive_seconds': 0},
                   {'pause_seconds': -1}, {'pivot_command': 0.31},
                   {'pivot_command': 0}, {'stop_margin': -0.01},
                   {'stop_margin': math.nan}, {'min_command': -0.01},
                   {'min_command': 0.08, 'max_command': 0.08}]:
            with self.assertRaises(ValueError):
                ApproachPolicy(**kw)


if __name__ == '__main__':
    unittest.main()
