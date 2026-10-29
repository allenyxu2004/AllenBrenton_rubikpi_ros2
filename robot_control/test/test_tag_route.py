"""Route progression and fail-stop tests."""
import unittest
from robot_control.tag_route_policy import RoutePolicy


class RouteTest(unittest.TestCase):
    def test_route(self):
        p = RoutePolicy()
        now = 0.
        for index, tag in enumerate([9, 8, 8, 7, 5, 5, 9]):
            self.assertEqual(p.target, tag)
            for _ in range(100):
                p.step(now, .5, 0., True)
                now += .05
                if p.index != index:
                    break
            self.assertEqual(p.index, index+1)
        self.assertEqual(p.state, 'DONE')
        self.assertEqual(p.step(now, 2., 0., True), (0., 0.))

    def test_loss_and_timeout(self):
        p = RoutePolicy()
        p.step(0., 2., 0., True)
        self.assertEqual(p.step(1.), (0., 0.))
        p.step(3.)
        self.assertIsNotNone(p.reason)
        p = RoutePolicy()
        p.step(0.)
        p.step(31.)
        self.assertIsNotNone(p.reason)

    def test_approach_interval_pause_and_resume(self):
        p = RoutePolicy()
        self.assertGreater(p.step(0., 2., 0., True)[0], 0)
        self.assertEqual(p.step(3.5, 2., 0., True), (0., 0.))
        self.assertEqual(p.step(3.7, 2., 0., True), (0., 0.))
        self.assertGreater(p.step(3.81, 2., 0., True)[0], 0)

    def test_brief_tracking_loss_recovers_without_failure(self):
        p = RoutePolicy()
        p.step(0., 2., 0., True)
        self.assertEqual(p.step(.3, fresh=False), (0., 0.))
        self.assertGreater(p.step(.6, 2., 0., True)[0], 0)
        self.assertIsNone(p.reason)

    def test_search_pause_uses_actual_stop_time(self):
        p = RoutePolicy()
        p.index = 1
        self.assertLess(p.step(0.)[0], 0)
        self.assertEqual(p.step(.3), (0., 0.))
        self.assertEqual(p.step(.45), (0., 0.))
        self.assertLess(p.step(.51)[0], 0)
