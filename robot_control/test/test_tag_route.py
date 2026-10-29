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
