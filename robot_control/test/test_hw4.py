"""Offline HW4 geometry, roadmap and controller fault/arrival tests (no ROS)."""

from concurrent.futures import Future

from copy import deepcopy

from dataclasses import asdict, replace

import json

import math

from pathlib import Path

import tempfile

from threading import Event

from types import SimpleNamespace

import unittest

import networkx as nx

import numpy as np

from robot_control.hw4.control import (
    DEFAULTS, NavigationCore, PERIOD, STOP, pulse_envelope, validate_settings,
)

from robot_control.hw4.course import (
    Course, point_segment_distance, read_course, segment_box_distance, segment_box_interval,
)

from robot_control.hw4.localization import FixedMapLocalizer

from robot_control.hw4.planning import (
    PlanningError, Roadmap, VisibilityRoadmap, build_roadmap, plan_course,
)

COURSE_PATH = Path(__file__).resolve().parents[1] / 'config' / 'hw4_course.yaml'

class InlineExecutor:
    def submit(self, function, *args):
        future = Future()
        try:
            future.set_result(function(*args))
        except Exception as error:
            future.set_exception(error)
        return future

    def shutdown(self, **kwargs):
        pass

class HeldExecutor(InlineExecutor):
    def submit(self, function, *args):
        self.future = Future()
        self.job = (function, args)
        return self.future

    def finish(self):
        function, args = self.job
        self.future.set_result(function(*args))

def transform(core, now, tag=6, stamp=None):
    stamp = now if stamp is None else stamp
    x, y, heading = core.localizer.x[:3]
    tx, ty = core.course.landmarks[tag]
    forward = math.cos(heading) * (tx - x) + math.sin(heading) * (ty - y)
    left = -math.sin(heading) * (tx - x) + math.cos(heading) * (ty - y)
    optical_z = forward - core.cfg['camera_forward_m']
    return SimpleNamespace(child_frame_id=f'tag_{tag}',
                           header=SimpleNamespace(frame_id='camera_frame', stamp=SimpleNamespace(
                               sec=math.floor(stamp), nanosec=round((stamp - math.floor(stamp)) * 1e9))),
                           transform=SimpleNamespace(translation=SimpleNamespace(x=-left, y=0.0, z=optical_z)))

class GeometryTests(unittest.TestCase):
    def test_exact_swept_distances(self):
        box = (1, 1, 2, 2)
        self.assertEqual(segment_box_interval((0, 1.5), (3, 1.5), box), (1 / 3, 2 / 3))
        self.assertEqual(segment_box_distance((0, 1.5), (3, 1.5), box), 0)
        self.assertAlmostEqual(segment_box_distance((0, 0), (0, 3), box), 1)
        self.assertAlmostEqual(segment_box_distance((0, 0), (0, 0), box), math.sqrt(2))
        self.assertAlmostEqual(segment_box_distance((0, 0), (3, 0), box), 1)
        self.assertEqual(segment_box_distance((1, 0), (1, 3), box), 0)  # Physical tangency.
        self.assertAlmostEqual(point_segment_distance((2, 1), (0, 0), (1, 0)), math.sqrt(2))

    def test_whole_segment_not_only_endpoints(self):
        course = read_course(COURSE_PATH)
        self.assertTrue(course.feasible(course.start, course.start))
        self.assertTrue(course.feasible(course.goal, course.goal))
        self.assertFalse(course.feasible(course.start, course.goal))
        self.assertEqual(course.clearance(course.start, course.goal), 0)
        self.assertLess(course.clearance((-1, 1), (1, 1)), 0)
        self.assertFalse(course.feasible((-1, 1), (1, 1)))

    def test_layout_and_drive_gate(self):
        course = read_course(COURSE_PATH)
        self.assertEqual(course.start, (0.6096, 2.4384))
        self.assertEqual(course.goal, (2.4384, 0.6096))
        self.assertAlmostEqual(course.heading, -math.pi / 2)
        self.assertEqual(set(course.landmarks), {0, 2, 3, 4, 5, 6, 7, 8, 21, 22, 23, 24})
        self.assertEqual(course.landmarks[7], (0, .3048))
        with self.assertRaises(ValueError):
            course.require_drive_ready()
        with self.assertRaises(ValueError):
            replace(course, measurements_verified=True, calibration_verified=True).require_drive_ready()
        replace(course, measurements_verified=True, calibration_verified=True,
                verification_notes='synthetic test').require_drive_ready()

    def test_duplicates_and_invalid_config(self):
        text = COURSE_PATH.read_text(encoding='utf-8-sig')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'duplicate.yaml'
            path.write_text(text.replace('  7: [0.0, 0.3048]',
                                         '  7: [0.0, 0.3048]\n  7: [0.0, 0.5]'), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'Duplicate'):
                read_course(path)
        course = read_course(COURSE_PATH)
        for changes in [dict(robot_radius=-1), dict(heading=math.nan),
                        dict(start=(1.5, 1.5)), dict(measurements_verified='false'),
                        dict(tracking_allowance=.001), dict(bounds=(0, 0, 0, 1))]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(course, **changes)

    def test_union_edges_and_reserved_tangency(self):
        course = Course((0, 0, 10, 10), ((3, 3, 5, 5), (4, 4, 6, 6)), (1, 1), 0, (9, 9), {0: (0, 0)},
                        robot_radius=.1, margin=.05, tracking_allowance=.05,
                        uncertainty_budget=0, motion_reserve=0)
        for a, b in course.boundary_segments:
            mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            self.assertFalse(any(l + 1e-8 < mid[0] < r - 1e-8 and bot + 1e-8 < mid[1] < top - 1e-8
                                 for l, bot, r, top in course.inflated))
        self.assertTrue(course.feasible((2.8, 2.8), (5.2, 2.8)))
        self.assertFalse(course.feasible((2.8, 3), (5.2, 3)))
        pieces = course.clip_free_segments((0, 4), (10, 4))
        self.assertEqual(len(pieces), 2)
        self.assertTrue(all(course.feasible(a, b) for a, b in pieces))

class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.course = read_course(COURSE_PATH)

    def test_both_routes_exact_endpoints_and_clearance(self):
        for objective in ('safety', 'distance'):
            with self.subTest(objective=objective):
                plan = plan_course(self.course, objective)
                self.assertEqual(plan.points[0], self.course.start)
                self.assertEqual(plan.points[-1], self.course.goal)
                self.assertEqual(plan.planner, 'voronoi' if objective == 'safety' else 'visibility')
                self.assertGreaterEqual(plan.center_clearance + 1e-9, self.course.padding)
                for a, b in zip(plan.points, plan.points[1:]):
                    self.assertTrue(self.course.feasible(a, b))
                    # Independently sample each returned sweep against physical hazards.
                    for t in np.linspace(0, 1, 101):
                        p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                        self.assertGreater(self.course.clearance(p, p), self.course.robot_radius + self.course.margin)
                second = plan_course(self.course, objective)
                self.assertEqual(plan.points, second.points)
                self.assertAlmostEqual(plan.distance, second.distance)

    def test_visibility_astar_matches_dijkstra_and_analytic_box_detour(self):
        course = Course((0, 0, 10, 10), ((4, 4, 6, 6),), (2, 5), 0, (8, 5), {0: (0, 0)},
                        robot_radius=.1, margin=.1, tracking_allowance=.05,
                        uncertainty_budget=0, motion_reserve=0)
        graph = VisibilityRoadmap(course)
        plan = graph.solve('distance')
        self.assertAlmostEqual(plan.distance,
                               nx.shortest_path_length(graph.graph, course.start, course.goal, weight='length'))
        r = course.padding
        expected = 2 * math.hypot(2 - r, 1 + r) + 2 + 2 * r
        self.assertAlmostEqual(plan.distance, expected)
        direct = replace(course, obstacles=())
        self.assertEqual(plan_course(direct, 'distance').points, (direct.start, direct.goal))

    def test_widest_then_shortest_tie_break(self):
        course = replace(self.course, start=(.7, 2.3), goal=(2.3, .7), obstacles=())
        roadmap = Roadmap(course)
        a, b, c, d = course.start, course.goal, (.8, 1.0), (2, 2)
        roadmap.graph.add_edge(a, b, clearance=.3, length=1, kind='test')
        roadmap.graph.add_edge(a, c, clearance=.6, length=3, kind='test')
        roadmap.graph.add_edge(c, b, clearance=.6, length=3, kind='test')
        roadmap.graph.add_edge(a, d, clearance=.6, length=2, kind='test')
        roadmap.graph.add_edge(d, b, clearance=.6, length=2, kind='test')
        self.assertEqual(roadmap.solve('safety').raw_points, (a, d, b))

    def test_disconnected_and_stationary_missions(self):
        blocked = replace(self.course, obstacles=((1.3, 0, 1.7, 3.048),))
        for objective in ('distance', 'safety'):
            with self.subTest(objective=objective):
                with self.assertRaises(PlanningError) as failure:
                    plan_course(blocked, objective)
                self.assertEqual(failure.exception.code, 'disconnected')
                stationary = replace(self.course, goal=self.course.start)
                plan = plan_course(stationary, objective)
                self.assertEqual(plan.points, (stationary.start,))
                self.assertEqual(plan.distance, 0)

    def test_sampling_refinement_and_cancellation(self):
        values = []
        for spacing in (.05, .025, .0125):
            plan = plan_course(replace(self.course, voronoi_spacing=spacing), 'safety')
            values.append(plan.center_clearance)
        self.assertLess(max(values) - min(values), .02)
        cancel = Event()
        cancel.set()
        with self.assertRaises(PlanningError):
            plan_course(self.course, 'safety', cancel=cancel)
        with self.assertRaises(PlanningError):
            plan_course(self.course, 'distance', timeout_s=-1)
        with self.assertRaises(PlanningError):
            plan_course(replace(self.course, voronoi_spacing=1e-7), 'safety')

    def test_overlapping_and_wall_touching_obstacles(self):
        course = Course((0, 0, 10, 10), ((4, 3, 6, 5), (5, 4, 7, 6), (4, 0, 5, 2)),
                        (2, 8), -math.pi / 2, (8, 2), {0: (0, 0)})
        for objective in ('distance', 'safety'):
            plan = plan_course(course, objective)
            self.assertTrue(all(course.feasible(a, b) for a, b in zip(plan.points, plan.points[1:])))

class ControllerTests(unittest.TestCase):
    def make_core(self, drive=True, executor=None, **settings):
        course = replace(read_course(COURSE_PATH), measurements_verified=True,
                         calibration_verified=True, verification_notes='synthetic test only')
        events = []
        core = NavigationCore(course, drive=drive, settings=settings, now=0,
                              record=lambda event, **fields: events.append(dict(event=event, **fields)),
                              executor=executor or InlineExecutor())
        self.addCleanup(core.close)
        core.events = events
        return core

    def start_core(self, **settings):
        core = self.make_core(**settings)
        core.step(.05)
        core.observe_tf(transform(core, .7), .7, .7)
        core.step(.7)
        core.step(.75)
        return core

    def test_default_dry_run_and_verification_gate(self):
        core = self.make_core(drive=False)
        for now in (.05, .5, 1, 10):
            self.assertEqual(core.step(now), STOP)
        self.assertIsNotNone(core.plan)
        with self.assertRaises(ValueError):
            NavigationCore(read_course(COURSE_PATH), drive=True)
        for changes in [dict(min_pulse_s=.01), dict(max_pulse_s=.2), dict(forward_left=.6),
                        dict(max_tag_gap_s=.1), dict(max_replans=1.2), dict(pivot_drift_m=math.nan)]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_settings(dict(DEFAULTS, **changes))

    def test_fresh_distinct_post_settle_observations(self):
        core = self.make_core()
        core.observe_tf(transform(core, .1), .1, .1)
        self.assertIsNone(core.last_tag)
        core.observe_tf(transform(core, .5), .5, .5)
        self.assertAlmostEqual(core.last_tag, .5)
        for now, stamp in [(.6, .5), (.7, 1.0), (1.3, .6)]:
            core.observe_tf(transform(core, now, stamp=stamp), now, now)
        self.assertAlmostEqual(core.last_tag, .5)
        core.motion = (.2, 0)
        core.observe_tf(transform(core, 1.4), 1.4, 1.4)
        self.assertAlmostEqual(core.last_tag, .5)
        reasons = {row.get('reason') for row in core.events}
        self.assertTrue({'before_settling', 'duplicate_or_out_of_order', 'stale_or_future'} <= reasons)

    def test_no_tag_startup_and_repeated_stop_after_fault(self):
        core = self.make_core()
        self.assertEqual(core.step(31), STOP)
        self.assertEqual(core.outcome, 'fault')
        self.assertEqual(core.step(32), STOP)
        self.assertEqual(sum(r['event'] == 'fault' for r in core.events), 1)

    def test_competing_publisher_and_bridge_loss(self):
        for fault in ('publisher', 'bridge'):
            core = self.start_core()
            core.step(.8, publishers=2 if fault == 'publisher' else 1,
                      bridge_ready=fault != 'bridge')
            self.assertTrue(core.finished)
            self.assertEqual(core.wheels, STOP)

    def test_tag_loss_timing_gap_and_bad_covariance(self):
        core = self.start_core()
        core.stop(.8)
        core.step(3)
        self.assertIn('stale', core.events[-1]['reason'])
        core = self.start_core()
        self.assertNotEqual(core.motion, STOP)
        self.assertEqual(core.step(1.0), STOP)
        self.assertIn('timing', core.events[-1]['reason'])
        core = self.start_core()
        core.localizer.P[0, 0] = math.nan
        self.assertEqual(core.step(.8), STOP)
        self.assertIn('Invalid', core.events[-1]['reason'])

    def test_worker_failure_timeout_and_stale_result(self):
        executor = HeldExecutor()
        core = self.make_core(executor=executor)
        self.assertEqual(core.step(.1), STOP)
        self.assertIsNone(core.plan)
        core.step(16)
        self.assertEqual(core.outcome, 'fault')
        executor = HeldExecutor()
        core = self.make_core(executor=executor)
        core.localizer.x[:2] += (.01, -.08)
        executor.finish()
        core.step(.1)
        self.assertIsNone(core.plan)
        self.assertEqual(core.generation, 2)
        self.assertEqual(core.wheels, STOP)
        executor.future.set_exception(ValueError('synthetic search failure'))
        core.step(.2)
        self.assertEqual(core.outcome, 'fault')

    def test_uncertainty_and_delayed_stop_envelopes(self):
        core = self.make_core()
        envelope = pulse_envelope(core.course, core.localizer, core.cfg, 'forward', .05)
        self.assertGreater(envelope.duration_s, .05 + .15)
        self.assertGreater(envelope.position_bound_m, 3 * .05)
        self.assertGreater(envelope.motion_reserve_used_m, core.cfg['coast_distance_m'])
        core.localizer.P[0, 0] = .2 ** 2
        envelope = pulse_envelope(core.course, core.localizer, core.cfg, 'left', .05)
        self.assertFalse(envelope.safe)
        self.assertIn('uncertainty', envelope.reason)
        core = self.make_core()
        core.localizer.x[:2] = (.24, .8)
        core.localizer.x[2] = math.pi
        self.assertFalse(pulse_envelope(core.course, core.localizer, core.cfg, 'forward', .05).safe)
        self.assertFalse(pulse_envelope(core.course, core.localizer, core.cfg, 'right', .05).safe)

    def test_fixed_landmarks_and_covariance(self):
        core = self.make_core()
        x, p = core.localizer.x[3:].copy(), core.localizer.P[3:, :].copy()
        for step in range(15):
            core.localizer.predict(.2, .1, .05)
            core.observe_tf(transform(core, 1 + step * .1), 1 + step * .1, 1 + step * .1)
        np.testing.assert_array_equal(x, core.localizer.x[3:])
        np.testing.assert_array_equal(p, core.localizer.P[3:, :])
        self.assertGreaterEqual(np.linalg.eigvalsh(core.localizer.P).min(), -1e-12)

    def test_envelope_bounds_intermediate_prediction_covariances(self):
        core = self.make_core()
        random = np.random.default_rng(42)
        for action in ('forward', 'left', 'right'):
            for _ in range(8):
                factor = random.normal(0, .02, (3, 3))
                core.localizer.P[:3, :3] = factor @ factor.T
                envelope = pulse_envelope(core.course, core.localizer, core.cfg, action, .15)
                predicted = deepcopy(core.localizer)
                steps = math.ceil(envelope.duration_s / .005)
                speed = core.cfg['forward_speed_mps'] if action == 'forward' else 0
                rate = 0 if action == 'forward' else core.cfg[f'{action}_rate_rps']
                if action == 'right':
                    rate = -rate
                for _ in range(steps):
                    predicted.predict(speed, rate, envelope.duration_s / steps)
                    bound = 3 * math.sqrt(max(np.linalg.eigvalsh(predicted.P[:2, :2])))
                    self.assertLessEqual(bound, envelope.position_bound_m + 1e-10)

    def test_goal_needs_new_stopped_observation(self):
        core = self.start_core()
        core.stop(1)
        core.localizer.x[:2] = core.course.goal
        core.target = len(core.plan.points) - 1
        core.decide(1.6)
        self.assertFalse(core.finished)
        self.assertIsNotNone(core.goal_pending)
        core.decide(2.2)
        self.assertFalse(core.finished)
        # Tag 4 lies forward if the robot faces right at this synthetic goal.
        core.localizer.x[2] = 0
        core.observe_tf(transform(core, 2.25, 4), 2.25, 2.25)
        core.decide(2.3)
        self.assertEqual(core.outcome, 'complete')
        self.assertEqual(core.step(2.35), STOP)

    def test_blocked_corner_requests_stopped_replan(self):
        core = self.make_core()
        core.step(.05)
        # Accepting this synthetic corner must not connect through the obstacle.
        core.localizer.x[:2] = core.course.start
        core.plan = replace(core.plan, points=(core.course.start, core.course.start, core.course.goal))
        core.target = 1
        core.decide(.7)
        self.assertIsNone(core.plan)
        self.assertEqual(core.generation, 2)
        self.assertEqual(core.wheels, STOP)

    def test_ideal_feedback_reaches_goal_for_both_planners(self):
        # Ideal kinematic/filter feedback verifies sequencing, not real visibility,
        # stopping dynamics, physical calibration or real-world collision safety.
        for objective in ('safety', 'distance'):
            core = self.make_core()
            core.objective = objective
            core.plan = None
            core.future = None
            core.generation = 0
            core.request_plan(0, 'test objective')
            for tick in range(1, 9000):
                now = tick * PERIOD
                if core.motion == STOP and tick % 2 == 0:
                    for tag in core.course.landmarks:
                        core.observe_tf(transform(core, now, tag), now, now)
                core.step(now)
                if core.finished:
                    break
            self.assertEqual(core.outcome, 'complete', (objective, core.events[-3:]))
            self.assertLessEqual(math.dist(core.localizer.x[:2], core.course.goal), core.course.waypoint_tolerance)

if __name__ == '__main__':
    unittest.main()
