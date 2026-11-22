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

from robot_control.hw4.course import (
    Course, point_segment_distance, read_course, segment_box_distance, segment_box_interval,
)

from robot_control.hw4.localization import FixedMapLocalizer

from robot_control.hw4.planning import (
    PlanningError, Roadmap, VisibilityRoadmap, build_roadmap, plan_course,
)

COURSE_PATH = Path(__file__).resolve().parents[1] / 'config' / 'hw4_course.yaml'

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

if __name__ == '__main__':
    unittest.main()
