"""Visibility and sampled-boundary Voronoi roadmaps, using library graph search."""

from dataclasses import dataclass
from itertools import combinations, pairwise
import math
import time

import networkx as nx
import numpy as np
from scipy.spatial import QhullError, Voronoi

from robot_control.hw4.course import EPS, Point, interpolate


class PlanningError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class Plan:
    planner: str
    points: tuple[Point, ...]
    raw_points: tuple[Point, ...]
    distance: float
    center_clearance: float
    clearance: float  # Body-to-hazard clearance.
    residual_clearance: float
    graph_nodes: tuple[Point, ...]
    graph_edges: tuple[tuple[Point, Point, str], ...]
    sites: tuple[Point, ...]
    build_s: float
    search_s: float
    site_spacing: float


    @property
    def grid_points(self):
        """Legacy plot API until the roadmap reporting migration."""
        return self.raw_points

    @property
    def grid_distance(self):
        return sum(math.dist(a, b) for a, b in pairwise(self.raw_points))


def collinear_only(points):
    result = []
    for p in points:
        while len(result) >= 2:
            a, b = result[-2:]
            ab, bc = (b[0] - a[0], b[1] - a[1]), (p[0] - b[0], p[1] - b[1])
            if (abs(ab[0] * bc[1] - ab[1] * bc[0]) > 1e-12
                    or ab[0] * bc[0] + ab[1] * bc[1] < 0):
                break
            result.pop()
        result.append(p)
    return tuple(result)


class Roadmap:
    name = ''

    def __init__(self, course, check=None):
        self.course = course
        self.check = check or (lambda: None)
        self.graph = nx.Graph()
        self.sites = ()
        self.build_s = 0.0

    def vertex(self, p):
        key = tuple(round(float(v), 12) for v in p)
        self.graph.add_node(key)
        return key

    def connect(self, a, b, kind='roadmap'):
        self.check()
        if a == b or not self.course.feasible(a, b):
            return False
        self.graph.add_edge(a, b, length=math.dist(a, b),
                            clearance=self.course.clearance(a, b), kind=kind)
        return True

    def solve(self, objective):
        self.check()
        started = time.monotonic()
        start, goal = self.course.start, self.course.goal
        if start == goal:
            raw = (start,)
        else:
            try:
                threshold = 0.0
                if objective == 'safety':
                    tree = nx.maximum_spanning_tree(self.graph, weight='clearance')
                    widest = nx.shortest_path(tree, start, goal)
                    threshold = min(tree[a][b]['clearance'] for a, b in pairwise(widest))
                graph = nx.subgraph_view(self.graph, filter_edge=lambda a, b:
                self.graph[a][b]['clearance'] >= threshold - EPS)
                raw = tuple(nx.astar_path(graph, start, goal,
                                          heuristic=lambda a, b: math.dist(a, b), weight='length'))
            except (nx.NetworkXNoPath, nx.NodeNotFound) as error:
                raise PlanningError('disconnected',
                                    f'No route on {self.name}; check clearance and endpoint connections') from error
        self.check()
        # No greedy shortcuts: retain the defining Voronoi route and its clearance.
        points = collinear_only(raw)
        if not all(self.course.feasible(a, b) for a, b in pairwise(points)):
            raise PlanningError('validation', 'A returned route segment failed independent validation')
        edges = list(pairwise(points)) or [(points[0], points[0])]
        center = min(self.course.clearance(a, b) for a, b in edges)
        return Plan(self.name, points, raw, sum(math.dist(a, b) for a, b in edges), center,
                    center - self.course.robot_radius, center - self.course.padding,
                    tuple(self.graph.nodes),
                    tuple((a, b, d['kind']) for a, b, d in self.graph.edges(data=True)),
                    self.sites, self.build_s, time.monotonic() - started,
                    self.course.voronoi_spacing if self.name == 'voronoi' else 0.0)

    def endpoints(self):
        # Exact endpoints must not be rounded to a lattice or boundary sample.
        self.graph.add_nodes_from((self.course.start, self.course.goal))


class VisibilityRoadmap(Roadmap):
    name = 'visibility'

    def __init__(self, course, check=None):
        super().__init__(course, check)
        started = time.monotonic()
        for edge in course.boundary_segments:
            for p in edge:
                if course.feasible(p, p):
                    self.vertex(p)
        self.endpoints()
        for a, b in combinations(sorted(self.graph.nodes), 2):
            self.connect(a, b, 'visibility')
        self.build_s = time.monotonic() - started


class VoronoiRoadmap(Roadmap):
    name = 'voronoi'

    def __init__(self, course, check=None):
        super().__init__(course, check)
        started = time.monotonic()
        owners = {}
        for feature, (a, b) in enumerate(course.boundary_segments):
            self.check()
            count = max(1, math.ceil(math.dist(a, b) / course.voronoi_spacing))
            if count + len(owners) > 6000:
                raise PlanningError('site_limit', 'Voronoi spacing creates too many boundary sites (limit 6000)')
            for i in range(count + 1):
                p = tuple(round(v, 12) for v in interpolate(a, b, i / count))
                owners.setdefault(p, set()).add(feature)
        self.sites = tuple(sorted(owners))
        try:
            diagram = Voronoi(np.array(self.sites))
        except (QhullError, ValueError) as error:
            raise PlanningError('degenerate', 'Cannot construct Voronoi roadmap; inspect boundary sites') from error
        extent = 8 * math.hypot(course.bounds[2] - course.bounds[0], course.bounds[3] - course.bounds[1])
        center = np.mean(diagram.points, axis=0)
        for site_ids, vertices in zip(diagram.ridge_points, diagram.ridge_vertices):
            self.check()
            if owners[self.sites[site_ids[0]]] & owners[self.sites[site_ids[1]]]:
                continue  # Two samples on one straight wall are not a medial ridge.
            if -1 in vertices:
                finite = [v for v in vertices if v >= 0]
                if not finite:
                    continue
                a = diagram.vertices[finite[0]]
                p, q = diagram.points[site_ids]
                tangent = q - p
                normal = np.array([-tangent[1], tangent[0]]) / np.linalg.norm(tangent)
                direction = 1 if np.dot((p + q) / 2 - center, normal) >= 0 else -1
                b = a + direction * extent * normal
            else:
                a, b = diagram.vertices[vertices]
            for left, right in course.clip_free_segments(tuple(a), tuple(b)):
                self.connect(self.vertex(left), self.vertex(right))
        self.endpoints()
        for endpoint in dict.fromkeys((course.start, course.goal)):
            self.attach(endpoint)
        self.connect(course.start, course.goal, 'connector')
        self.build_s = time.monotonic() - started

    def attach(self, endpoint):
        # Only connect to the medial roadmap, not to the other endpoint's
        # connectors. Interior projections split edges and retain their scores.
        candidates = set()
        for a, b, data in list(self.graph.edges(data=True)):
            self.check()
            if data['kind'] != 'roadmap':
                continue
            candidates.update((a, b))
            dx, dy = b[0] - a[0], b[1] - a[1]
            t = ((endpoint[0] - a[0]) * dx + (endpoint[1] - a[1]) * dy) / (dx * dx + dy * dy)
            if not EPS < t < 1 - EPS:
                continue
            p = tuple(round(v, 12) for v in interpolate(a, b, t))
            if not self.course.feasible(endpoint, p):
                continue
            p = self.vertex(p)
            self.graph.remove_edge(a, b)
            self.connect(a, p)
            self.connect(p, b)
            candidates.add(p)
        for p in sorted(candidates):
            self.connect(endpoint, p, 'connector')


def build_roadmap(course, objective, check=None):
    if objective not in ('distance', 'safety'):
        raise ValueError('objective must be safety or distance')
    return (VoronoiRoadmap if objective == 'safety' else VisibilityRoadmap)(course, check)


def plan_course(course, objective, timeout_s=15.0, cancel=None):
    """Bound Python construction loops; failed/cancelled results never drive."""
    deadline = time.monotonic() + timeout_s

    def check():
        if (cancel is not None and cancel.is_set()) or time.monotonic() > deadline:
            raise PlanningError('timeout', 'Planning cancelled or deadline exceeded')

    roadmap = build_roadmap(course, objective, check)
    return roadmap.solve(objective)


class CourseGrid:
    """Compatibility adapter for legacy HW4 callers during roadmap migration."""

    def __init__(self, course):
        self.course = course
        self.points = {}
        x0, y0, x1, y1 = course.bounds
        step = course.resolution
        for i in range(math.ceil((x1-x0)/step)):
            for j in range(math.ceil((y1-y0)/step)):
                point = (x0+(i+.5)*step, y0+(j+.5)*step)
                if course.feasible(point, point):
                    self.points[i, j] = point

    def plan(self, objective):
        return plan_course(self.course, objective)
