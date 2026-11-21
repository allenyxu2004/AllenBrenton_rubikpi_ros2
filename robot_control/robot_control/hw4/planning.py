"""A* and maximum-bottleneck search over the same eight-neighbor grid."""

from dataclasses import dataclass
from itertools import pairwise
import math

import networkx as nx

from robot_control.hw4.course import Point


@dataclass(frozen=True)
class Plan:
    points: tuple[Point, ...]
    grid_points: tuple[Point, ...]
    grid_distance: float
    distance: float
    clearance: float  # Minimum distance from the robot envelope to any hazard.


class CourseGrid:
    """Cell centers plus exact endpoints, with swept clearance on each edge."""

    def __init__(self, course):
        self.course = course
        self.points = {}
        self.graph = nx.Graph()
        x0, y0, x1, y1 = course.bounds
        step = course.resolution
        columns = math.ceil((x1 - x0) / step)
        rows = math.ceil((y1 - y0) / step)
        for i in range(columns):
            for j in range(rows):
                point = (x0 + (i + 0.5) * step, y0 + (j + 0.5) * step)
                if course.clearance(point, point) > course.padding:
                    self.points[i, j] = point

        self.graph.add_nodes_from(self.points)
        for i, j in self.points:
            for dx, dy in ((0, 1), (1, 0), (1, 1), (1, -1)):
                neighbor = (i + dx, j + dy)
                if neighbor not in self.points:
                    continue
                if dx and dy and ((i + dx, j) not in self.points
                                  or (i, j + dy) not in self.points):
                    continue
                self.connect((i, j), neighbor)

        for name, point in [('start', course.start), ('goal', course.goal)]:
            self.points[name] = point
            self.graph.add_node(name)
            i = math.floor((point[0] - x0) / step)
            j = math.floor((point[1] - y0) / step)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    cell = (i + dx, j + dy)
                    if cell in self.points:
                        self.connect(name, cell)
        if math.dist(course.start, course.goal) <= step:
            self.connect('start', 'goal')

    def connect(self, a, b):
        start, end = self.points[a], self.points[b]
        clearance = self.course.clearance(start, end) - self.course.padding
        if clearance > 0:
            self.graph.add_edge(a, b, length=math.dist(start, end), clearance=clearance)

    def widest_clearance(self):
        """The path in a maximum spanning tree maximizes the bottleneck."""
        tree = nx.maximum_spanning_tree(self.graph, weight='clearance')
        path = nx.shortest_path(tree, 'start', 'goal')
        return min(tree[a][b]['clearance'] for a, b in pairwise(path))

    def shortest_path(self, threshold):
        def allowed(a, b):
            return self.graph[a][b]['clearance'] >= threshold

        def heuristic(a, b):
            return math.dist(self.points[a], self.points[b])

        graph = nx.subgraph_view(self.graph, filter_edge=allowed)
        path = nx.astar_path(graph, 'start', 'goal', heuristic=heuristic, weight='length')
        return [self.points[node] for node in path]

    def plan(self, objective):
        if objective not in ('distance', 'safety'):
            raise ValueError('Objective must be distance or safety')
        try:
            threshold = self.widest_clearance() if objective == 'safety' else 0.0
            grid_points = self.shortest_path(threshold)
        except nx.NetworkXNoPath as error:
            raise ValueError('No collision-free route on the configured grid') from error
        points = shortcut(self.course, grid_points, threshold)
        edges = list(pairwise(points))
        grid_distance = sum(math.dist(a, b) for a, b in pairwise(grid_points))
        return Plan(tuple(points), tuple(grid_points), grid_distance,
                    sum(math.dist(a, b) for a, b in edges),
                    min(self.course.clearance(a, b) for a, b in edges)
                    - self.course.robot_radius)


def shortcut(course, points, threshold):
    """Skip waypoints only if the replacement respects the search clearance."""
    result = [points[0]]
    index = 0
    while index < len(points) - 1:
        for target in range(len(points) - 1, index, -1):
            clearance = course.clearance(points[index], points[target]) - course.padding
            if clearance > 0 and clearance >= threshold:
                break
        result.append(points[target])
        index = target
    return result
