"""Analytic rectangle geometry and surveyed configuration; no geometry library."""

from dataclasses import dataclass
from functools import cached_property
import math
from pathlib import Path

import yaml

Point = tuple[float, float]
Rectangle = tuple[float, float, float, float]
EPS = 1e-9


def numbers(values, count):
    result = tuple(float(value) for value in values)
    if len(result) != count or not all(map(math.isfinite, result)):
        raise ValueError(f'Expected {count} finite coordinates, got {values}')
    return result


class UniqueLoader(yaml.SafeLoader):
    """Reject duplicate configuration keys, including landmark IDs."""


def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if key in result:
            raise ValueError(f'Duplicate YAML key: {key}')
        result[key] = loader.construct_object(value_node)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def segment_box_interval(a, b, rectangle):
    """Closed segment/rectangle intersection in segment parameters, or None."""
    lo, hi = 0.0, 1.0
    for axis in (0, 1):
        delta = b[axis] - a[axis]
        lower, upper = rectangle[axis], rectangle[axis + 2]
        if delta == 0:
            if not lower <= a[axis] <= upper:
                return None
        else:
            enter, leave = sorted(((lower - a[axis]) / delta, (upper - a[axis]) / delta))
            lo, hi = max(lo, enter), min(hi, leave)
            if lo > hi:
                return None
    return lo, hi


def interpolate(a, b, t):
    return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))


def point_segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0,
                                              ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
    return math.dist(point, interpolate(a, b, t))


def point_box_distance(point, rectangle):
    x0, y0, x1, y1 = rectangle
    return math.hypot(max(x0 - point[0], 0.0, point[0] - x1),
                      max(y0 - point[1], 0.0, point[1] - y1))


def segment_box_distance(a, b, rectangle):
    if segment_box_interval(a, b, rectangle) is not None:
        return 0.0
    x0, y0, x1, y1 = rectangle
    return min(point_box_distance(a, rectangle), point_box_distance(b, rectangle),
               *(point_segment_distance(p, a, b)
                 for p in ((x0, y0), (x0, y1), (x1, y0), (x1, y1))))


@dataclass(frozen=True)
class Course:
    bounds: Rectangle
    obstacles: tuple[Rectangle, ...]
    start: Point
    heading: float
    goal: Point
    landmarks: dict[int, Point]
    tag_size: float = 0.1524
    resolution: float = 0.05  # Offline heatmap spacing only.
    robot_radius: float = 0.15
    margin: float = 0.08
    waypoint_tolerance: float = 0.05
    voronoi_spacing: float = 0.025

    def __post_init__(self):
        numbers(self.bounds, 4)
        numbers(self.start, 2)
        numbers(self.goal, 2)
        values = (self.tag_size, self.resolution, self.robot_radius, self.margin,
                  self.waypoint_tolerance, self.voronoi_spacing, self.heading)
        if not all(map(math.isfinite, values)):
            raise ValueError('Course values must be finite')
        if min(self.tag_size, self.resolution, self.robot_radius,
               self.waypoint_tolerance, self.voronoi_spacing) <= 0:
            raise ValueError('Size, spacing, radius and tolerance must be positive')
        if self.margin < 0:
            raise ValueError('Margin must be nonnegative')
        x0, y0, x1, y1 = self.bounds
        if x1 - x0 <= 2 * self.padding or y1 - y0 <= 2 * self.padding:
            raise ValueError('Workspace has no interior after reserve inflation')
        if len(self.obstacles) > 64:
            raise ValueError('At most 64 rectangles supported')
        for rectangle in self.obstacles:
            left, bottom, right, top = numbers(rectangle, 4)
            if not (x0 <= left < right <= x1 and y0 <= bottom < top <= y1):
                raise ValueError('Obstacles must be ordered rectangles inside bounds')
        if not self.landmarks:
            raise ValueError('At least one mapped landmark is required')
        for tag, point in self.landmarks.items():
            if type(tag) is not int or not 0 <= tag < 587:
                raise ValueError('Landmark IDs must be 36h11 integers in [0,586]')
            x, y = numbers(point, 2)
            if not x0 <= x <= x1 or not y0 <= y <= y1:
                raise ValueError('Landmark centers must be inside workspace bounds')
        for name, point in [('start', self.start), ('goal', self.goal)]:
            if not self.feasible(point, point):
                raise ValueError(f'{name} is outside reserved free space')

    @property
    def padding(self):
        return self.robot_radius + self.margin + self.waypoint_tolerance

    @property
    def inset(self):
        x0, y0, x1, y1 = self.bounds
        r = self.padding
        return x0 + r, y0 + r, x1 - r, y1 - r

    @cached_property
    def inflated(self):
        r = self.padding
        return tuple((x0 - r, y0 - r, x1 + r, y1 + r) for x0, y0, x1, y1 in self.obstacles)

    def clearance(self, a, b):
        """Exact minimum center-to-hazard distance over a complete segment."""
        x0, y0, x1, y1 = self.bounds
        wall = min(min(x - x0, x1 - x, y - y0, y1 - y) for x, y in (a, b))
        return min([wall] + [segment_box_distance(a, b, r) for r in self.obstacles])

    def feasible(self, a, b):
        """Reserved boundary contact is legal; physical contact is not."""
        x0, y0, x1, y1 = self.inset
        if not all(x0 - EPS <= x <= x1 + EPS and y0 - EPS <= y <= y1 + EPS for x, y in (a, b)):
            return False
        for left, bottom, right, top in self.inflated:
            if segment_box_interval(a, b, (left + EPS, bottom + EPS, right - EPS, top - EPS)) is not None:
                return False
        return self.clearance(a, b) > self.robot_radius + self.margin

    @cached_property
    def boundary_segments(self):
        """Expose union boundaries with a rectangular cell arrangement.

        Internal occupied-cell edges are omitted. Extra collinear vertices are
        harmless. This handles overlapping rectangles without polygon booleans.
        """
        x0, y0, x1, y1 = self.inset
        xs = sorted({x0, x1, *(max(x0, min(x1, r[k])) for r in self.inflated for k in (0, 2))})
        ys = sorted({y0, y1, *(max(y0, min(y1, r[k])) for r in self.inflated for k in (1, 3))})
        free = set()
        for i in range(len(xs) - 1):
            for j in range(len(ys) - 1):
                x, y = (xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2
                if not any(l < x < r and b < y < t for l, b, r, t in self.inflated):
                    free.add((i, j))
        edges = []
        for i, j in sorted(free):
            a, b, c, d = (xs[i], ys[j]), (xs[i + 1], ys[j]), (xs[i + 1], ys[j + 1]), (xs[i], ys[j + 1])
            for neighbor, edge in [((i, j - 1), (a, b)), ((i + 1, j), (b, c)),
                                   ((i, j + 1), (c, d)), ((i - 1, j), (d, a))]:
                if neighbor not in free:
                    edges.append(edge)
        # Merge contiguous collinear pieces into boundary features. Voronoi
        # sites on the same feature must not generate artificial medial ridges.
        groups = {}
        for a, b in edges:
            axis = 0 if a[1] == b[1] else 1
            constant = a[1 - axis]
            groups.setdefault((axis, constant), []).append(sorted((a[axis], b[axis])))
        merged = []
        for (axis, constant), intervals in sorted(groups.items()):
            lo, hi = sorted(intervals)[0]
            for start, end in sorted(intervals)[1:]:
                if start <= hi + EPS:
                    hi = max(hi, end)
                else:
                    merged.append(((lo, constant), (hi, constant)) if axis == 0
                                  else ((constant, lo), (constant, hi)))
                    lo, hi = start, end
            merged.append(((lo, constant), (hi, constant)) if axis == 0
                          else ((constant, lo), (constant, hi)))
        return tuple(merged)

    def clip_free_segments(self, a, b):
        """Clip a ridge to inset bounds and split at rectangle boundaries."""
        interval = segment_box_interval(a, b, self.inset)
        if interval is None:
            return ()
        lo, hi = interval
        cuts = {lo, hi}
        for rectangle in self.inflated:
            hit = segment_box_interval(a, b, rectangle)
            if hit:
                cuts.update(t for t in hit if lo < t < hi)
        cuts = sorted(cuts)
        return tuple((interpolate(a, b, t0), interpolate(a, b, t1))
                     for t0, t1 in zip(cuts, cuts[1:])
                     if t1 - t0 > EPS and self.feasible(interpolate(a, b, t0), interpolate(a, b, t1)))


def read_course(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        data = yaml.load(stream, Loader=UniqueLoader)
    x, y, heading = numbers(data['start_xy_heading_deg'], 3)
    kwargs = dict(bounds=numbers(data['bounds_m'], 4), start=(x, y), heading=math.radians(heading),
                  goal=numbers(data['goal_m'], 2),
                  obstacles=tuple(numbers(r, 4) for r in data['obstacles_m']),
                  landmarks={tag: numbers(p, 2) for tag, p in data['landmarks_m'].items()})
    for field, key in [('tag_size', 'tag_size_m'), ('resolution', 'resolution_m'),
                       ('robot_radius', 'robot_radius_m'), ('margin', 'margin_m'),
                       ('waypoint_tolerance', 'waypoint_tolerance_m'),
                       ('voronoi_spacing', 'voronoi_spacing_m')]:
        if key in data:
            kwargs[field] = float(data[key])
    return Course(**kwargs)
