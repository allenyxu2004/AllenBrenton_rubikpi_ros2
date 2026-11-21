"""Load the measured HW4 map and compute continuous footprint clearance."""

from dataclasses import dataclass
from functools import cached_property
import math
from pathlib import Path

from shapely.geometry import LineString, Point as GeometryPoint, box
import yaml


Point = tuple[float, float]
Rectangle = tuple[float, float, float, float]


def numbers(values, count):
    result = tuple(float(value) for value in values)
    if len(result) != count or not all(map(math.isfinite, result)):
        raise ValueError(f'Expected {count} finite coordinates, got {values}')
    return result


@dataclass(frozen=True)
class Course:
    bounds: Rectangle
    obstacles: tuple[Rectangle, ...]
    start: Point
    heading: float
    goal: Point
    landmarks: dict[int, Point]
    tag_size: float
    resolution: float
    robot_radius: float
    margin: float
    waypoint_tolerance: float

    @property
    def padding(self):
        return self.robot_radius + self.margin + self.waypoint_tolerance

    @cached_property
    def obstacle_shapes(self):
        return tuple(box(*rectangle) for rectangle in self.obstacles)

    def clearance(self, a, b):
        """Minimum center-to-hazard distance along a whole straight segment."""
        x0, y0, x1, y1 = self.bounds
        # Clearance to an axis-aligned wall is affine along the segment.
        # Its minimum is at an endpoint and is negative beyond the wall.
        wall = min(min(x - x0, x1 - x, y - y0, y1 - y) for x, y in (a, b))
        if wall <= 0 or not self.obstacle_shapes:
            return wall
        segment = GeometryPoint(a) if tuple(a) == tuple(b) else LineString((a, b))
        return min(wall, *(segment.distance(shape) for shape in self.obstacle_shapes))


def read_course(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    bounds = numbers(data['bounds_m'], 4)
    obstacles = tuple(numbers(box, 4) for box in data['obstacles_m'])
    start_x, start_y, heading_deg = numbers(data['start_xy_heading_deg'], 3)
    tags = data['landmarks_m']
    if not tags or any(type(tag) is not int or not 0 <= tag < 587 for tag in tags):
        raise ValueError('landmarks_m requires integer 36h11 IDs in [0, 586]')
    landmarks = {tag: numbers(point, 2) for tag, point in tags.items()}
    resolution, radius, margin, tolerance, tag_size = numbers(
        [data[key] for key in ('resolution_m', 'robot_radius_m', 'margin_m',
                               'waypoint_tolerance_m', 'tag_size_m')], 5)
    course = Course(bounds, obstacles, (start_x, start_y), math.radians(heading_deg),
                    numbers(data['goal_m'], 2), landmarks, tag_size,
                    resolution, radius, margin, tolerance)
    x0, y0, x1, y1 = bounds
    if x0 >= x1 or y0 >= y1:
        raise ValueError('bounds_m must have ordered min and max coordinates')
    for left, bottom, right, top in obstacles:
        if not (x0 <= left < right <= x1 and y0 <= bottom < top <= y1):
            raise ValueError('Each obstacle must be an ordered rectangle inside bounds_m')
    if min(resolution, radius, tolerance, tag_size) <= 0 or margin < 0:
        raise ValueError('Resolution, radius, tolerance and tag size must be positive; margin >= 0')
    for name, point in [('start', course.start), ('goal', course.goal)]:
        if course.clearance(point, point) <= course.padding:
            raise ValueError(f'{name} is outside the navigable area after footprint inflation')
    if math.dist(course.start, course.goal) <= tolerance:
        raise ValueError('Start and goal must be farther apart than the waypoint tolerance')
    return course
