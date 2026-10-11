from dataclasses import dataclass


@dataclass(frozen=True)
class Waypoint:
    # target pose in the world frame: x, y in meters, theta in radians (CCW positive)
    x: float
    y: float
    theta: float


def load_waypoints(path):
    # read "x,y,theta" lines; blank lines and lines starting with '#' are skipped
    waypoints = []
    with open(path) as f:
        for line_num, line in enumerate(f, start=1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            fields = [field.strip() for field in line.split(',')]
            if len(fields) != 3:
                raise ValueError(
                    f'{path}:{line_num}: expected "x,y,theta", got "{line}"')

            try:
                x, y, theta = (float(field) for field in fields)
            except ValueError:
                raise ValueError(
                    f'{path}:{line_num}: non-numeric value in "{line}"') from None

            waypoints.append(Waypoint(x, y, theta))

    if not waypoints:
        raise ValueError(f'{path}: no waypoints found')

    return waypoints
