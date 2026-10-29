"""Route order and proven controller settings; edit this file to change the route."""
from typing import NamedTuple, Optional


class RouteStep(NamedTuple):
    """One route action; None distance uses the normal target_distance parameter."""

    action: str
    tag_id: int
    distance: Optional[float] = None


STAGES = [
    RouteStep('approach', 9),
    RouteStep('align', 8),
    RouteStep('approach', 8),
    RouteStep('align', 7),
    RouteStep('approach', 7),
    RouteStep('align', 5),
    RouteStep('approach', 5),
    RouteStep('align', 9),
    RouteStep('approach', 9, distance=1.5),
]

# Same settings as the working standalone approach_tag command, except the
# route's normal standoff is 0.5 m. ROS parameter overrides still take priority.
ROUTE_DEFAULTS = {
    'tag_frame': 'tag_9',
    'target_distance': 0.5,
    'square_to_tag': True,
    'min_command': 0.08,
    'max_command': 0.15,
    'max_duration': 60.0,
    'use_hw1_turns': True,
    'drive_seconds': 3.5,
    'pause_seconds': 0.3,
}
ALIGNMENT_TIMEOUT_SECONDS = 30.0
