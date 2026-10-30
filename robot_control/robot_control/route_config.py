"""Route order and proven controller settings; edit this file to change the route."""
from typing import NamedTuple, Optional


class RouteStep(NamedTuple):
    """One route action; None distance uses TARGET_DISTANCE_M."""

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

# Firmware commands, not m/s. Pivots use the existing HW1 calibration.
TARGET_DISTANCE_M = 0.5
MIN_COMMAND = 0.08
MAX_COMMAND = 0.15
APPROACH_TIMEOUT_SECONDS = 60.0
ALIGNMENT_TIMEOUT_SECONDS = 30.0
DRIVE_SECONDS = 3.5
PAUSE_SECONDS = 0.3
FORWARD_ERROR_OFFSET_M = 0.05
POSE_TIMEOUT_SECONDS = 0.6
LOSS_GRACE_SECONDS = 1.5
CAMERA_FRAME = 'camera_frame'
# Forward-facing camera; no lateral/vertical offset or mounting rotation.
CAMERA_OFFSET_FROM_ROBOT_CENTER_M = 3.5 * 0.0254
CONTROL_PERIOD_SECONDS = 0.05
PROGRESS_LOG_SECONDS = 1.0
MIN_QUATERNION_NORM = 1e-6
REPORT_DIRECTORY = '~/.ros/route_reports'
