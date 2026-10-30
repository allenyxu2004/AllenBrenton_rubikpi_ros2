"""Planar goal geometry and cautious square-on approach using visual poses."""

import math

from robot_control.approach_policy import ApproachPolicy, Outcome, MAX_BEARING_RADIANS
from robot_control.control_config import MIN_QUATERNION_NORM
from robot_control.kinematic_control import FORWARD_LEFT, FORWARD_RIGHT


POSITION_TOLERANCE_M = 0.08
HEADING_TOLERANCE_RADIANS = math.radians(3)
SETTLE_SECONDS = 0.5
MAX_STEERING_RADIANS = math.radians(19)
MAX_TAG_TILT_RADIANS = math.radians(30)
MAX_FACING_NORMAL_Z = -0.2


def square_goal(x, z, quaternion, standoff):
    """Return goal right/forward and final clockwise heading in robot coordinates.

    AprilTag PnP's +Z normal points out of the printed face toward the viewer.
    x/z already include the camera-to-robot-centre translation.
    """
    norm = math.sqrt(sum(v * v for v in quaternion))
    if not all(math.isfinite(v) for v in (x, z, *quaternion)) or norm < MIN_QUATERNION_NORM:
        raise ValueError('Invalid tag orientation')
    qx, qy, qz, qw = (v / norm for v in quaternion)
    nx = 2 * (qx * qz + qw * qy)
    ny = 2 * (qy * qz - qw * qx)
    nz = 1 - 2 * (qx * qx + qy * qy)
    if abs(ny) > math.sin(MAX_TAG_TILT_RADIANS) or nz >= MAX_FACING_NORMAL_Z:
        raise ValueError('Square mode requires an upright tag facing the camera')
    length = math.hypot(nx, nz)
    nx, nz = nx / length, nz / length
    return x + standoff * nx, z + standoff * nz, math.atan2(-nx, -nz)


class SquareApproachPolicy(ApproachPolicy):
    """Approach the normal-line waypoint, align to its normal, then verify at rest.

    Supports modest initial offsets while the tag remains visible. Does not
    reverse or navigate around obstacles to reach a goal behind the robot.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.goal = None
        self.phase = 'WAIT'
        self.settled_since = None

    def update_goal(self, x, z, quaternion):
        self.goal = square_goal(x, z, quaternion, self.target)

    def step(self, now, distance=None, bearing=None, fresh=False):
        if not self.ready(now, fresh and self.goal is not None):
            self.settled_since = None
            return 0., 0.
        if (bearing is None or not math.isfinite(bearing)
                or abs(bearing) > MAX_BEARING_RADIANS):
            return self.stop('Tag more than 20 degrees off centre or invalid bearing')
        gx, gz, heading = self.goal
        error = math.hypot(gx, gz)
        if error > POSITION_TOLERANCE_M:
            self.phase = 'POSITION'
            self.settled_since = None
            if gz <= 0:
                return self.stop('Square-on goal is behind robot; reposition farther away')
            control_angle = math.atan2(gx, gz)
        elif abs(heading) > HEADING_TOLERANCE_RADIANS:
            self.phase = 'ORIENTATION'
            self.settled_since = None
            self.aligning = True
            control_angle = heading
        else:
            self.phase = 'SETTLE'
            if self.settled_since is None:
                self.settled_since = now
            if now - self.settled_since >= SETTLE_SECONDS:
                self.stop('Square-on goal reached: '
                          f'position within {POSITION_TOLERANCE_M:g} m, '
                          f'heading within {math.degrees(HEADING_TOLERANCE_RADIANS):g} deg',
                          Outcome.SUCCEEDED)
            return 0., 0.
        control_angle = max(-MAX_STEERING_RADIANS,
                            min(MAX_STEERING_RADIANS, control_angle))
        left, right = self.motion(
            now, error + self.stop_margin, control_angle,
            force_align=self.phase == 'ORIENTATION')
        if self.phase == 'POSITION' and left > 0 and right > 0:
            # Reuse HW1's measured straight-line wheel ratio in this mode.
            left *= FORWARD_LEFT / FORWARD_RIGHT
            scale = min(1., self.max_command / max(left, right))
            left, right = left * scale, right * scale
            left = max(self.min_command, left)
            right = max(self.min_command, right)
        return left, right
