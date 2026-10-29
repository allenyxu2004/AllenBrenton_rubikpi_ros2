"""Planar goal geometry and cautious square-on approach using visual poses."""

import math

from robot_control.approach_policy import ApproachPolicy
from robot_control.kinematic_control import FORWARD_LEFT, FORWARD_RIGHT


def square_goal(x, z, quaternion, standoff):
    """Return goal right/forward and final clockwise heading in robot coordinates.

    AprilTag PnP's +Z normal points out of the printed face toward the viewer.
    x/z already include the camera-to-robot-centre translation.
    """
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(sum(v * v for v in quaternion))
    if not all(math.isfinite(v) for v in (x, z, *quaternion)) or norm < 1e-6:
        raise ValueError('Invalid tag orientation')
    qx, qy, qz, qw = (v / norm for v in quaternion)
    nx = 2 * (qx * qz + qw * qy)
    ny = 2 * (qy * qz - qw * qx)
    nz = 1 - 2 * (qx * qx + qy * qy)
    if abs(ny) > math.sin(math.radians(30)) or nz >= -0.2:
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
        if self.reason is not None:
            return 0., 0.
        if not fresh or self.goal is None:
            self.settled_since = None
            return super().step(now, distance, bearing, False)
        if bearing is None or not math.isfinite(bearing) or abs(bearing) > math.radians(35):
            return self.stop('Tag more than 20 degrees off centre or invalid bearing')
        gx, gz, heading = self.goal
        error = math.hypot(gx, gz)
        if error > 0.08:
            self.phase = 'POSITION'
            self.settled_since = None
            if gz <= 0:
                return self.stop('Square-on goal is behind robot; reposition farther away')
            control_angle = math.atan2(gx, gz)
        elif abs(heading) > math.radians(3):
            self.phase = 'ORIENTATION'
            self.settled_since = None
            self.aligning = True
            control_angle = heading
        else:
            self.phase = 'SETTLE'
            if self.settled_since is None:
                self.settled_since = now
            # Keep safety/deadline accounting active, but discard motion commands.
            super().step(now, self.target + self.stop_margin + 1, 0., True)
            if self.reason is None and now - self.settled_since >= 0.5:
                self.stop('Square-on goal reached: position within 8 cm, heading within 3 deg')
            return 0., 0.
        # Existing pulse controller turns toward this waypoint. Limit only its
        # steering input; actual tag bearing is checked independently above.
        control_angle = max(-math.radians(19), min(math.radians(19), control_angle))
        # Do not latch the old scalar-range arrival stop before pose alignment.
        left, right = super().step(
            now, self.target + self.stop_margin + max(error, 0.01), control_angle, True)
        if self.phase == 'POSITION' and left > 0 and right > 0:
            # Reuse HW1's measured straight-line wheel ratio in this mode.
            left *= FORWARD_LEFT / FORWARD_RIGHT
            scale = min(1., self.max_command / max(left, right))
            left, right = left * scale, right * scale
            left = max(self.min_command, left)
            right = max(self.min_command, right)
        return left, right
