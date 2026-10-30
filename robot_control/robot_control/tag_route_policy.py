"""Motion, square-on approach and turn-only alignment for the HW2 route."""

import math
from enum import Enum

from robot_control import route_config as cfg

from robot_control.kinematic_control import (
    LEFT_TURN_LEFT, LEFT_TURN_RIGHT,
    RIGHT_TURN_LEFT, RIGHT_TURN_RIGHT, FORWARD_LEFT, FORWARD_RIGHT,
)


MAX_BEARING_RADIANS = math.radians(20)
ALIGN_ENTER_RADIANS = math.radians(8)
ALIGN_EXIT_RADIANS = math.radians(3)
PIVOT_PAUSE_SECONDS = 0.4
PIVOT_PULSE_SECONDS = 0.05
FORWARD_GAIN = 0.25
STEERING_GAIN = 0.12
MAX_STEERING_FRACTION = 0.5
POSITION_TOLERANCE_M = 0.08
HEADING_TOLERANCE_RADIANS = math.radians(3)
SETTLE_SECONDS = 0.5
MAX_TAG_TILT_RADIANS = math.radians(30)
MAX_FACING_NORMAL_Z = -0.2
ALIGN_COMPLETE = 'Route alignment complete'
SEARCH_PULSE_SECONDS = 0.1
SEARCH_PAUSE_SECONDS = 0.2
CENTER_TOLERANCE_RADIANS = math.radians(3)
CENTER_SETTLE_SECONDS = 0.5
MAX_STEERING_RADIANS = math.radians(19)


class Outcome(Enum):
    RUNNING = 'running'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'


class MotionPolicy:
    """Shared calibrated motion and latched safety stops."""

    def __init__(self, target, max_duration):
        values = (target, max_duration, cfg.MIN_COMMAND, cfg.MAX_COMMAND,
                  cfg.DRIVE_SECONDS, cfg.PAUSE_SECONDS, cfg.LOSS_GRACE_SECONDS,
                  cfg.FORWARD_ERROR_OFFSET_M)
        if (not all(math.isfinite(v) for v in values) or target <= 0 or max_duration <= 0
                or not 0 <= cfg.MIN_COMMAND < cfg.MAX_COMMAND <= 0.3
                or cfg.DRIVE_SECONDS <= 0 or cfg.PAUSE_SECONDS < 0
                or cfg.LOSS_GRACE_SECONDS < 0 or cfg.FORWARD_ERROR_OFFSET_M < 0):
            raise ValueError('Invalid route motion settings')
        self.target = target
        self.max_duration = max_duration
        self.lost_since = None
        self.pivot_started_at = None
        self.pivot_duration = 0.0
        self.pivot_pause_until = 0.0
        self.pivot_direction = None
        self.segment_started_at = None
        self.pause_started_at = None
        self.paused = False
        self.aligning = False
        self.started_at = None
        self.tracking = False
        self.reason = None
        self.outcome = Outcome.RUNNING
        self.last_turn_direction = None
        self.turn_pause_until = None

    def pause_after_turn(self, now):
        """Stop before changing motion and require a post-settle camera frame."""
        self.last_turn_direction = None
        self.pivot_started_at = None
        self.turn_pause_until = now + PIVOT_PAUSE_SECONDS

    def step(self, now, bearing=None, fresh=False, observation_time=None):
        """Gate all motion after a turn, including transitions into driving.

        observation_time is the image capture time mapped onto the monotonic
        clock, not the time its delayed TF message arrived.
        """
        if (self.last_turn_direction is not None
                and self.pivot_started_at is not None
                and now - self.pivot_started_at >= self.pivot_duration):
            self.pause_after_turn(now)
        if self.turn_pause_until is not None:
            # Keep timeouts and tracking-loss stops active while stationary.
            if not self.ready(now, fresh, allow_search=True):
                return 0.0, 0.0
            # Initial search must remain possible before any tag is visible.
            # Once tracking starts, every turn requires a settled observation.
            if (now < self.turn_pause_until or (self.tracking and (
                    observation_time is None or not math.isfinite(observation_time)
                    or observation_time < self.turn_pause_until))):
                return 0.0, 0.0
            self.turn_pause_until = None

        left, right = self._step(now, bearing, fresh)
        direction = (1 if left > 0 else -1) if left * right < 0 else None
        if self.last_turn_direction is not None and direction != self.last_turn_direction:
            self.pause_after_turn(now)
            return 0.0, 0.0
        self.last_turn_direction = direction
        return left, right

    def stop(self, reason, outcome=Outcome.FAILED):
        if self.reason is None:
            self.reason = reason
            self.outcome = outcome
        return 0.0, 0.0

    def ready(self, now, fresh, allow_search=False):
        """Check latched stops, deadline and tracking loss before any motion."""
        if self.started_at is None:
            self.started_at = now
        if self.reason is not None:
            return False
        if now - self.started_at >= self.max_duration:
            self.stop('Test time limit reached')
            return False
        if not fresh:
            if self.tracking:
                if self.lost_since is None:
                    self.lost_since = now
                self.pivot_started_at = None
                if now - self.lost_since >= cfg.LOSS_GRACE_SECONDS:
                    self.stop('Tag lost or stale; restart controller to retry')
            return allow_search and not self.tracking and self.reason is None
        self.lost_since = None
        self.tracking = True
        return True

    def interval_paused(self, now):
        """Measure full pauses from actual transitions, even after delayed ticks."""
        # Use actual transition times so a delayed callback cannot skip a pause.
        if self.segment_started_at is None:
            self.segment_started_at = now
        if self.paused:
            if now - self.pause_started_at < cfg.PAUSE_SECONDS:
                return True
            self.paused = False
            self.segment_started_at = now
        elif (cfg.PAUSE_SECONDS > 0
              and now - self.segment_started_at >= cfg.DRIVE_SECONDS):
            self.paused = True
            self.pause_started_at = now
            return True
        return False

    def motion(self, now, distance_error, bearing, force_align=False):
        """Choose a pivot or forward steering without applying arrival rules."""
        if self.interval_paused(now):
            return 0.0, 0.0
        # Align before translating when lateral error is substantial. Hysteresis
        # prevents repeated mode switching at one bearing threshold.
        if force_align or abs(bearing) >= ALIGN_ENTER_RADIANS:
            self.aligning = True
        elif abs(bearing) <= ALIGN_EXIT_RADIANS:
            self.aligning = False
        if self.aligning:
            return self.pivot(now, bearing)
        self.pivot_started_at = None
        self.pivot_pause_until = 0.0
        return self.drive(distance_error, bearing)

    def pivot(self, now, bearing):
        """Return calibrated pivot pulses for optical bearing."""
        direction = 1 if bearing > 0 else -1
        if self.pivot_started_at is not None:
            if (direction != self.pivot_direction
                    or now - self.pivot_started_at >= self.pivot_duration):
                self.pivot_started_at = None
                self.pivot_pause_until = now + PIVOT_PAUSE_SECONDS
                return 0.0, 0.0
        if now < self.pivot_pause_until:
            return 0.0, 0.0
        if self.pivot_started_at is None:
            # Keep each correction to one nominal 20 Hz control tick. The
            # step wrapper enforces settling and a new image before moving.
            self.pivot_duration = PIVOT_PULSE_SECONDS
            self.pivot_started_at = now
            self.pivot_direction = direction
        return ((RIGHT_TURN_LEFT, RIGHT_TURN_RIGHT) if direction > 0
                else (LEFT_TURN_LEFT, LEFT_TURN_RIGHT))

    def drive(self, distance_error, bearing):
        """Apply steering and deadband compensation to positive forward error."""
        # Firmware units are uncalibrated: these are wheel commands, not m/s.
        headroom = cfg.MAX_COMMAND - cfg.MIN_COMMAND
        forward = min(headroom, FORWARD_GAIN * distance_error)
        # Positive optical bearing = tag right: left wheel must go faster.
        steering_limit = forward * MAX_STEERING_FRACTION
        turn = max(-steering_limit, min(steering_limit, STEERING_GAIN * bearing))
        left, right = forward + turn, forward - turn
        scale = min(1.0, headroom / max(left, right))
        # Add deadband compensation after steering so neither wheel stalls.
        # Arrival/loss stops above always bypass this floor and send exact zero.
        return cfg.MIN_COMMAND + left * scale, cfg.MIN_COMMAND + right * scale


def square_goal(x, z, quaternion, standoff):
    """Return goal right/forward and final clockwise heading in robot coordinates.

    AprilTag PnP's +Z normal points out of the printed face toward the viewer.
    x/z already include the camera-to-robot-centre translation.
    """
    norm = math.sqrt(sum(v * v for v in quaternion))
    if not all(math.isfinite(v) for v in (x, z, *quaternion)) or norm < cfg.MIN_QUATERNION_NORM:
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


class SquareApproachPolicy(MotionPolicy):
    """Approach the normal-line waypoint, align to its normal, then verify at rest.

    Supports modest initial offsets while the tag remains visible. Does not
    reverse or navigate around obstacles to reach a goal behind the robot.
    """

    def __init__(self, target):
        super().__init__(target, cfg.APPROACH_TIMEOUT_SECONDS)
        self.goal = None
        self.phase = 'WAIT'
        self.settled_since = None

    def update_goal(self, x, z, quaternion):
        self.goal = square_goal(x, z, quaternion, self.target)

    def _step(self, now, bearing=None, fresh=False):
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
            now, error + cfg.FORWARD_ERROR_OFFSET_M, control_angle,
            force_align=self.phase == 'ORIENTATION')
        if self.phase == 'POSITION' and left > 0 and right > 0:
            # Reuse HW1's measured straight-line wheel ratio in this mode.
            left *= FORWARD_LEFT / FORWARD_RIGHT
            scale = min(1., cfg.MAX_COMMAND / max(left, right))
            left, right = left * scale, right * scale
            left = max(cfg.MIN_COMMAND, left)
            right = max(cfg.MIN_COMMAND, right)
        return left, right


class AlignTagPolicy(MotionPolicy):
    """Search left, then center the target; never drive forward in this stage."""

    def __init__(self, target):
        """Keep the shared motion settings and initialize turn-only timers."""
        super().__init__(target, cfg.ALIGNMENT_TIMEOUT_SECONDS)
        self.search_pulse = None
        self.search_pause_until = 0.0
        self.centered_since = None

    def _step(self, now, bearing=None, fresh=False):
        """Return one pair of wheel commands from the latest target observation."""
        if not self.ready(now, fresh, allow_search=True):
            self.centered_since = None
            return 0.0, 0.0
        if not fresh:
            self.centered_since = None
            return self._search_left(now)

        if bearing is None or not math.isfinite(bearing):
            return self.stop('Invalid alignment bearing')
        if abs(bearing) <= CENTER_TOLERANCE_RADIANS:
            return self._settle_centered(now)
        return self._turn_toward_tag(now, bearing)

    def _search_left(self, now):
        """Alternate left pivots and full pauses measured from actual stop time."""
        if self.search_pulse is not None:
            if now - self.search_pulse >= SEARCH_PULSE_SECONDS:
                self.search_pulse = None
                self.search_pause_until = now + SEARCH_PAUSE_SECONDS
                return 0.0, 0.0
            return LEFT_TURN_LEFT, LEFT_TURN_RIGHT
        if now < self.search_pause_until:
            return 0.0, 0.0
        self.search_pulse = now
        return LEFT_TURN_LEFT, LEFT_TURN_RIGHT

    def _settle_centered(self, now):
        """Require a stable centered observation before marking this step done."""
        self.pivot_started_at = None
        if self.centered_since is None:
            self.centered_since = now
        if now - self.centered_since >= CENTER_SETTLE_SECONDS:
            self.stop(ALIGN_COMPLETE, Outcome.SUCCEEDED)
        return 0.0, 0.0

    def _turn_toward_tag(self, now, bearing):
        """Reuse the proven calibrated pulse controller for small corrections."""
        self.centered_since = None
        self.aligning = True
        steering = max(-MAX_STEERING_RADIANS, min(MAX_STEERING_RADIANS, bearing))
        return self.motion(now, 0.0, steering, force_align=True)
