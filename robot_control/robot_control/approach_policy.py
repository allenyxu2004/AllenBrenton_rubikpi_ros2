"""Bounded wheel commands for a short tag approach with in-place alignment."""

import math
from enum import Enum

from robot_control.control_config import POLICY_DEFAULTS

from robot_control.kinematic_control import (
    LEFT_TURN_LEFT, LEFT_TURN_RIGHT, LEFT_TURN_RATE,
    RIGHT_TURN_LEFT, RIGHT_TURN_RIGHT, RIGHT_TURN_RATE,
)


MAX_COMMAND = 0.3
MAX_STOP_MARGIN_M = 0.3
MAX_BEARING_RADIANS = math.radians(20)
ALIGN_ENTER_RADIANS = math.radians(8)
ALIGN_EXIT_RADIANS = math.radians(3)
PIVOT_PAUSE_SECONDS = 0.2
MIN_PIVOT_SECONDS = 0.05
MAX_PIVOT_SECONDS = 0.12
PIVOT_CORRECTION_FRACTION = 0.5
PIVOT_RESIDUAL_RADIANS = math.radians(2)
FORWARD_GAIN = 0.25
STEERING_GAIN = 0.12
MAX_STEERING_FRACTION = 0.5


class Outcome(Enum):
    RUNNING = 'running'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'


class ApproachPolicy:
    """Latch a stop on arrival, tracking failure, or the test time limit."""

    def __init__(self,
                 target_distance=POLICY_DEFAULTS['target_distance'],
                 max_command=POLICY_DEFAULTS['max_command'],
                 max_duration=POLICY_DEFAULTS['max_duration'],
                 min_command=POLICY_DEFAULTS['min_command'],
                 drive_seconds=POLICY_DEFAULTS['drive_seconds'],
                 pause_seconds=POLICY_DEFAULTS['pause_seconds'],
                 pivot_command=POLICY_DEFAULTS['pivot_command'],
                 use_hw1_turns=POLICY_DEFAULTS['use_hw1_turns'],
                 stop_margin=POLICY_DEFAULTS['stop_margin'],
                 loss_grace_seconds=POLICY_DEFAULTS['loss_grace_seconds']):
        if not all(math.isfinite(v) for v in
                   (target_distance, max_command, max_duration, min_command,
                    drive_seconds, pause_seconds, pivot_command, stop_margin,
                    loss_grace_seconds)):
            raise ValueError('Approach parameters must be finite')
        if (target_distance <= 0 or not 0 <= min_command < max_command <= MAX_COMMAND
                or max_duration <= 0 or drive_seconds <= 0 or pause_seconds < 0
                or not 0 < pivot_command <= MAX_COMMAND
                or not 0 <= stop_margin <= MAX_STOP_MARGIN_M
                or loss_grace_seconds < 0):
            raise ValueError(
                'Require target/duration/drive_seconds > 0, '
                'pause_seconds/loss_grace_seconds >= 0, '
                '0 <= min_command < max_command <= 0.3, 0 < pivot_command <= 0.3, '
                f'0 <= stop_margin <= {MAX_STOP_MARGIN_M}')
        self.target = target_distance
        self.stop_margin = stop_margin
        self.max_command = max_command
        self.max_duration = max_duration
        self.min_command = min_command
        self.use_hw1_turns = use_hw1_turns
        self.loss_grace_seconds = loss_grace_seconds
        self.lost_since = None
        self.pivot_started_at = None
        self.pivot_duration = 0.0
        self.pivot_pause_until = 0.0
        self.pivot_direction = None
        self.pivot_command = pivot_command
        self.drive_seconds = drive_seconds
        self.pause_seconds = pause_seconds
        self.segment_started_at = None
        self.pause_started_at = None
        self.paused = False
        self.aligning = False
        self.started_at = None
        self.tracking = False
        self.reason = None
        self.outcome = Outcome.RUNNING

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
                if now - self.lost_since >= self.loss_grace_seconds:
                    self.stop('Tag lost or stale; restart controller to retry')
            return allow_search and not self.tracking and self.reason is None
        self.lost_since = None
        self.tracking = True
        return True

    def step(self, now, distance=None, bearing=None, fresh=False):
        if not self.ready(now, fresh):
            return 0.0, 0.0
        if not all(v is not None and math.isfinite(v) for v in (distance, bearing)):
            return self.stop('Invalid tag pose')
        if distance <= self.target + self.stop_margin:
            return self.stop('Stopping threshold reached (target + coast margin)',
                             Outcome.SUCCEEDED)
        if abs(bearing) > MAX_BEARING_RADIANS:
            return self.stop('Tag more than 20 degrees off centre')
        return self.motion(now, distance - self.target, bearing)

    def interval_paused(self, now):
        """Measure full pauses from actual transitions, even after delayed ticks."""
        # Use actual transition times so a delayed callback cannot skip a pause.
        if self.segment_started_at is None:
            self.segment_started_at = now
        if self.paused:
            if now - self.pause_started_at < self.pause_seconds:
                return True
            self.paused = False
            self.segment_started_at = now
        elif (self.pause_seconds > 0
              and now - self.segment_started_at >= self.drive_seconds):
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
        """Return calibrated pulses or a continuous pivot for optical bearing."""
        if self.use_hw1_turns:
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
                rate = RIGHT_TURN_RATE if direction > 0 else LEFT_TURN_RATE
                # Half the estimated correction, capped at 0.12 s. The 20 Hz
                # timer quantizes pulses; remeasure after a 0.2 s stop.
                self.pivot_duration = max(MIN_PIVOT_SECONDS, min(
                    MAX_PIVOT_SECONDS, PIVOT_CORRECTION_FRACTION
                    * (abs(bearing) - PIVOT_RESIDUAL_RADIANS) / rate))
                self.pivot_started_at = now
                self.pivot_direction = direction
            return ((RIGHT_TURN_LEFT, RIGHT_TURN_RIGHT) if direction > 0
                    else (LEFT_TURN_LEFT, LEFT_TURN_RIGHT))
        turn = math.copysign(self.pivot_command, bearing)
        return turn, -turn

    def drive(self, distance_error, bearing):
        """Apply steering and deadband compensation to positive forward error."""
        # Firmware units are uncalibrated: these are wheel commands, not m/s.
        headroom = self.max_command - self.min_command
        forward = min(headroom, FORWARD_GAIN * distance_error)
        # Positive optical bearing = tag right: left wheel must go faster.
        steering_limit = forward * MAX_STEERING_FRACTION
        turn = max(-steering_limit, min(steering_limit, STEERING_GAIN * bearing))
        left, right = forward + turn, forward - turn
        scale = min(1.0, headroom / max(left, right))
        # Add deadband compensation after steering so neither wheel stalls.
        # Arrival/loss stops above always bypass this floor and send exact zero.
        return self.min_command + left * scale, self.min_command + right * scale
