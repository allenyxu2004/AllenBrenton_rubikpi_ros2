"""Bounded wheel commands for a short tag approach with in-place alignment."""

import math


class ApproachPolicy:
    """Latch a stop on arrival, tracking failure, or the test time limit."""

    def __init__(self, target_distance=1.0, max_command=0.08, max_duration=20.0,
                 min_command=0.0, drive_seconds=3.5, pause_seconds=0.3,
                 pivot_command=0.2):
        if not all(math.isfinite(v) for v in
                   (target_distance, max_command, max_duration, min_command,
                    drive_seconds, pause_seconds, pivot_command)):
            raise ValueError('Approach parameters must be finite')
        if (target_distance <= 0 or not 0 <= min_command < max_command <= 0.3
                or max_duration <= 0 or drive_seconds <= 0 or pause_seconds < 0
                or not 0 < pivot_command <= 0.3):
            raise ValueError(
                'Require target/duration/drive_seconds > 0, pause_seconds >= 0, '
                '0 <= min_command < max_command <= 0.3, 0 < pivot_command <= 0.3')
        self.target = target_distance
        self.max_command = max_command
        self.max_duration = max_duration
        self.min_command = min_command
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

    def stop(self, reason):
        if self.reason is None:
            self.reason = reason
        return 0.0, 0.0

    def step(self, now, distance=None, bearing=None, fresh=False):
        if self.started_at is None:
            self.started_at = now
        if self.reason is not None:
            return 0.0, 0.0
        if now - self.started_at >= self.max_duration:
            return self.stop('Test time limit reached')
        if not fresh:
            if self.tracking:
                return self.stop('Tag lost or stale; restart controller to retry')
            return 0.0, 0.0
        if not all(v is not None and math.isfinite(v) for v in (distance, bearing)):
            return self.stop('Invalid tag pose')
        self.tracking = True
        if distance <= self.target:
            return self.stop('Target distance reached')
        if abs(bearing) > math.radians(20):
            return self.stop('Tag more than 20 degrees off centre')
        # Use actual transition times so a delayed callback cannot skip a pause.
        if self.segment_started_at is None:
            self.segment_started_at = now
        if self.paused:
            if now - self.pause_started_at < self.pause_seconds:
                return 0.0, 0.0
            self.paused = False
            self.segment_started_at = now
        elif (self.pause_seconds > 0
              and now - self.segment_started_at >= self.drive_seconds):
            self.paused = True
            self.pause_started_at = now
            return 0.0, 0.0
        # Align before translating when lateral error is substantial. Hysteresis
        # prevents repeated mode switching at one bearing threshold.
        if abs(bearing) >= math.radians(8):
            self.aligning = True
        elif abs(bearing) <= math.radians(3):
            self.aligning = False
        if self.aligning:
            # Optical bearing positive = tag right = clockwise pivot.
            turn = math.copysign(self.pivot_command, bearing)
            return turn, -turn
        # Firmware units are uncalibrated: these are wheel commands, not m/s.
        headroom = self.max_command - self.min_command
        forward = min(headroom, 0.25 * (distance - self.target))
        # Positive optical bearing = tag right: left wheel must go faster.
        turn = max(-forward * 0.5, min(forward * 0.5, 0.12 * bearing))
        left, right = forward + turn, forward - turn
        scale = min(1.0, headroom / max(left, right))
        # Add deadband compensation after steering so neither wheel stalls.
        # Arrival/loss stops above always bypass this floor and send exact zero.
        return self.min_command + left * scale, self.min_command + right * scale
