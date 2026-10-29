"""Bounded wheel commands for a short, forward-only tag approach."""

import math


class ApproachPolicy:
    """Latch a stop on arrival, tracking failure, or the test time limit."""

    def __init__(self, target_distance=1.0, max_command=0.08, max_duration=20.0,
                 min_command=0.0):
        if not all(math.isfinite(v) for v in
                   (target_distance, max_command, max_duration, min_command)):
            raise ValueError('Approach parameters must be finite')
        if (target_distance <= 0 or not 0 <= min_command < max_command <= 0.3
                or max_duration <= 0):
            raise ValueError(
                'Require target > 0, 0 <= min_command < max_command <= 0.3, duration > 0')
        self.target = target_distance
        self.max_command = max_command
        self.max_duration = max_duration
        self.min_command = min_command
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
