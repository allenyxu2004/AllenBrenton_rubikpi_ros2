"""Turn-only route step using the same feedback timing as approach_tag."""
import math

from robot_control.approach_policy import ApproachPolicy
from robot_control.kinematic_control import LEFT_TURN_LEFT, LEFT_TURN_RIGHT

ALIGN_COMPLETE = 'Route alignment complete'
SEARCH_PULSE_SECONDS = 0.1
SEARCH_PAUSE_SECONDS = 0.2
CENTER_TOLERANCE_RADIANS = math.radians(3)
CENTER_SETTLE_SECONDS = 0.5
MAX_STEERING_RADIANS = math.radians(19)


class AlignTagPolicy(ApproachPolicy):
    """Search left, then center the target; never drive forward in this stage."""

    def __init__(self, *args, **kwargs):
        """Keep the shared motion settings and initialize turn-only timers."""
        super().__init__(*args, **kwargs)
        self.search_started = None
        self.search_pulse = None
        self.search_pause_until = 0.0
        self.centered_since = None

    def step(self, now, distance=None, bearing=None, fresh=False):
        """Return one pair of wheel commands from the latest target observation."""
        if self.reason:
            return 0.0, 0.0
        if self.search_started is None:
            self.search_started = now
        if now - self.search_started >= self.max_duration:
            return self.stop('Alignment/search time limit reached')

        if not fresh:
            self.centered_since = None
            if self.tracking:
                # Once acquired, loss means stop/recover, not blind searching.
                return super().step(now, distance, bearing, False)
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
        self.tracking = True
        self.lost_since = None
        self.pivot_started_at = None
        if self.centered_since is None:
            self.centered_since = now
        if now - self.centered_since >= CENTER_SETTLE_SECONDS:
            self.stop(ALIGN_COMPLETE)
        return 0.0, 0.0

    def _turn_toward_tag(self, now, bearing):
        """Reuse the proven calibrated pulse controller for small corrections."""
        self.centered_since = None
        self.aligning = True
        steering = max(-MAX_STEERING_RADIANS, min(MAX_STEERING_RADIANS, bearing))
        # A synthetic range avoids the base policy's distance-arrival stop.
        # Forcing aligning=True above ensures this call only produces pivots.
        steering_only_distance = self.target + self.stop_margin + 1.0
        return super().step(now, steering_only_distance, steering, True)
