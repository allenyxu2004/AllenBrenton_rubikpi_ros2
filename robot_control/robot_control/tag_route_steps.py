"""Turn-only route step using the same feedback timing as approach_tag."""
import math
from robot_control.approach_policy import ApproachPolicy
from robot_control.kinematic_control import LEFT_TURN_LEFT, LEFT_TURN_RIGHT

# Third field overrides the normal approach distance for that stage only.
STAGES = [('approach', 9, None), ('align', 8, None), ('approach', 8, None),
          ('align', 7, None), ('approach', 7, None), ('align', 5, None),
          ('approach', 5, None), ('align', 9, None), ('approach', 9, 1.5)]
ALIGN_COMPLETE = 'Route alignment complete'


class AlignTagPolicy(ApproachPolicy):
    """Search left, then center without any forward translation."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.search_started = None
        self.search_pulse = None
        self.search_pause_until = 0.
        self.centered_since = None

    def step(self, now, distance=None, bearing=None, fresh=False):
        if self.reason:
            return 0., 0.
        if self.search_started is None:
            self.search_started = now
        if now-self.search_started >= self.max_duration:
            return self.stop('Alignment/search time limit reached')
        if not fresh:
            self.centered_since = None
            if self.tracking:
                return super().step(now, distance, bearing, False)
            if self.search_pulse is not None:
                if now-self.search_pulse >= 0.1:
                    self.search_pulse = None
                    self.search_pause_until = now+0.2
                    return 0., 0.
                return LEFT_TURN_LEFT, LEFT_TURN_RIGHT
            if now < self.search_pause_until:
                return 0., 0.
            self.search_pulse = now
            return LEFT_TURN_LEFT, LEFT_TURN_RIGHT
        if bearing is None or not math.isfinite(bearing):
            return self.stop('Invalid alignment bearing')
        if abs(bearing) <= math.radians(3):
            self.tracking = True
            self.lost_since = None
            self.pivot_started_at = None
            self.centered_since = now if self.centered_since is None else self.centered_since
            if now-self.centered_since >= 0.5:
                self.stop(ALIGN_COMPLETE)
            return 0., 0.
        self.centered_since = None
        self.aligning = True
        # Match approach_tag's calibrated pulses, pause and loss handling.
        # Clamp only the steering input during acquisition at the image edges.
        steering = max(-math.radians(19), min(math.radians(19), bearing))
        return super().step(now, self.target+self.stop_margin+1., steering, True)
