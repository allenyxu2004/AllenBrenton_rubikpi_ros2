"""Differential-drive calibration and dead-reckoned pose estimate"""

from dataclasses import dataclass
import math


# Motor commands are firmware units, not physical wheel speeds.
FORWARD_LEFT = 0.25
FORWARD_RIGHT = 0.2
FORWARD_SPEED = 23.5 * 0.0254 / 2  # m/s: 23.5 inches in 2 seconds
LEFT_TURN_LEFT = -0.45
LEFT_TURN_RIGHT = 0.3
LEFT_TURN_RATE = math.pi / 2 / 0.989  # rad/s magnitude: 90 degrees
RIGHT_TURN_LEFT = 0.45
RIGHT_TURN_RIGHT = -0.3
RIGHT_TURN_RATE = math.pi / 2 / 0.859  # rad/s magnitude: 90 degrees


def normalize_angle(angle):
    """Return the shortest signed angle in [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class EstimatedPose:
    """Pose integrated from commanded motion, never a measured pose."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def integrate(self, speed, turn_rate, elapsed):
        """Integrate the differential-drive model for straight/pivot motion."""
        self.x += speed * math.cos(self.theta) * elapsed
        self.y += speed * math.sin(self.theta) * elapsed
        self.theta = normalize_angle(self.theta + turn_rate * elapsed)
