"""Use HW3's motion and measurement models with a surveyed landmark map."""

import numpy as np

from robot_control.slam.ekf import EkfSlam


class FixedMapLocalizer(EkfSlam):
    """Anchor the surveyed start, then estimate motion against fixed landmarks."""

    def __init__(self, course, translation_noise, rotation_noise, innovation_gate):
        super().__init__(translation_noise, rotation_noise, gate=innovation_gate)
        self.x = np.array([*course.start, course.heading,
                           *(coordinate for point in course.landmarks.values()
                             for coordinate in point)])
        self.indices = {tag: 3 + 2 * i for i, tag in enumerate(course.landmarks)}
        self.start_xy = np.array(course.start, dtype=float)
        self.position_anchored = True
        # Start pose and surveyed tags are exact initial anchors, as in HW3.
        # Zero covariance makes stationary observations diagnostic only: their
        # Kalman gain is zero, so they cannot relocate the placed robot.
        self.P = np.zeros((len(self.x), len(self.x)))

    def predict(self, speed, turn_rate, dt):
        if dt <= 0:
            return
        if speed != 0:
            self.position_anchored = False
        super().predict(speed, turn_rate, dt)
        if self.position_anchored:
            # Alignment pivots change heading, not the known starting center.
            # Do not let HW3's translational process noise during a pivot give
            # biased camera observations permission to shift that center.
            self.x[:2] = self.start_xy
            self.P[:2, :] = 0.0
            self.P[:, :2] = 0.0

    def observe(self, tag_id, forward, left, noise=None):
        if tag_id not in self.indices:
            return 'unknown'
        return super().observe(tag_id, forward, left, noise)
