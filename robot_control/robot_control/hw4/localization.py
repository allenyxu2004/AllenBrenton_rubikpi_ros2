"""Use HW3's motion and measurement models with a surveyed landmark map."""

import numpy as np

from robot_control.slam.ekf import EkfSlam


class FixedMapLocalizer(EkfSlam):
    """The initial pose has uncertainty; surveyed landmarks are constants."""

    def __init__(self, course, translation_noise, rotation_noise,
                 initial_position_std, initial_heading_std, innovation_gate):
        super().__init__(translation_noise, rotation_noise, gate=innovation_gate)
        self.x = np.array([*course.start, course.heading,
                           *(coordinate for point in course.landmarks.values()
                             for coordinate in point)])
        self.indices = {tag: 3 + 2 * i for i, tag in enumerate(course.landmarks)}
        # Fixed tags have zero variance and zero cross-covariance. Their
        # Kalman gain remains zero under prediction and correction.
        self.P = np.zeros((len(self.x), len(self.x)))
        self.P[:3, :3] = np.diag([initial_position_std**2, initial_position_std**2,
                                  initial_heading_std**2])

    def observe(self, tag_id, forward, left, noise=None):
        if tag_id not in self.indices:
            return 'unknown'
        return super().observe(tag_id, forward, left, noise)
