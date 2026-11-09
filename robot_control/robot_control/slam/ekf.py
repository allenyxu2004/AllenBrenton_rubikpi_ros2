"""Planar EKF-SLAM, independent of ROS, cameras, and motor firmware.

State: [robot_x, robot_y, heading, tag_a_x, tag_a_y, tag_b_x, ...].
Measurements: tag center [forward, left] in the robot's ground plane.
All lengths are metres; all angles are radians, counterclockwise positive.

Notation used in the equation comments:
    x_hat = estimated joint state (self.x); P = covariance (self.P).
    Superscript '-' means prediction, before the current observation;
    superscript '+' means correction, after that observation.
    M^T = matrix transpose (M.T); @ = matrix multiplication; I = identity.
    N = 3 + 2 * number_of_landmarks, so x_hat is N x 1 and P is N x N.
    NumPy stores state vectors as 1-D arrays, but the math treats them as columns.
"""

import math

import numpy as np

from robot_control.kinematic_control import normalize_angle


class EkfSlam:
    """Grow a joint Gaussian as previously unseen landmark IDs arrive."""

    def __init__(self, translation_noise=0.08, rotation_noise=0.20,
                 measurement_noise=0.04, gate=9.21):
        # Fix the gauge: startup defines (0, 0, 0). This is not a known world pose.
        #
        # With two landmarks, for example:
        #
        #           [ x_r   ]           [ P_rr   P_r1   P_r2 ]
        #           [ y_r   ]           [                   ]
        # x_hat  =  [ theta ]      P  =  [ P_1r   P_11   P_12 ]
        #           [ m_1x  ]           [                   ]
        #           [ m_1y  ]           [ P_2r   P_21   P_22 ]
        #           [ m_2x  ]
        #           [ m_2y  ]
        #
        # P_rr is 3x3; P_r1 is 3x2; P_11 and P_12 are 2x2.
        # Diagonal entries are variances. Off-diagonal entries encode how
        # estimation errors move together, including robot/landmark errors.
        # Initially there are no landmark rows or columns at all.
        self.x = np.zeros(3)
        self.P = np.zeros((3, 3))
        self.indices = {}
        self.translation_noise = translation_noise
        self.rotation_noise = rotation_noise
        self.measurement_noise = measurement_noise
        self.gate = gate  # chi-square threshold for a 2D innovation (99%).

    def predict(self, speed, turn_rate, dt):
        """Propagate commanded unicycle motion and all robot/map correlations.

        Midpoint integration is accurate for short pulses. F differentiates
        motion with respect to state; G differentiates it with respect to [v,w].
        Q is a continuous control-noise density, so G Q G.T scales with dt.
        """
        if dt <= 0:
            return
        # Motion model: u_k = [v, omega]^T, a = theta + omega * dt/2.
        #
        #                   [ x_r + v cos(a) dt ]
        # x_hat^- = f(x,u) = [ y_r + v sin(a) dt ]
        #                   [ theta + omega dt ]
        #                   [ all landmarks unchanged ]
        #
        # The midpoint angle 'a' approximates turning during the time step.
        theta = self.x[2] + turn_rate * dt / 2
        c, s = math.cos(theta), math.sin(theta)
        # F = partial f / partial x, evaluated at the current estimate:
        #
        #       [ 1  0  -v sin(a) dt | 0 ... 0 ]
        #       [ 0  1   v cos(a) dt | 0 ... 0 ]
        # F  =  [ 0  0       1      | 0 ... 0 ]    (N x N)
        #       [ ------------------+--------- ]
        #       [       0           | I_map   ]
        #
        # A heading error changes the predicted x/y displacement. That is
        # why those two entries are nonzero even for straight driving.
        F = np.eye(len(self.x))
        F[0, 2], F[1, 2] = -speed * s * dt, speed * c * dt
        # G_r = partial f_robot / partial [v, omega]:
        #
        #       [ cos(a) dt    -v sin(a) dt^2/2 ]
        # G_r = [ sin(a) dt     v cos(a) dt^2/2 ]    (3 x 2)
        #       [     0                dt      ]
        #
        # Full-state G would be [G_r; 0_map]. We store only G_r because
        # command noise directly affects the robot, not stationary landmarks.
        G = np.array([[c * dt, -speed * s * dt**2 / 2],
                      [s * dt, speed * c * dt**2 / 2], [0, dt]])
        # Model uncertainty only while commanded to move. Stationary slip and
        # camera/extrinsic errors are not estimated by this simple model.
        # Q = (1/dt) [ sigma_v^2      0       ]
        #            [     0      sigma_w^2  ]    (2 x 2)
        # sigma_v and sigma_w are continuous noise amplitudes; dividing by
        # dt makes G_r Q G_r^T grow roughly with elapsed time, not dt^2.
        Q = np.diag([self.translation_noise**2, self.rotation_noise**2]) / dt
        if speed == 0 and turn_rate == 0:
            Q *= 0
        self.x[:3] += [speed * c * dt, speed * s * dt, turn_rate * dt]
        self.x[2] = normalize_angle(self.x[2])
        # P^- = F P^+ F^T + G Q G^T.
        # The first product propagates ALL cross-covariances. The noise term
        # occupies only the top-left 3x3 block because G's map rows are zero.
        self.P = F @ self.P @ F.T
        self.P[:3, :3] += G @ Q @ G.T
        # P should equal P^T; average them to remove floating-point asymmetry.
        self.P = (self.P + self.P.T) / 2

    def observe(self, tag_id, forward, left, noise=None):
        """Initialize a landmark or correct a known one; reject outliers.

        Noise is an isotropic standard deviation in metres, not a variance.
        New landmarks inherit robot uncertainty AND its cross-correlations.
        """
        z = np.array([forward, left], dtype=float)
        sigma = self.measurement_noise if noise is None else noise
        if not np.all(np.isfinite(z)) or not math.isfinite(sigma) or sigma <= 0:
            return 'invalid'
        # z = [forward, left]^T,  R = [ sigma^2     0    ] (2 x 2).
        #                            [    0     sigma^2 ]
        # R models independent, equally noisy horizontal camera coordinates.
        R = np.eye(2) * sigma**2
        c, s = math.cos(self.x[2]), math.sin(self.x[2])
        # C(theta) = [ cos(theta)  -sin(theta) ] maps robot axes into map axes.
        #            [ sin(theta)   cos(theta) ]
        # C^T is its inverse. Use C for rotation, R for measurement covariance.
        rotation = np.array([[c, -s], [s, c]])
        if tag_id not in self.indices:
            return self.initialize_landmark(tag_id, z, R, rotation)
        return self.update_landmark(tag_id, z, R, rotation)

    def initialize_landmark(self, tag_id, measurement, measurement_covariance,
                            robot_to_map):
        """Add a new landmark and its covariance links; called by observe()."""
        # Inverse measurement model for a NEW landmark:
        #
        # d = C z,      m_new = [x_r, y_r]^T + d.
        #
        # J_x = partial m_new / partial [x_r, y_r, theta]
        #     = [ 1  0  -d_y ]     J_z = partial m_new / partial z = C.
        #       [ 0  1   d_x ]
        #
        # Robot heading uncertainty moves the landmark sideways around
        # the robot, so its position covariance must include that effect.
        landmark_offset = robot_to_map @ measurement
        initialization_jacobian = np.array([
            [1, 0, -landmark_offset[1]],
            [0, 1, landmark_offset[0]],
        ])
        # B = Cov(old_state, m_new) = P[:, robot] J_x^T     (N x 2)
        # D = Cov(m_new, m_new) = J_x P_rr J_x^T + C R C^T (2 x 2)
        #
        # x_aug = [ x_old ]       P_aug = [ P_old   B ]
        #         [ m_new ]               [  B^T    D ]
        #
        # B connects the new landmark to the existing robot AND map.
        # Omitting B would incorrectly treat those estimates as independent.
        cross_covariance = self.P[:, :3] @ initialization_jacobian.T
        propagated_robot_covariance = (
            initialization_jacobian @ self.P[:3, :3] @ initialization_jacobian.T)
        rotated_measurement_covariance = (
            robot_to_map @ measurement_covariance @ robot_to_map.T)
        landmark_covariance = propagated_robot_covariance + rotated_measurement_covariance
        self.indices[tag_id] = len(self.x)
        self.x = np.concatenate([self.x, self.x[:2] + landmark_offset])
        self.P = np.block([[self.P, cross_covariance],
                          [cross_covariance.T, landmark_covariance]])
        # Do not correct again using the observation that initialized it.
        return 'new'

    def update_landmark(self, tag_id, measurement, measurement_covariance,
                        robot_to_map):
        """Correct a known landmark and correlated state; called by observe()."""
        landmark_index = self.indices[tag_id]
        # Forward measurement model for a KNOWN landmark j:
        #
        # h_j(x) = C^T (m_j - p_r) = [ h_f ]   (expected forward/left).
        #                            [ h_l ]
        landmark_position = self.x[landmark_index:landmark_index + 2]
        robot_to_landmark = landmark_position - self.x[:2]
        predicted_measurement = robot_to_map.T @ robot_to_landmark
        # H = partial h_j / partial x. Example when observing landmark 1:
        #
        #       robot x,y,theta     landmark 1      landmark 2
        #     [ -c  -s   h_l    |     c   s     |     0   0    ]
        # H = [  s  -c  -h_f    |    -s   c     |     0   0    ]
        #
        # H has 2 rows and N columns. Only this landmark's columns are
        # directly involved, but covariance links let the whole map update.
        H = np.zeros((2, len(self.x)))
        H[:, :2] = -robot_to_map.T
        H[:, 2] = [predicted_measurement[1], -predicted_measurement[0]]
        H[:, landmark_index:landmark_index + 2] = robot_to_map.T
        # Innovation nu = z - h_j(x_hat^-): measured minus predicted location.
        # S = H P^- H^T + R is the 2x2 covariance of that discrepancy.
        innovation = measurement - predicted_measurement
        innovation_covariance = H @ self.P @ H.T + measurement_covariance
        # d^2 = nu^T S^-1 nu: discrepancy measured in uncertainty units.
        # Reject an implausible observation before it can distort the map.
        # solve(S, nu) computes S^-1 nu without constructing an inverse.
        squared_mahalanobis = innovation @ np.linalg.solve(innovation_covariance, innovation)
        if squared_mahalanobis > self.gate:
            return 'outlier'
        # Kalman gain K = P^- H^T S^-1                         (N x 2)
        # Updated mean x_hat^+ = x_hat^- + K nu                (N x 1)
        #
        # The solve below uses symmetry of P and S:
        # solve(S, H P)^T = (S^-1 H P)^T = P H^T S^-1.
        # Larger uncertainty relative to camera noise generally gives the
        # observation more influence, including on correlated landmarks.
        kalman_gain = np.linalg.solve(innovation_covariance, H @ self.P).T
        state_correction = kalman_gain @ innovation
        self.x += state_correction
        self.x[2] = normalize_angle(self.x[2])
        # Joseph form maintains positive semidefiniteness under roundoff.
        # A = I - K H
        # P^+ = A P^- A^T + K R K^T.
        # This is algebraically equivalent to (I-KH)P^- in exact arithmetic,
        # but is numerically safer for covariance matrices.
        residual_projection = np.eye(len(self.x)) - kalman_gain @ H
        self.P = (residual_projection @ self.P @ residual_projection.T
                  + kalman_gain @ measurement_covariance @ kalman_gain.T)
        self.P = (self.P + self.P.T) / 2
        return 'updated'
