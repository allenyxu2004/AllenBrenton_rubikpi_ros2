# Homework 3, Question 6: EKF-SLAM algorithm

This answer describes the implementation in `robot_control/robot_control/slam/ekf.py`,
with observations supplied by `slam_node.py` and `slam/observations.py`.
The noise values below are heuristic starting settings, not experimentally fitted
estimates.

All equations use plain text. `A^T` means transpose, `inv(A)` means matrix
inverse, and adjacent matrices are multiplied. `P` denotes the state covariance
called Sigma in the assignment. Superscripts `-` and `+` denote prediction and
correction, respectively.

## i) State and measurement vectors

We implement planar EKF-SLAM with the joint state

```text
x = [x_r, y_r, theta, m_1x, m_1y, ..., m_Mx, m_My]^T
```

Here `(x_r, y_r, theta)` is the robot pose, and `(m_jx, m_jy)` is landmark
`j`'s position in the SLAM map frame. Lengths are in metres and angles are in
radians, with counterclockwise rotation positive. The number of landmarks `M`
starts at zero and grows as previously unseen AprilTag IDs are observed. The
state has `N = 3 + 2M` entries.

Each landmark observation is processed individually:

```text
z_j = [z_f, z_l]^T
```

`z_f` and `z_l` are the tag center's forward and left coordinates relative to
the robot center. We convert the AprilTag camera translation into these
coordinates using the camera mounting offset and yaw. For our level,
forward-facing camera with zero mounting yaw and lateral offset:

```text
z_f = camera_z + 0.0889 m
z_l = -camera_x
```

The optical camera axes have positive z forward and positive x right. The
vertical coordinate is omitted because the SLAM model is two-dimensional.

## ii) System, control, and measurement matrices

### Motion model and its Jacobians

The control input is

```text
u = [v, omega]^T
```

We use calibrated linear and angular speeds corresponding to the motor
commands. Prediction uses commanded motion rather than wheel odometry.

For timestep `dt`, define `a = theta + omega*dt/2`. Midpoint integration gives

```text
x_r_next   = x_r + v*cos(a)*dt
y_r_next   = y_r + v*sin(a)*dt
theta_next = theta + omega*dt
m_j_next   = m_j                 (all landmarks are stationary)
```

This nonlinear motion model is `x_next = f(x, u)`. We obtain the system matrix
by differentiating with respect to the state:

```text
F = df/dx = [ F_r     0   ]
            [  0    I_2M ]

      [ 1  0  -v*sin(a)*dt ]
F_r = [ 0  1   v*cos(a)*dt ]
      [ 0  0        1      ]
```

The control/noise Jacobian is the derivative with respect to `[v, omega]`:

```text
G = df/du = [ G_r ]
            [  0  ]

      [ cos(a)*dt   -v*sin(a)*dt^2/2 ]
G_r = [ sin(a)*dt    v*cos(a)*dt^2/2 ]
      [     0               dt      ]
```

`F` is N by N, and the full-state `G` is N by 2. The implementation stores
only the 3 by 2 robot block of `G`, since its landmark rows are zero. `G` is
a local Jacobian; mean prediction uses the nonlinear function `f`.

The EKF prediction equations are

```text
x^- = f(x^+, u)
P^- = F P^+ F^T + G Q G^T
```

### Measurement model and its Jacobian

Define the rotation from robot coordinates into map coordinates:

```text
           [ cos(theta)  -sin(theta) ]
C(theta) = [ sin(theta)   cos(theta) ]
```

For landmark `j`, the expected robot-relative observation is

```text
p_r    = [x_r, y_r]^T
h_j(x) = C(theta)^T (m_j - p_r) = [h_f, h_l]^T
```

We obtain `H_j = dh_j/dx` by differentiation. Let `c = cos(theta)` and
`s = sin(theta)`. Its nonzero blocks are

```text
            robot columns       landmark j columns
            x_r  y_r  theta       m_jx  m_jy
H_j = [     -c   -s    h_l   ...    c     s    ... ]
      [      s   -c   -h_f   ...   -s     c    ... ]
```

All other landmark columns are zero. `H_j` has 2 rows and N columns.

For an accepted observation, we compute

```text
innovation = z_j - h_j(x^-)
S          = H_j P^- H_j^T + R
K          = P^- H_j^T inv(S)
x^+        = x^- + K innovation
```

We use the Joseph form for the covariance correction:

```text
A   = I - K H_j
P^+ = A P^- A^T + K R K^T
```

This form is numerically robust. Heading is normalized after prediction and
correction.

## iii) Initial state and covariance

We initialize

```text
x_0|0 = [0, 0, 0]^T
P_0|0 = zeros(3, 3)
```

There are initially no landmark entries. This defines the starting robot
position and heading as the origin and orientation of the local SLAM frame.
It does not assume the robot's starting pose in the ground-truth world frame
is known. Zero initial covariance fixes this coordinate-frame freedom;
uncertainty subsequently accumulates through motion prediction.

For evaluation, an independently measured starting world pose provides the
rigid transformation from the local map to the ground-truth frame:

```text
m_world = p_start_world + C(theta_start_world) m_map
```

This transformation is used for comparison, not as prior landmark information
in the filter.

## iv) System and measurement noise

We model motion uncertainty through the control-noise covariance

```text
Q = diag(sigma_v^2, sigma_omega^2) / dt

sigma_v     = 0.08 m/sqrt(s)
sigma_omega = 0.20 rad/sqrt(s)
```

These parameters are continuous noise amplitudes. Dividing by `dt` makes the
leading motion-covariance growth proportional to elapsed time. Noise enters
the state covariance as `G Q G^T`. The implementation adds this noise while
commanded to move and adds none while stopped.

For each observation, measurement covariance is

```text
d_j     = sqrt(z_f^2 + z_l^2)
sigma_j = 0.04 m + 0.02*d_j
R_j     = sigma_j^2 I_2
```

This assumes independent, equally noisy forward and lateral coordinates, with
standard deviation increasing with distance.

These values are heuristic starting settings rather than estimates obtained
from repeated calibration trials. Motion noise represents effects such as
slip and differences between commanded and actual movement. Measurement noise
approximates AprilTag position uncertainty. Observations are accepted only
after stopping and settling to reduce motion-related errors.

To estimate these parameters experimentally, we would repeat fixed motion
commands and measure displacement and heading variation, then record
stationary tag measurements at several distances to estimate measurement
covariance. This describes a calibration procedure, not a completed experiment.

## v) Handling landmark visibility changes

### First detection of a new landmark

We associate observations using AprilTag IDs. For an unseen ID, we initialize
the landmark with the inverse measurement model:

```text
d     = C(theta) z
m_new = p_r + d
```

We append its two position coordinates to the state. The filter therefore
discovers the landmark count instead of assuming eight landmarks in advance.

The initialization Jacobians are

```text
      [ 1  0  -d_y ]
J_r = [ 0  1   d_x ]

J_z = C(theta)
```

The new landmark inherits robot uncertainty and correlations with the existing
state:

```text
B = P[:, robot] J_r^T
D = J_r P_rr J_r^T + C R C^T

        [ x_old ]            [ P_old   B ]
x_aug = [ m_new ]    P_aug = [  B^T    D ]
```

Here `P_rr` is the robot's 3 by 3 covariance block and `P[:, robot]` contains
the robot's covariance links with the entire existing state. We do not reuse
the initialization observation as a separate EKF correction.

### Landmark outside the field of view

We retain the landmark and its covariance in the state. Its position remains
stationary in prediction, and no measurement update is performed for that
landmark. Observations of other landmarks can still affect its estimate
through cross-covariances. Disappearance does not create a zero-valued
measurement or remove the landmark.

### Previously detected landmark reappearing

We retrieve the existing landmark by ID and perform an EKF correction. Before
correction, we apply an innovation gate:

```text
distance_squared = innovation^T inv(S) innovation
reject if distance_squared > 9.21
```

This is the 99% chi-square threshold for a two-dimensional innovation under
the assumed Gaussian model. An accepted reobservation can reduce accumulated
robot and map uncertainty through their correlations.
