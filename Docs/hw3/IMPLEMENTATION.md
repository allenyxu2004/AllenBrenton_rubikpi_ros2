# How the HW3 implementation fits together

Read the files in this order:

| File under `robot_control/robot_control/` | Responsibility |
|---|---|
| `slam/ekf.py` | State, covariance, motion prediction, landmark initialization, correction. No ROS. |
| `slam/observations.py` | Tag ID parsing and camera-to-robot planar coordinates. No ROS. |
| `slam/trajectory.py` | Polygon vertices and next-action selection from estimated pose. No ROS. |
| `slam_node.py` | ROS subscriptions/publishers, motor calibration parameters, pulse/settle timing, stop conditions, logging. |
| `slam/evaluate.py` | Offline plots, frame alignment, landmark error CSV, per-lap means. |

Run offline analysis on the laptop with NumPy and Matplotlib; ROS is not needed.
The robot only needs NumPy for the filter and writes JSONL logs for transfer.

`config/hw3_slam.yaml` lists every SLAM/controller parameter, including run
settings, physical calibration, and tuning. Python retains fallback defaults.
Optional launch arguments override only the values explicitly supplied.
`config/hw3_tags.yaml` contains detector settings with no landmark whitelist.
`launch/hw3_slam.launch.py` starts the complete stack, stationary by default.
Existing HW1/HW2 controllers are unchanged.

In `ekf.py`, `observe()` validates measurements and dispatches to
`initialize_landmark()` for new IDs or `update_landmark()` for known IDs.
Intermediate names such as `innovation_covariance`, `kalman_gain`, and
`state_correction` match the equation comments beside each calculation.

## State and measurements

The joint state is `x = [robot_x, robot_y, theta, lm1_x, lm1_y, ...]`.
`P` is its full covariance. `indices` maps tag IDs to landmark offsets in `x`.
It starts empty, with no predefined number of tags.

The startup robot pose and covariance are zero: this fixes the arbitrary map
coordinate system. It does not assert knowledge of the robot's room position.
The measured startup world pose is used only for offline evaluation.

Camera optical coordinates are right/down/forward. For a level forward-facing
camera, the measured tag center in robot coordinates is
`z = [camera_z + forward_offset, -camera_x + left_offset]`.
The adapter also handles camera mounting yaw. Vertical separation is discarded,
so this is a Cartesian ground-plane measurement rather than a 3D range.

## Prediction: what motion do commands suggest?

The control input is `u = [v, w]`, supplied by the calibrated straight/pivot
primitive. These are physical m/s and rad/s, not firmware wheel values.
For time `dt`, use `a = theta + w*dt/2`:

```text
robot_x += v*cos(a)*dt
robot_y += v*sin(a)*dt
theta   += w*dt
P        = F P F^T + G Q G^T
```

`F` is identity except `F[x,theta] = -v*sin(a)*dt` and
`F[y,theta] = v*cos(a)*dt`. Landmarks remain stationary. Their correlations
with the moving robot are propagated through the full matrix multiplication.
`G` has nonzero entries only in the robot block:

```text
G_robot = [[cos(a)*dt, -v*sin(a)*dt^2/2],
           [sin(a)*dt,  v*cos(a)*dt^2/2],
           [0,         dt]]
```

`Q = diag(translation_noise^2, rotation_noise^2)/dt` for moving intervals.
These parameters are continuous control noise amplitudes (m/sqrt(s),
rad/sqrt(s)), not direct wheel-command errors. This scaling makes accumulated
noise approximately independent of timer frequency. A stationary command adds
no process noise in this simplified model. There is no extra yaw or wheel
measurement from the IMU/encoders.

## Correction: what does the camera say?

For a known landmark `m`, predicted measurement is
`h(x) = Rot(theta)^T * (m - robot_position)`.
The Jacobian `H` includes derivatives with respect to robot position, heading,
and that landmark's position; other landmark columns are zero.

```text
innovation = z - h(x)
S = H P H^T + R
K = P H^T S^-1
x = x + K innovation
```

The implementation uses linear solves instead of explicit inverses and Joseph
form for the covariance update. The full covariance lets one tag observation
correct both the robot and correlated map entries. A squared Mahalanobis
innovation above `innovation_gate` is rejected. The default 9.21 corresponds to
a 99% 2D chi-square gate under the model assumptions.

`R = sigma^2 I`, where `sigma = measurement_noise_m + range_noise_fraction *
planar_distance`. This simple isotropic model is a starting approximation; real
tag depth error can be much larger than lateral error. Measure stationary tag
scatter at several ranges and tune it. Fit process noise from repeated motion
trials using actual pulse lengths. Systematic calibration errors are not fixed
by reducing covariance or simply increasing filter confidence.

## First sighting, disappearance, and return

A new ID is initialized as `m = robot_position + Rot(theta)*z`.
The state grows by two entries. If `Jx` and `Jz` are derivatives of this inverse
measurement model, its covariance is
`Jx P_robot Jx^T + Jz R Jz^T`, and its cross-covariance with the old state is
`P[:,robot] Jx^T`. Keeping those cross terms is essential. The initializing
observation is not immediately reused as another independent correction.

A landmark leaving the field of view stays in the state unchanged; its
correlations still propagate. Its return uses the existing ID and correction
step. Unique physical tags must have unique IDs. No orientation, wall geometry,
or known tag positions are used. False first detections can create false
landmarks: the detector uses zero corrected bits, but there is no multi-frame
confirmation or landmark deletion scheme in this implementation.

## Timing and control

The ROS node publishes at 20 Hz. It predicts using the previous commanded
motion, then chooses the next action from the current SLAM estimate. It never
opens the serial port itself. Firmware commands and predicted physical rates
come from the same chosen primitive.

No initial tag observation is required. Before discovering any landmarks, the
filter only predicts motion and grows uncertainty. There is no no-tag timeout;
the route-duration and pose-uncertainty limits still apply. A new tag initializes
a map entry; subsequent sightings provide drift corrections.

Motion is pulsed, followed by settling and an observation window. TF messages
are applied only while stopped, and only if their image timestamps are after
the settling deadline. Duplicate/stale frames are discarded. Thus the filter
does not need a delayed-state buffer for images captured during motion. This
assumes the robot is actually stationary by the deadline; serial lag, coast,
rolling shutter, and timestamp accuracy still need hardware verification.

The route policy does not access ground truth. It pivots until the estimated
bearing to the next vertex is small, then drives toward it until within position
tolerance. Returning landmarks can correct accumulated drift, including on later
laps, but consistency depends on visibility and model quality. Small maps and
reasonable initial drift are necessary for this EKF's local linearization.

Each lap closes by pivoting to startup heading zero, then observing while
stationary for `lap_observe_s`. The lap's arrival snapshot is recorded after this
pause so the per-lap error calculation includes those corrections. The lap then
completes without repositioning or realigning in response to corrections during
the pause. No tag is
required to appear during the pause; this restores the estimated viewing
direction but cannot guarantee the original tag is physically in view.

