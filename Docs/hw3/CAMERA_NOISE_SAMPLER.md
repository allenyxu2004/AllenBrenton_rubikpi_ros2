# Measure camera noise at one distance

The `camera_noise` script collects 200 distinct observations of one AprilTag,
calculates camera scatter, and saves the samples and results. It reads raw tag
transforms before any SLAM outlier rejection. It does not move the robot.

## 1. Build and start the camera/tag pipeline

On the robot, from your colcon workspace:

```bash
colcon build --symlink-install --packages-select robot_control
source install/setup.bash
ros2 launch robot_control hw3_slam.launch.py drive:=false
```

If that pipeline is already running stationary, use it. Keep the robot and tag
still throughout collection. Disable other driving controllers. The sampler
aborts if it receives a nonzero motor command, but cannot detect manual movement
or residual coasting.

## 2. Collect at the measured position

Open another terminal on the robot, source the workspace, and run:

```bash
source install/setup.bash
ros2 run robot_control camera_noise --x-ft 0 --y-ft 6
```

Tag **4** is the default. Use `--tag-id ID` to sample a different tag.

| Argument | Meaning |
|---|---|
| `--x-ft 0` | Tag is zero feet left of robot center; right is negative |
| `--y-ft 6` | Tag is six feet forward of robot center |

These are **tag coordinates relative to the stationary robot**, not global map
coordinates. This input convention follows the checklist example; internally
SLAM orders the observation as `[forward, left]`. Measure horizontal offsets
from the robot center to the tag center, not from the camera lens or along a
slanted line to an elevated tag. Six feet is 1.8288 meters.

The default target is 200 observations with a 120-second timeout. At 10 accepted
frames per second, collection takes approximately 20 seconds after settling.
Each observation must have a new, increasing camera timestamp. Identical
coordinates in different frames still count. Stale, future, duplicate, invalid,
and wrong-camera-frame observations are excluded; valid geometric outliers are
retained for noise measurement.

Optional settings:

```bash
ros2 run robot_control camera_noise --tag-id 4 --x-ft 0 --y-ft 6 \
  --samples 200 --timeout-s 180 --output-dir ~/camera_noise_results
```

The sampler reads camera offsets, mounting yaw, settling time, timestamp age
limit, and existing noise settings from the installed `hw3_slam.yaml`. Use
`--config /path/to/your/hw3_slam.yaml` if your running setup uses another file.
Make sure its camera geometry matches the actual mounting and SLAM settings.
Topic overrides are `--tf-topic` and `--command-topic`.

## 3. Read the saved results

Each run creates a new folder under `~/.ros/camera_noise/` and prints its path:

- `observations.csv`: timestamps, forward/left measurements, and original
  camera translations, in meters.
- `summary.json`: input position, settings, software version, sample count,
  completion status, skipped-frame counts, and calculated statistics.

The console prints:

- **Forward and left standard deviations:** measured scatter around each
  coordinate's average, using sample variance with denominator N-1.
- **Combined camera standard deviation:**
  `sqrt((forward_variance + left_variance) / 2)`, matching the current
  equal-noise model `R = sigma^2 I` at this distance. Printed in meters and feet.
- **Mean error (bias):** average measured position minus your supplied position.
- **Configured model standard deviation:** the old model's prediction at your
  supplied distance, for comparison. With the current defaults at six feet,
  this is `0.04 + 0.02 * 1.8288 = 0.076576 m`, not an experimental result.

The JSON also includes the full measured 2x2 covariance, including correlation
between forward and left, and per-coordinate root-mean-square error (RMSE).
Bias and RMSE are separate from the standard deviation: consistent readings can
still be wrong. Your supplied distance labels the experiment and measures bias;
the script estimates scatter from the actual camera observations.

A timeout, Ctrl-C, or motion command saves an explicitly incomplete result and
returns a nonzero exit code. Fewer than two observations cannot produce a sample
standard deviation. Check `status: "complete"` and `collected_samples: 200`
before using a run as your intended 200-sample experiment.

Repeat at several distances to tune `measurement_noise_m` and
`range_noise_fraction`. One distance gives only one noise level, so the script
does not fit both parameters or change your configuration. Distinct frames may
still have correlated errors; repeat runs and camera viewpoints to check how
representative the result is.
