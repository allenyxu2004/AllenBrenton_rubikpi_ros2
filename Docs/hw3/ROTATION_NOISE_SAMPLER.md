# Repeat measured turns to estimate rotation Q

Stop SLAM, teleop, and other command publishers. In a sourced robot terminal,
run exactly one motor bridge:

```bash
ros2 run robot_control motor_control
```

In another sourced terminal, run:

```bash
ros2 run robot_control rotation_noise left
```

Default: ten nominal 90-degree turns. Reset the robot to the same marked center
and heading before each trial, then press Enter to move. After it stops, enter
the **actual turn magnitude in degrees**, such as `88.5` or `92`, not the error
relative to 90. Use positive magnitudes for both left and right turns. If using
a 90-degree floor reference, actual angle is 90 plus overshoot or minus
undershoot. The last short pulse can make the command prediction exceed 90;
statistics compare the entered angle with the printed command prediction.

Repeat separately for right turns:

```bash
ros2 run robot_control rotation_noise right
```

Optional: `--trials 5 --angle 90`. Enter `q` at a prompt to finish early or
Ctrl-C to stop. Completed measurements are retained; interrupted movement is
excluded. Reset only when stopped. Each trial requires explicit Enter input.

The sampler reads signed wheel commands, turn rate, pulse length, and pauses
from the installed `hw3_slam.yaml`; it does not use camera or IMU feedback.
It reports mean angle error, sample standard deviation, and estimated
`rotation_noise = std(error_radians) / sqrt(commanded_moving_seconds)`.
Mean bias should be addressed through rate calibration separately. At least
two measured trials are needed for a standard deviation. Physical measurement
uncertainty contributes to the observed scatter.

Timestamped JSONL logs are saved under `~/.ros/rotation_noise/`, including
settings, pulse durations, actual elapsed pulse times, measurements, running
statistics, and final completion status. The estimate uses planned moving time,
excluding stopped pauses; inspect elapsed times for scheduling discrepancies.
The script does not update YAML automatically. Keep left/right results separate
and choose/document one shared rotation-noise setting for the current filter.
