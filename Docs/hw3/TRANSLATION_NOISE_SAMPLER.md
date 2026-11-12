# Measure forward movement noise

Stop SLAM, teleop, and other motor-command publishers first, including SLAM with
`drive:=false` (it still publishes stops). No camera or AprilTags are required.
The script publishes `motor_commands`; the bridge owns the serial port. Run only
one motor bridge. These commands work in ordinary SSH terminals on the robot.

Build on the robot and start the motor bridge in one terminal:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_control
source install/setup.bash
ros2 run robot_control motor_control
```

In another terminal, source the workspace and supply one number: feet per trial.

```bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_control translation_noise 3
```

The script runs **10 trials**. Before each, put the robot at the same marked
starting position and heading, clear its path, and press Enter. After it stops,
measure forward displacement from the same point on the robot in **feet**, then
enter that number. Decimal values are accepted (2 feet 6 inches is `2.5`). Measure
along the initial forward direction; record lateral drift separately. Reset the
robot manually for the next trial. Enter `q` at a prompt to finish early, or use
Ctrl-C to stop. Completed measurements are retained; interrupted movement does
not count as a trial.

Motion settings come directly from the installed `config/hw3_slam.yaml`:
`forward_left`, `forward_right`, `forward_speed_mps`, `pulse_s`, `settle_s`, and
`observe_s`. These are the same calibration and pauses used by current SLAM.
Custom runtime overrides on a separate SLAM launch are not read by this script.
Each requested distance is split into capped forward pulses followed by
`settle_s + observe_s` pauses, with commands refreshed at 20 Hz. Like SLAM, the
last pulse has a minimum duration of 0.05 seconds, so the printed predicted
distance can slightly exceed the requested distance. This test uses an identical
open-loop pulse sequence each trial; it does not apply camera corrections.

The console and `~/.ros/translation_noise/translation_noise_<timestamp>.jsonl`
report:

- Each measured distance and signed error: actual minus predicted distance.
- Mean error (bias), separately from sample standard deviation (`N - 1`).
- Error standard deviation in meters and feet, after at least two trials.
- Suggested `translation_noise = error_std_m / sqrt(commanded_moving_s)`.

Moving time is the sum of planned pulse durations, excluding all stopped pauses.
Predicted travel is that time times the calibrated forward speed. The log also
records actual elapsed pulse times for diagnosing scheduling delays. Transport,
serial timing, and measurement error contribute to the observed scatter. The
noise estimate is a starting estimate under the checklist's equal-time model.
Large consistent errors indicate calibration bias, not random noise. Logs include
the software version, calibration, every completed measurement, running statistics,
and a final summary, including on early exit. One measurement leaves the standard
deviation unset (`null`), rather than reporting a misleading zero. Each invocation
starts a fresh experiment; unrelated sessions are not combined. The script does
not automatically change SLAM's noise settings.

To inspect a saved result, use the exact log path printed by the script:

```bash
tail -n 1 "$HOME/.ros/translation_noise/translation_noise_<timestamp>.jsonl"
```

Replace `<timestamp>` with the number in your log filename. The last JSON line
is the `summary` event: `error_std_m` and `error_std_ft` are the requested error
standard deviation, and `translation_noise_m_per_sqrt_s` is the suggested SLAM
setting. Check `trials` and `status` before using it; `complete` means all ten
measurements were entered. An early exit keeps the available results but returns
a nonzero exit code.
