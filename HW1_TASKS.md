# Homework 1 task checklist

Assignment: [Homework 1 PDF](https://hichristensen.com/CSE276A-26/pdf/hw1-2026.pdf)

Deadline: **October 16, 2026, before midnight.**

Course instruction: do not use the IMU for this homework. Suggested starting approach: calibrated timing for both distance and turning, with position and heading estimated from the kinematic model.

This is an open-loop controller: it does not measure and correct the robot's actual pose during the run. Imperfect waypoint tracking and accumulated error are expected. The goal is to implement the calibrated motion model, measure the resulting performance, and explain its limitations in the report.

## 1. Get the robot running

- [ ] SSH into the robot and source ROS 2 and the workspace.
- [ ] Start `motor_control`.
- [ ] Verify that commands on `motor_commands` move the robot.
- [ ] Resolve why direct serial commands of `3` behaved differently before choosing calibration commands. The current ROS motor node clips each side to `[-0.5, 0.5]`; direct serial writes bypass that clipping. The board's actual supported range is not established.
- [ ] Calibrate using the same ROS command path the final controller will use.

## 2. Define coordinates and get the waypoint file

- [ ] Download the actual `waypoints.txt` from Canvas; verify whether the local file matches it.
- [ ] Define the robot's starting position as `(0, 0)` and starting heading as `0`.
- [ ] Use meters and radians, with counterclockwise turns positive.
- [ ] Treat waypoint angles as absolute headings and state this assumption in the report. The assignment does not explicitly define the heading frame.

## 3. Calibrate motion

- [ ] Measure forward distance and actual elapsed time at a few motor command values.
- [ ] Repeat each trial and calculate average speed in meters per second.
- [ ] Measure sideways drift and find a left/right command pair that drives reasonably straight.
- [ ] Measure left and right turning rates separately, recording angle and elapsed time.
- [ ] Measure turn angles using floor markings or a protractor; verify which command pair turns left and which turns right.
- [ ] Store selected commands and calibration values in a configuration file.

Existing measurements in `Docs/hw1/measurements.md` are a starting point. Convert inches to meters and measure position consistently from the robot's center. Trial counts and these particular calibration experiments are recommendations, not prescribed by the assignment.

## 4. Implement the waypoint controller

Complete `robot_control/robot_control/velocity_control.py`:

- [ ] Load the waypoints using the existing parser.
- [ ] Include a differential-drive kinematic model to estimate motion.
- [ ] Calculate distance and direction to the next waypoint.
- [ ] Turn toward it, drive toward it, and then align to its requested final heading.
- [ ] Use the shortest signed angle difference to choose left or right.
- [ ] Estimate position and heading from calibrated motion rates and actual elapsed time; do not use IMU feedback.
- [ ] Calculate drive duration as `distance / forward_speed` and turn duration as `abs(angle_error) / turn_rate`, choosing the separately calibrated left or right turn rate.
- [ ] Publish `[L, R]` to `motor_commands` continuously at about 20 Hz; the motor node stops after 0.15 seconds without commands.
- [ ] Stop when finished or interrupted.
- [ ] Make speeds, calibration values, stopping tolerances, and waypoint-file path configurable.
- [ ] Add a runnable `main()` for the existing `velocity_control` entry point.

### Kinematic model

With calibrated physical wheel-side speeds `v_L` and `v_R`, and effective track width `b`:

```text
v = (v_R + v_L) / 2
omega = (v_R - v_L) / b

x_next = x + v * cos(theta) * dt
y_next = y + v * sin(theta) * dt
theta_next = theta + omega * dt
```

Motor command values are not verified physical speeds. Calibration supplies the relationship between commands and motion. If forward speed and turning rate are calibrated directly, use those measured values in the pose update equations. Timing-based position estimates accumulate error from slipping, drift, and speed variation.

For straight segments, use the calibrated forward speed as `v` and approximately zero `omega`. For pivot turns, use approximately zero `v` and the calibrated signed turning rate as `omega` (positive for left, negative for right). Check that the chosen turning commands keep the robot's center approximately stationary. Initialize `(x, y, theta)` to `(0, 0, 0)` at the marked starting pose, integrate using actual elapsed time, and normalize heading differences to `[-pi, pi]`. These are open-loop estimates, so validate them against physical measurements rather than IMU readings.

## 5. Test progressively

- [ ] Test a turn in each direction.
- [ ] Test a 1-meter straight drive.
- [ ] Test one waypoint with a final heading.
- [ ] Test several waypoints.
- [ ] Run the full Canvas route, preferably repeatedly.
- [ ] Measure actual position and heading errors using floor markings, a tape measure, and a protractor, independently of the controller's estimates.
- [ ] Record completion time, failures, and accumulated drift.
- [ ] Adjust calibration and controller settings based on measured results.

Use a marked starting pose and the same surface for calibration and route tests.

## 6. Make startup convenient

- [ ] Add a launch file that starts `motor_control` and the waypoint controller together.
- [ ] Update `setup.py` installation settings for new launch/config files.
- [ ] Build on the robot and verify the complete launch command.
- [ ] Document how to build and run the controller.

A combined launch file is optional convenience. Two SSH terminals can run the nodes separately. ROS 2 runs on the robot; a laptop dev container is not required. Keep `motor_control` as the only program opening the motor board's serial port.

### Running the implemented open-loop controller

Calibration constants are at the top of `robot_control/robot_control/velocity_control.py`. The initial forward calibration uses `[0.36, 0.2]` and 44 inches in 4 seconds. Left turning uses `[-0.45, 0.3]` and 90 degrees in 0.738 seconds. Right turning uses `[0.42, -0.3]` and assumes 180 degrees total across the two trials (1.478 seconds). Confirm that assumption on the robot. Motion segments are capped at 4 seconds, with brief stops between them.

On the robot, from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_control
source install/setup.bash
ros2 run robot_control motor_control
```

In a second terminal, source ROS and the workspace, place the robot at the marked `(0, 0, 0)` starting pose, then run:

```bash
ros2 run robot_control velocity_control
# To select a different waypoint file:
ros2 run robot_control velocity_control --ros-args -p waypoint_file:=/path/to/waypoints.txt
```

The controller waits for a `motor_commands` subscriber, then starts automatically. Do not run keyboard teleop or another motor-command publisher alongside it. Ctrl-C sends repeated stop commands. The logged pose is an open-loop estimate, not a physical measurement. Verify the local waypoint file against Canvas before running the full route.

## 7. Record and submit

- [ ] Record an example robot run.
- [ ] Upload the recording to YouTube as **unlisted**.
- [ ] Write a brief report describing the algorithm, kinematic model, control algorithm, and code.
- [ ] Explain calibration, report measured performance, and discuss limitations.
- [ ] Explain why open-loop errors accumulate (for example, wheel slip, unequal motor response, timing variation, and imperfect calibration), including differences between estimated and measured final poses.
- [ ] Include the YouTube URL in the report; do not include the video file in the ZIP.
- [ ] ZIP the report and code and upload to Canvas before the deadline.

The assignment explicitly requires the waypoint algorithm with a kinematic model, Python/ROS 2 implementation on the platform, a run using the waypoint file, a report discussing calibration and performance, code/report ZIP submission, and a video URL in the report.
