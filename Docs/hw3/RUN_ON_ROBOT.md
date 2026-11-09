# HW 3: build, run, and evaluate SLAM

This implements a planar EKF-SLAM system and a square/octagon waypoint controller.
It starts with **zero landmarks**. AprilTag IDs identify landmarks; their world
coordinates and total count are never supplied to the filter. Startup defines
the arbitrary map pose `(0, 0, 0)`.

The implementation has been numerically tested on the development computer.
ROS launch, camera timing, motor response, and real mapping accuracy still need
to be checked on the RubikPi. This guide does not represent a completed robot run.

## 1. Prepare the robot and environment

Set up the assignment's 10 ft square (minimum 8 ft), with two distinct tags on
each side. Use unique 36h11 IDs. Measure tag **centers** in a room/world coordinate
system. Record the robot center's startup position and heading in that system,
but do not give these measurements to SLAM during a run.

Keep the camera level and forward-facing. Position tags so the camera can see
them, with overlapping views where possible. Keep the driving route clear and
leave enough clearance for the whole robot, not just its center.

Copy/sync this updated repository to the robot's existing workspace before
building. The commands below assume the checkout is
`~/ros2_ws/src/AllenBrenton_rubikpi_ros2`; adjust that checkout path if necessary.

Humble and Jazzy are ROS 2 distributions (named releases). This guide assumes
Jazzy; verify it is installed on the robot before continuing:

```bash
ssh ubuntu@192.168.1.61
ls /opt/ros
```

If that lists `jazzy`, use the commands below. If it lists another distribution,
use that name in the setup paths and `ros-<distribution>-foxglove-bridge` package.
Do not install another ROS distribution just to match this guide.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"  # Should print jazzy.
sudo apt update
sudo apt install python3-numpy python3-serial \
  ros-jazzy-foxglove-bridge
colcon build --symlink-install
source install/setup.bash
```

The SLAM startup box displays the running software version, full log path,
and actual left/right turn rates. The JSONL config event stores the version
and all effective parameters. Check these after rebuilding: a newer software
version alone does not guarantee that an edited YAML contains the latest rates.
The laptop evaluator also prints the version recorded in the run; older logs
are labeled unversioned.

The existing board setup must already provide `qtiqmmfsrc`, the AprilTag build
dependencies, and serial access through the `dialout` group.
NumPy is needed for the online filter; Matplotlib is only needed on the laptop
for offline analysis.

Stop any HW1/HW2 controller, keyboard teleop, or standalone serial script first.
The HW3 launch starts its own camera, detector, motor bridge, and controller.
Do not launch another motor bridge or command publisher alongside it.

## 2. Check the calibration settings

```bash
cd ~/ros2_ws/src/AllenBrenton_rubikpi_ros2
nano robot_control/config/hw3_tags.yaml
nano robot_control/config/hw3_slam.yaml
```

Check these values before autonomous driving:

| Setting | Meaning |
|---|---|
| `size` in `hw3_tags.yaml` | Actual outer black edge of the printed tag, metres. Default is 6 inches, `0.1524`. |
| Camera intrinsics | Use your measured calibration in `robot_vision/config/camera_parameter.yaml`, at the capture resolution. |
| `camera_forward_m` | Camera optical center ahead of robot rotation center. Default is 3.5 inches. |
| `camera_left_m`, `camera_yaw_rad` | Mounting offset/rotation, positive toward robot left. Defaults are zero. |
| `forward_*`, `left_*`, `right_*` | Short-pulse checks give forward `0.219070` m/s, left `2.117690` and right `1.969739` rad/s; recheck on your floor/battery. |
| Noise parameters | Initial guesses that need fitting from repeated camera measurements and motion trials. |

The camera conversion assumes negligible mounting pitch/roll; substantial tilt
requires a full 3D extrinsic transform before projecting into the ground plane.
Tag height is ignored when forming horizontal measurements. Tag orientation is
not used. This filter uses commanded motion, **not encoder feedback or IMU**.

Rebuild/source after editing configuration so the installed copy is current:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_control
source install/setup.bash
```

`hw3_slam.yaml` lists all SLAM/controller settings, including `drive`, `shape`,
`side_m`, and `laps`. A plain launch uses those YAML values. Launch arguments
override only settings you explicitly supply, for example `laps:=3`.
Keep `drive: false` in the file for stationary startup, or explicitly pass
`drive:=false` when checking the setup.

The HW3 detector has no ID whitelist. Upstream names its transforms `tag36h11:<id>`;
the adapter also understands HW2's `tag_<id>` names. No changes to HW2's tag
configuration are required. The camera launch enables rectification.

## 3. Run a stationary check first

```bash
ros2 launch robot_control hw3_slam.launch.py drive:=false
```

The explicit `drive:=false` continuously publishes zero wheel commands and builds
a map from stationary observations. Leave the robot still: moving it by hand is
not modeled. This is a setup check, not a driving experiment.

In a second SSH terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /slam/pose --once
ros2 topic echo /tf --once
ros2 topic info /motor_commands --verbose
ls -lt ~/.ros/hw3_slam
```

Check that the motor bridge reports a successful serial connection. The SLAM
node cannot tell whether the bridge actually opened the serial port. Verify
that `/tf` contains transforms from `camera_frame` to `tag36h11:<id>`, and the log
contains `observation` entries with `result: "new"` or `"updated"`.
There should be exactly one publisher on `/motor_commands`.

Press Ctrl-C in the launch terminal to end this check. Restarting launches a new
filter and creates a new log. `drive` is selected at startup; changing a live
ROS parameter does not arm the controller.

## 4. Drive the square

Measure and write down the startup robot pose for this run. Face along your
chosen initial map +x direction. A tag need not be initially visible. Then run:

```bash
ros2 launch robot_control hw3_slam.launch.py \
  drive:=true shape:=square side_m:=0.9 laps:=1
```

This **starts autonomous movement** once a motor subscriber is present and the
initial settling/observation window has elapsed, even without a visible tag.
Until a tag is observed, pose prediction relies on the calibrated motion model.
Corners in the startup map frame are:
`(0.9,0)`, `(0.9,0.9)`, `(0,0.9)`, `(0,0)`. Travel is counterclockwise.

Every decision uses the current SLAM pose. The controller pivots toward the next
corner, then drives forward in short pulses. It stops between pulses, waits for
mechanical settling, and accepts only images captured after settling. Without
visible tags it continues using motion prediction; there is no no-tag timeout.
It stops for
excessive pose uncertainty, a motion callback gap, another command publisher,
or the route time limit. A fault requires restarting the run.

On completion it prints `Route complete; remaining stopped` and continues
publishing zeros. Press Ctrl-C to close the log and stop the launch. Ctrl-C
during motion also requests zero commands; the bridge's 0.15 s command watchdog
is an additional stop mechanism. Stay ready to physically stop the robot during
initial trials. If the camera process hangs during shutdown, verify motion has
stopped before restarting the stack.

## 5. Drive the octagon and repeated laps

Reset the robot physically to the measured start location, or measure its new
startup pose. Start a fresh run:

```bash
ros2 launch robot_control hw3_slam.launch.py \
  drive:=true shape:=octagon side_m:=0.45 laps:=1
```

`side_m` is the length of **each edge**, not the total width. An octagon has a
bounding width and height of `(1 + sqrt(2)) * side_m`: about 1.09 m here. Starting
at `(0,0)` along +x, this octagon extends from approximately `x=-0.318` to
`x=0.768` and `y=0` to `y=1.086`. Account for the part behind the start x.
The 0.9 m square and 0.45 m octagon are conservative initial sizes; choose and
justify your final experiment sizes based on visibility and clearance.

For repeated laps, keep the same filter alive throughout the laps:

```bash
ros2 launch robot_control hw3_slam.launch.py \
  drive:=true shape:=square side_m:=0.9 laps:=3
```

```bash
ros2 launch robot_control hw3_slam.launch.py \
  drive:=true shape:=octagon side_m:=0.45 laps:=3
```

Run those commands separately, stopping each launch before starting the next.
At the final corner of every lap, the controller turns back to startup heading
zero, then stays stopped for `lap_observe_s` (default 1 s) before recording the
lap as complete. This gives initially visible tags a chance to correct the pose
and map before the lap snapshot. Reacquisition is not guaranteed, and the pause
ends even if no tag is detected. The lap then completes without another position
or heading correction maneuver. For repeated laps, the next lap starts from the
updated SLAM estimate.

If observations are consistently rejected or the pose jumps, inspect
calibration, extrinsics, timing, and printed tag size before relaxing thresholds.
`settle_s` must exceed the motor stop/camera timing lag. `observe_s` must allow a
fresh processed frame to arrive. The nominal 0.15 s pulse / 0.35 s settling /
0.25 s observation cycle reduces average speed, but instantaneous wheel commands
still use the existing calibrated motion primitives. The motion/noise model does
not model acceleration or wheel slip; short-pulse rates may need remeasurement.

## 6. Plot and measure errors on your laptop

Each run writes a unique JSONL under `~/.ros/hw3_slam/`; the node prints its path.
It records parameters, accepted/rejected observations, commands, arrivals, and
the joint state/covariance every 0.2 s. Each completed lap has an arrival snapshot.

From version 0.1.4, JSONL diagnostics also include:

- `prediction`: actual and integrated elapsed time, commanded speed/turn rate,
  wheel commands, pose before/after prediction, and robot covariance.
- `observation`: state/pose before and after processing, landmark indices,
  camera translation, measurement noise, image age and stop-relative timing.
  For known tags, `diagnostics` contains the predicted measurement, innovation,
  innovation covariance, squared Mahalanobis gate score, and accepted robot gain.
  New tags have initialization metadata instead of a correction innovation.
- `observation_skipped`: tag ID, timing, and reason a frame was discarded
  (moving, unsettled, stale, duplicate, invalid position, or outside range).

These details go to the file, not the terminal. Prediction intervals and pose
corrections can now be distinguished without inferring motion from sparse plot
samples. More logging adds I/O overhead; timing-gap faults still stop the run.
The existing plotting and transfer commands remain unchanged.

After stopping the run, open a terminal on your **laptop**, outside SSH. Copy the
logs from the robot:

```bash
mkdir -p ~/hw3_logs
scp -r ubuntu@192.168.1.61:~/.ros/hw3_slam/. ~/hw3_logs/
```

From the repository root on your laptop, prepare the analysis environment once:

```bash
python3 -m venv ~/.venvs/hw3-analysis
source ~/.venvs/hw3-analysis/bin/activate
python -m pip install numpy matplotlib
```

For each new laptop terminal, activate that environment and, from the repository
root, make the Python package available. No ROS installation is needed:

```bash
source ~/.venvs/hw3-analysis/bin/activate
export PYTHONPATH="$PWD/robot_control"
```

Create a CSV using your actual measurements, with this header and one row per
tag (ID, world x, world y):

```text
id,x_m,y_m
```

For example, save it as `~/hw3_ground_truth.csv`. Do not pass it to the online
SLAM node. Plot a run using its exact log path:

```bash
python -m robot_control.slam.evaluate \
  ~/hw3_logs/REPLACE_WITH_RUN_square.jsonl
```

For quantitative comparison, supply the independently measured startup robot
pose. Replace `X_METRES`, `Y_METRES`, and `YAW_RADIANS` with your measured startup
numbers, then run on your laptop:

```bash
python -m robot_control.slam.evaluate \
  ~/hw3_logs/REPLACE_WITH_RUN_square.jsonl \
  --ground-truth ~/hw3_ground_truth.csv \
  --start X_METRES Y_METRES YAW_RADIANS \
  --output ~/hw3_square.png
```

The tool saves the estimated trajectory, final landmarks, and 95% covariance
ellipses in a PNG, displayed in feet. Ground-truth CSV inputs, startup pose
inputs, and reported errors remain in metres (heading in radians).
Dashed lines automatically label estimated center-to-center distances for tags
7–5, 4–9, 3–0, and 8–2 when both IDs are present. These are map estimates, not
ground-truth distances. Edit `DISTANCE_PAIRS` in `slam/evaluate.py` if the layout
changes.
With ground truth it also saves a per-landmark error CSV and
prints mean Euclidean landmark error at each completed lap and at the end. It
reports missing IDs so an incomplete map is visible. Compare the same set of
landmarks across experiments; a mean over fewer visible landmarks is not a fair
improvement claim. Covariance expresses the filter's model uncertainty, not a
guarantee of physical accuracy.

Repeat for the octagon log. Without `--start`, the plot is in the arbitrary
startup map frame; ground-truth evaluation requires `--start`.

## 7. What belongs in the report

Include the measured map sketch, both trajectories/maps with covariance,
landmark errors, repeated-lap comparison, trajectory size trade-offs, noise
selection evidence, and an unlisted video link. ZIP the code separately.
See [IMPLEMENTATION.md](IMPLEMENTATION.md) for the equations and code structure.
