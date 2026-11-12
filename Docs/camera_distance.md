# Camera → Foxglove → AprilTag distance runbook

> Historical experiment notes: this checkout does not contain
> `camera_distance.launch.py`, `tag_distance.py`, or the `approach_tag`
> executable described below. These commands require the corresponding
> experimental implementation and are not a runnable guide for this revision.
> For the available camera and detector launches, see
> [camera setup](setup/camera_setup.md) and
> [the robot control README](../robot_control/README.md).
> The tag IDs and ROS distribution below describe that experiment; check the
> current configuration and installed ROS distribution before running commands.

This setup uses **ROS 2 Jazzy** and the robot repository at
`~/ros2_ws/AllenBrenton_rubikpi_ros2`. The last Foxglove screenshot used
`192.168.1.100`; replace that address below if the robot's IP changes.
Run `hostname -I` on the robot to check it.

## 1. Connect from your Mac

```bash
ssh ubuntu@192.168.1.100
```

## 2. Build on the robot (first time or after code changes)

Make sure the updated files from your Mac are on the robot first. If needed,
run this from a separate **Mac terminal**, with any running launch stopped:

```bash
rsync -av \
  --exclude='.git/' --exclude='.DS_Store' --exclude='__pycache__/' \
  --exclude='build/' --exclude='install/' --exclude='log/' \
  ./ \
  ubuntu@192.168.1.100:~/ros2_ws/AllenBrenton_rubikpi_ros2/
```

On the **robot**, use a fresh terminal that has not sourced this workspace's
`install/setup.bash`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws

ls AllenBrenton_rubikpi_ros2/robot_control/launch/camera_distance.launch.py

colcon build --symlink-install --cmake-clean-cache \
  --base-paths ~/ros2_ws/AllenBrenton_rubikpi_ros2 \
  --packages-select robot_vision_camera apriltag_ros robot_control \
  --allow-overriding apriltag_ros
```

Continue only when the build reports all three packages finished successfully.
The clean-cache option handles the old cached source path under `src/`.
The explicit base path selects this repository if other copies exist.
The robot already has the required camera plugin and Foxglove bridge working;
these commands do not reinstall dependencies.

## 3. Start everything on the robot (every session)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_control camera_distance.launch.py
```

Leave this terminal running. This starts the camera, Foxglove bridge on port
8765, AprilTag detector, and distance reporter. It does not start motor nodes.
Do not start another camera launch alongside it.

## 4. Connect Foxglove on your Mac

Open a Foxglove WebSocket connection using:

```text
ws://192.168.1.100:8765
```

In the Image panel settings:

- Set **General → Topic** to `/camera/image_raw/compressed`.
- Set **Rectify** to **Off**; the camera node already rectifies the image.
- Leave AprilTag detection on `/camera/image_raw`; the display choice does not change it.

If the WebSocket connection fails, open another **Mac terminal**:

```bash
ssh -N -L 8765:localhost:8765 ubuntu@192.168.1.100
```

Keep the tunnel running and use `ws://localhost:8765` in Foxglove.
Stop the tunnel with Ctrl-C when finished.

## 5. Show a tag and read distance

Use a **36h11 tag with ID 0 or 1**. The whole black square and its surrounding
white border must be visible. Other IDs are ignored by the current config.

The launch terminal prints output like:

```text
tag_0: robot-centre-to-tag distance = 1.089 m | bearing = +0.0 deg (positive = right) (forward depth = 1.089 m)
```

The numbers above are an example. Bearing is measured from the robot forward
direction: positive means right, negative means left, and zero means straight
ahead. It describes the direction to the tag, not the rotation of the tag face. For a Foxglove Plot panel, add:

```text
/tag_0/distance/data
/tag_1/distance/data
```

Or read the topic in another **robot terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /tag_0/distance
```

Range and bearing are relative to the robot centre. The camera faces forward
at the front bumper, 3.5 inches (0.0889 m) ahead of the centre. The reporter
adds this offset to camera `z` before calculating range and bearing:
`forward_depth = z + 0.0889`, `range = sqrt(x*x + y*y + forward_depth**2)`,
`bearing = atan2(x, forward_depth)`. Positive bearing means right.
The mounting model assumes zero lateral/vertical offset and no mounting
rotation; a measured camera height or tilt would require additional correction.
The constant is `CAMERA_OFFSET_FROM_ROBOT_CENTER_M` in `tag_distance.py`.
The detector's `/tf` poses remain camera-relative.

A lost tag produces NaN after one second; plots may retain
historical readings. Before a tag has been detected, its distance topic has
no readings.

The configuration assumes the **outer black square is 0.1524 m (6 inches) per side**,
excluding the white margin. Measure your tag. If different, update `size`
and the corresponding entries in `tag.sizes` in
`apriltag_ros/cfg/tags_36h11.yaml`, then rebuild and relaunch.

The supplied calibration is untuned. Compare readings against a tape measure;
accurate distance requires calibration of this camera at the capture resolution
(default 1280×720) and the correct physical tag size.

## 6. Stop

Press **Ctrl-C** in the launch terminal and wait for shutdown before restarting.
If shutdown hangs, inspect processes from another robot terminal:

```bash
pgrep -af 'robot_vision_camera_node|apriltag_node|foxglove_bridge|tag_distance'
```

Confirm which processes belong to the old launch before stopping them; do not
start another capture process while the old camera process remains alive.

## Troubleshooting commands

In another robot terminal, first source ROS and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Run these individually. Use Ctrl-C to stop a streaming command before the next:

```bash
# Is the bridge listening?
ss -ltnp | grep 8765

# Are compressed frames arriving? Observe for about 10 seconds.
ros2 topic hz /camera/image_raw/compressed

# Are raw frames arriving?
ros2 topic hz /camera/image_raw

# Is calibration being published?
ros2 topic echo /camera/camera_info --once

# Are any tags detected? An empty detections list means no accepted tag.
ros2 topic echo /detections

# Is the detector producing poses?
ros2 topic echo /tf

# Check publisher/subscriber QoS if synchronization warnings return.
ros2 topic info /camera/image_raw --verbose
ros2 param get /apriltag_node reliable_camera_qos
```

The last parameter should be `True` with the updated detector. If it is missing,
check that the updated source was copied, rebuilt, and sourced on the robot.

- **Missing launch file:** copy the changes and rebuild `robot_control`.
- **CMake source-path mismatch:** rerun the build in step 2 with `--cmake-clean-cache`.
- **Workspace override warning:** use a fresh terminal; sourcing Jazzy again does
  not remove a workspace already sourced in that shell.
- **No image:** inspect camera/GStreamer logs and confirm a single camera process.
  “Pipeline started” alone does not confirm frames are arriving.
- **No detections:** check family, ID, lighting, sharpness, and the full white border.
- **Detections but no distance:** inspect CameraInfo and `/tf`; the reporter expects
  direct transforms from `camera_frame` to `tag_0` or `tag_1`.
- **Synchronization warnings:** compare raw image delivery with CameraInfo and check
  QoS. The camera uses identical timestamps for each image/CameraInfo pair.

## Short motion test: approach to 1 m with steering correction

Copy the latest source to the robot, then build `robot_control` using step 2
(or rebuild all three packages there). Keep the camera/detector launch running.
Use a clear floor area, start just beyond 1 m from the tag, and keep the tag
near the middle of the image. The controller uses the robot-centre range,
including the 3.5-inch forward camera offset. Target accuracy depends on the
camera calibration, tag size, and physical stopping distance.

Do not run keyboard teleop, `move_forward`, or any other motor command source
at the same time. Start the serial motor bridge alone in **robot terminal 2**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_control motor_control
```

Confirm it logs `Serial connection established`. In **robot terminal 3**,
start the movement explicitly (this command arms motion as soon as a fresh tag
pose is available):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_control approach_tag --ros-args \
  -p tag_frame:=tag_0 -p target_distance:=1.0 \
  -p max_command:=0.08 -p max_duration:=20.0
```

Use `tag_1` for ID 1. The node publishes `[L, R]` at 20 Hz. A tag to the right
makes the left wheel faster; a tag to the left makes the right wheel faster.
The forward command shrinks near the target. Commands are firmware units,
not calibrated m/s; 0.08 may be below the motor's movement threshold.
Verify the wheel direction and stop behaviour on a supported chassis with
wheels clear of the floor before the first floor test.

The controller latches a stop when range is at or below 1 m, the tag pose is
older than 0.3 s after tracking begins, the tag is more than 20 degrees off
centre, another motor command publisher is detected, or the 20-second test
limit expires. It waits stationary for its first tag pose. It does not drive backward, but may reverse one wheel to align in place.
It does not restart automatically after a stop. Restart the command to retry.
It continues publishing zero commands while stopped. It has no obstacle detection.

Press **Ctrl-C in the approach controller terminal** to cancel; the motor
bridge's existing 0.15-second command watchdog also zeros the wheels when
commands cease. Stop the motor bridge afterward. If the steering moves the tag
farther from the image centre, stop the test and check motor-side/sign wiring.
Do not compensate by increasing speed. Small commands may stop short due to
motor deadband; this test does not guarantee an exact 1.000 m stop.

### If commands continue but the robot stalls near the target

Observed testing showed the robot stopped moving at commands around
`L=0.077, R=0.045`. The optional `min_command` parameter compensates for this
motor deadband. It defaults to zero to preserve the original behaviour.
After copying and rebuilding the updated `robot_control`, try:

```bash
ros2 run robot_control approach_tag --ros-args -p tag_frame:=tag_0 -p target_distance:=1.0 -p min_command:=0.08 -p max_command:=0.15 -p max_duration:=40.0
```

During forward motion, wheel commands stay above `min_command` while preserving steering;
the controller still sends exact zero on arrival or any stop condition. The
parameters must satisfy `0 <= min_command < max_command <= 0.3`. The 0.08 floor
is an initial estimate from the logs, not a calibrated motor threshold. A moving
floor can cause some overshoot; observe the stop before increasing speed.
Increasing only `max_command` does not cure near-target deadband, because the
proportional command decreases with remaining distance.

### Interval motion from HW1

HW1 `robot_control/robot_control/velocity_control.py` uses at most 4.0 seconds
of motion and 0.3 seconds stopped between segments. The measurement notes in
`Docs/hw1/measurements.md` use 3.5-second drive trials; `scripts/drive.py` pauses
0.5 seconds between trials and accesses serial directly (do not run it alongside
the motor bridge).

The tag approach now defaults to 3.5-second drive intervals and 0.3-second
zero-command pauses. Steering is updated throughout each drive interval.
Tracking checks, arrival stops, and the total time limit remain active during
pauses. The logs show `Interval pause` and `Interval resumed`. This reproduces
the previous test pattern; it does not establish that firmware has a timeout.
After copying and rebuilding the updated controller, test explicitly with:

```bash
ros2 run robot_control approach_tag --ros-args -p tag_frame:=tag_0 -p target_distance:=1.0 -p min_command:=0.08 -p max_command:=0.15 -p max_duration:=40.0 -p drive_seconds:=3.5 -p pause_seconds:=0.3
```

`max_duration` includes pauses. Set `pause_seconds:=0.0` to compare with
continuous motion. Keep the same speed settings when comparing the two modes.


### Align before approaching

If the tag bearing reaches 8 degrees, the controller pivots in place until
bearing is within 3 degrees, then resumes forward motion. Positive bearing
commands `L=+pivot_command, R=-pivot_command`; negative bearing does the opposite.
The 20-degree stop limit and all tracking/interval stops still apply. Logs show
`Aligning in place` and `Aligned; approaching`. This avoids driving toward the
tag with a large bearing error; HW1 calibration indicates unequal wheel response.
Verify that bearing decreases during alignment. If it increases instead, stop
and check motor-side/sign conventions and camera orientation before retrying.


Alignment uses a separate `pivot_command` (default 0.2, maximum 0.3).
A 0.15 forward limit may be insufficient for an in-place turn on the floor.
To test stronger alignment without increasing forward speed, add
`-p pivot_command:=0.25` to the approach command, retaining
`-p max_command:=0.15`. Confirm that bearing decreases during each pivot.
If commands continue but no turning occurs, stop and inspect motor bridge
logs, wheel motion, and power rather than repeatedly increasing commands.

### Use the measured HW1 turn commands

The approach node now defaults to `use_hw1_turns:=true`. Alignment reuses
`kinematic_control.py`: right pivot `(0.45, -0.30)`, left pivot `(-0.45, 0.30)`.
Forward motion still obeys `max_command`; these calibrated pivot commands
intentionally exceed the forward limit. The controller requests 0.05–0.12 s
turn pulses based on remaining bearing and the measured HW1 turn rate, then
publishes zero for 0.2 s before trying again. Pulse timing is quantized by the
20 Hz timer and subject to scheduling/serial delay; verify actual turns on the
robot. Bearing, arrival, tracking-loss, and total-duration stops remain active.

```bash
ros2 run robot_control approach_tag --ros-args -p tag_frame:=tag_0 -p target_distance:=1.0 -p min_command:=0.08 -p max_command:=0.15 -p max_duration:=40.0 -p use_hw1_turns:=true
```

`pivot_command` is ignored in this mode. Set `use_hw1_turns:=false` to return
to the earlier symmetric pivot controlled by `pivot_command`. Forward wheel
commands are not yet compensated using HW1's 0.25:0.20 straight-line ratio.

### Allow for forward coasting after stop

`stop_margin` defaults to 0.05 m. For `target_distance:=1.0`, the controller
commands zero at a reported range of 1.05 m so the robot can coast toward 1 m.
This is a tunable allowance, not measured braking compensation. The controller
latches the stop and does not reverse to correct overshoot. For example, add
`-p stop_margin:=0.08` for an 8 cm allowance. Set zero to restore the old threshold.
Keep speed and floor conditions fixed while tuning: if it stops 2 cm too close,
increase the margin by roughly 2 cm; if it stops short, reduce the margin.
The margin affects forward arrival, not pivot angular overshoot. Valid range is
0–0.3 m. Measure the final resting distance, not the range at the stop command.

## Finish centred and square to the tag

After copying and rebuilding `robot_control`, keep the camera launch and motor
bridge running and explicitly enable the optional pose-based mode:

```bash
ros2 run robot_control approach_tag --ros-args -p tag_frame:=tag_0 -p target_distance:=1.0 -p square_to_tag:=true -p min_command:=0.08 -p max_command:=0.15 -p max_duration:=60.0 -p use_hw1_turns:=true
```

`target_distance` is now the **horizontal standoff from the tag plane to the
robot centre along the tag's outward normal**, rather than a 3D slant range.
The target point is `tag_position + target_distance * outward_normal`. The
final forward heading faces opposite that normal. This requires the tag pose
quaternion, not just its image bearing. The camera's 3.5-inch forward offset
is included; the existing assumptions about camera centring and tilt remain.
Use an upright, fixed tag facing the robot and begin farther out than the
requested standoff, with only a modest lateral/heading offset.

The controller approaches the target point, rotates to face the tag plane,
and publishes zero commands while verifying **8 cm position error and 3 degrees
heading error for 0.5 seconds**. Logs add `goal_error`, `square_heading`, and
`phase` (`POSITION`, `ORIENTATION`, `SETTLE`). Square mode uses this pose tolerance
instead of the scalar `stop_margin` arrival rule. The position tolerance permits
some forward coasting; final resting accuracy must be measured physically.
It does not reverse to recover a goal behind the robot. It does not report
success based solely on reaching the scalar range.

Square mode also uses HW1's 0.25:0.20 forward wheel ratio, within the requested
wheel command limits. The ratio was measured at a different speed; this is an
initial compensation, not a new calibration. Calibrated pulsed pivots, interval
pauses, tracking loss, the 20-degree tag-bearing bound, publisher-conflict checks,
and the total-duration stop remain active. Large lateral offsets may need turns
that lose visibility; in that case it stops rather than driving blind. Begin
near the tag's perpendicular centreline for the first hardware test.

This feature has local geometry and policy tests, but needs robot validation.
Camera calibration, pose noise, motor deadband, pivot pulse timing, and coasting
limit achievable accuracy. A timeout/visibility stop is not successful alignment.
