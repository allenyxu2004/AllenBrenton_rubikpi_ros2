# Project memory for HW4

Last reviewed: 2026-11-20. This describes the local working tree, including uncommitted files; it is not a record of what was submitted or deployed. HW4 design choices below are agreed plans, not implemented features. See [implementation.md](implementation.md) for the build sequence.

## Quick lookup and source precedence

| Need | Source |
|---|---|
| SSH, source sync, build, launch, and log retrieval | [Robot workflow below](#ssh-build-and-run-on-the-robot), [existing HW3 run guide](../hw3/RUN_ON_ROBOT.md) |
| HW4 requirements and deadline | [Assignment PDF](CSE276A_26_HW4.pdf), [rubric](rubric.txt) |
| Repository rules and versioning | [AGENTS.md](../../AGENTS.md) |
| Current HW3 runtime parameters | [hw3_slam.yaml](../../robot_control/config/hw3_slam.yaml) |
| Online SLAM math | [ekf.py](../../robot_control/robot_control/slam/ekf.py) |
| Timing, observations, commands, logs | [slam_node.py](../../robot_control/robot_control/slam_node.py) |
| HW3 final report | [HW3 Report - FINAL.pdf](../hw3/ReportProgress/HW3%20Report%20-%20FINAL.pdf) |
| Offline reconstruction and plotting | [REPORT_PLOTS.md](../hw3/REPORT_PLOTS.md), [smooth.py](../../robot_control/robot_control/slam/smooth.py) |
| Noise measurements and tuning history | [EXPERIMENTS.md](../hw3/EXPERIMENTS.md) |

Use the assignment/rubric for requirements, current code and launch-selected YAML for runtime behavior, and the final report for reported results. Older implementation plans and experiment notes can describe earlier settings or proposals. Where these disagree, preserve the distinction rather than treating every document as current implementation.

## SSH, build, and run on the robot

### Connect from the laptop

Run in a laptop terminal (PowerShell or another SSH-capable shell):

```powershell
ssh ubuntu@ubuntu.local
```

The user supplies the password when prompted. Do not put the password in documentation, commands, or logs. Use this hostname for SSH and SCP; older IP addresses and SSH aliases in historical guides are not the current connection instructions.

### Identify ROS and the existing checkout

After login, run these commands in the **robot's Bash shell**:

```bash
ls /opt/ros
echo "$ROS_DISTRO"
cd ~/ros2_ws
colcon list
```

`colcon list` reports package names and their source paths. Locate `robot_control`, `robot_vision_camera`, and `apriltag_ros`; their parent checkout is the repository to update. Existing guides mention both `~/ros2_ws/AllenBrenton_rubikpi_ros2` and `~/ros2_ws/src/AllenBrenton_rubikpi_ros2`. Use the discovered checkout, not an assumed duplicate. Build from `~/ros2_ws`, not from the checkout or a package directory.

The [HW3 run guide](../hw3/RUN_ON_ROBOT.md) and [camera guide](../setup/camera_setup.md) use **Jazzy**, while the starter instructions assume **Humble**. Inspect `/opt/ros` and the robot's existing environment before selecting a distribution. Do not install or switch distributions merely to match an old example. The commands below use Jazzy; replace `jazzy` with `humble` if that is the installed distribution used by this workspace.

### Update source before building

Local Windows edits do not automatically update the robot. Inspect `git status --short` in the discovered robot checkout before transferring files. For committed changes on a clean checkout, update the intended branch with `git pull --ff-only`. For uncommitted laptop changes, copy the specific changed files into the corresponding checkout paths; include new modules and any modified `setup.py`, package manifest, launch, and configuration files. Preserve robot-side changes and do not copy laptop `.venv`, `build`, `install`, or `log` directories.

For example, from laptop PowerShell, **replace `ROBOT_CHECKOUT` with the absolute checkout path discovered above** before using:

```powershell
scp .\robot_control\config\hw3_slam.yaml ubuntu@ubuntu.local:ROBOT_CHECKOUT/robot_control/config/hw3_slam.yaml
```

That example transfers only one config; it is not a complete software deployment. Rebuild after transferring the full intended change set.

### Build and source on the robot

These commands follow the existing [HW3 build/run guide](../hw3/RUN_ON_ROBOT.md) and [HW2 package README](../../robot_control/README.md):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_control robot_vision_camera apriltag_ros
source install/setup.bash
ros2 pkg prefix robot_control
ros2 pkg executables robot_control
```

Stop if the build fails. `ros2 pkg prefix` should resolve to the intended workspace installation; executable listing should include `hw3_slam`. For subsequent Python/controller/config-only changes, rebuild with `colcon build --symlink-install --packages-select robot_control`, then source `install/setup.bash` again. Rebuild the camera package when changing its C++ source or calibration assets.

In **every new robot terminal**, source the selected ROS distribution and then the workspace overlay:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

The existing run guide documents dependency setup. The board also needs `qtiqmmfsrc`, detector dependencies, Foxglove bridge, Python dependencies, and `dialout` serial access. Treat package installation as setup/troubleshooting rather than a step to repeat for each build.

### Run the implemented HW3 stack

Stop other controllers and serial scripts first. The combined launch starts camera, detector, motor bridge, and controller. Begin with a stationary check:

```bash
ros2 launch robot_control hw3_slam.launch.py drive:=false
```

Keep the robot still. In a second SSH terminal, source ROS and the workspace as above, then inspect:

```bash
ros2 topic echo /slam/pose --once
ros2 topic echo /tf --once
ros2 topic info /motor_commands --verbose
ls -lt ~/.ros/hw3_slam
```

Check the launch console for a successful serial connection, expected software version, effective turn rates, and log path. Confirm camera-to-tag TF and exactly one motor-command publisher. Press Ctrl-C in the launch terminal before starting another run.

After checking calibration and clearing the physical route, these commands **start autonomous motion**. Run one at a time, with the robot reset to its measured start:

```bash
ros2 launch robot_control hw3_slam.launch.py drive:=true shape:=square side_m:=0.9 laps:=1
# Separate run, after stopping the square launch and resetting the robot:
ros2 launch robot_control hw3_slam.launch.py drive:=true shape:=octagon side_m:=0.45 laps:=1
```

Use `laps:=3` for repeated laps. Completion leaves the node publishing zeros; Ctrl-C closes the run. HW3 has no obstacle avoidance: these polygon commands are for a clear HW3 test area, not the HW4 obstacle layout. See the existing run guide for calibration checks, route geometry, and failure behavior.

### Retrieve logs and distinguish future HW4 commands

After stopping the launch, copy the filename printed by the node. From laptop PowerShell, replace `RUN_FILENAME.jsonl` with that exact filename:

```powershell
scp ubuntu@ubuntu.local:~/.ros/hw3_slam/RUN_FILENAME.jsonl .
```

See [REPORT_PLOTS.md](../hw3/REPORT_PLOTS.md) for laptop evaluation. The existing `move_log.sh` helper uses the older `276a` alias, so the explicit SCP command above avoids depending on that alias.

**HW4 is not executable yet.** The planned `hw4_navigation.launch.py`, `hw4_navigation`/`hw4_report` entry points, `hw4/pose`, and `hw4/path` are future additions. See [the implementation runbook requirements](implementation.md#robot-runbook-to-deliver-with-hw4) for the commands the implementation must support. Building the current project does not create those algorithms.

## Project structure and operating constraints

| Location | Responsibility |
|---|---|
| `robot_control/` | `ament_python` ROS package: serial motor/IMU bridge, HW1 controller, HW2 tag route, HW3 EKF-SLAM, samplers, offline analysis |
| `robot_vision/` | C++17 `ament_cmake` package named **robot_vision_camera**: Qualcomm GStreamer camera, raw/JPEG images, CameraInfo, Foxglove bridge |
| `apriltag_ros/` | Vendored C++14 AprilTag detector, built with `-Werror`; publishes detections and camera-to-tag TF |
| `scripts/` | Calibration checks, route-log parsing, shell helpers, analysis dependency lists |
| `Docs/hw1`, `Docs/hw2`, `Docs/hw3` | Measurements, reports, plans, explanations, and experiment records |
| `Homeworks/` | Historical assignment copies and HW3 question notes |
| `pyproject.toml` | Laptop analysis dependencies; distinct from robot ROS packaging |

The intended runtime is ROS 2 Humble on the RubikPi with an OV5647 camera. Windows is the development/analysis environment. Build in the robot's colcon workspace with `colcon build --symlink-install`, then source `install/setup.bash`. Launch files find installed package shares; do not add new machine-specific absolute paths.

- Controllers publish `motor_commands` (`std_msgs/Float32MultiArray`, `[L, R]`). The motor bridge alone opens `/dev/ttyUSB0`, 115200 baud, and writes newline-delimited JSON.
- Commands are firmware units, clipped to `[-0.5, 0.5]`, not verified wheel speeds. Publish continuously at about 20 Hz. The bridge zeros commands after 0.15 s without an update.
- Straight and left/right pivot motions have separate empirical speed/rate calibration. No calibrated wheel-radius/track-width conversion, `cmd_vel` interface, or ROS wheel odometry was found in these controllers.
- The bridge publishes `imu_data`, but the homework controllers discussed here do not fuse it. Gyro units remain a bridge concern. A serial-open failure logs a warning while the bridge stays alive; a subscriber count alone does not prove hardware readiness.
- Never run a second motor-command publisher or a direct-serial script alongside the active controller. Keyboard teleop depends on a display session and is unsuitable for plain SSH.
- Python executables and launch/config files are installed through [setup.py](../../robot_control/setup.py). C++ assets use CMake installation rules.
- Increment the `robot_control` patch version once per change set and mirror it in `package.xml` before committing. `setup.py` reads `version.py`; startup and JSONL configuration records must use that value. Leave vendored versions unchanged.

## HW1: calibrated open-loop control

The implemented [velocity_control.py](../../robot_control/robot_control/velocity_control.py) loads `(x, y, heading)` waypoints and uses turn, drive, final-heading stages. [kinematic_control.py](../../robot_control/robot_control/kinematic_control.py) supplies angle normalization, calibration constants, and an integrated pose estimate. Position comes from commanded rates and elapsed time, not visual or IMU feedback.

Its calibration is historical: forward `[0.25, 0.20]` corresponds to about 0.29845 m/s; left and right rates derive from 90-degree turns taking 0.989 s and 0.859 s. These differ from HW3 short-pulse calibration. Do not mix command pairs, pulse regimes, and rates across homeworks.

The [HW1 report](../hw1/hw1_report.pdf) describes rotation slip, changing pivot centers, battery sensitivity, and accumulating error (roughly 12 inches by the final return in its listed run). [Measurements](../hw1/measurements.md) include older raw trials. The lesson for HW4 is to remeasure short-pulse behavior and verify physical performance independently of the integrated estimate.

## HW2: visual tag-relative feedback

The actual controller is [tag_route.py](../../robot_control/robot_control/tag_route.py) plus [tag_route_policy.py](../../robot_control/robot_control/tag_route_policy.py), with route settings in [route_config.py](../../robot_control/robot_control/route_config.py). It alternates approach and alignment stages through tags `9 -> 8 -> 7 -> 5 -> 9`, with 0.5 m robot-center standoffs except the final 1.5 m approach.

- Live control uses tag-relative goal geometry, including the tag face normal and camera offset. It does not maintain a fused global pose EKF.
- Numerical, timestamp, depth, quaternion, and geometric validity checks reject unsuitable measurements. This is not temporal smoothing of ordinary pose noise.
- Short corrective turns are followed by settling and an observation captured after settling. Stale target feedback stops motion; brief losses can recover, persistent losses latch failure. Initial alignment search is distinct from general recovery after losing a tracked target.
- JSONL records feed [parse_route_log.py](../../scripts/parse_route_log.py), which can reconstruct world pose offline using measured tag poses. Offline reconstruction is not live global localization.

The [performance reflection](../hw2/part_j_overall_performance.md) records one completed run in 106.86 s, estimated mean position residual 4.01 cm, and mean absolute heading residual 1.12 degrees. About 48.1% of control updates commanded a stop. These are controller-derived residuals, not independently measured physical errors.

The [regrade assessment](../hw2/regrade_assessment.md) highlights missing explicit total-distance reporting and the distinction between estimated residuals and physical localization errors. The older [HW2 implementation plan](../hw2/HW2_IMPLEMENTATION_PLAN.md) contains proposals that were not all implemented. Carry these reporting lessons into HW4.

## HW3: online EKF-SLAM

[IMPLEMENTATION.md](../hw3/IMPLEMENTATION.md) explains the architecture. The pure Python [EkfSlam](../../robot_control/robot_control/slam/ekf.py) starts with robot pose `(0, 0, 0)`, zero 3x3 covariance, and no landmarks. This fixes the arbitrary local frame; it does not supply a measured world start pose.

- **State:** `[x_r, y_r, theta, m_1x, m_1y, ...]`, dimension `3 + 2N`; tag IDs select landmark entries. N grows dynamically.
- **Prediction:** calibrated physical inputs `(v, omega)` with midpoint heading `a = theta + omega*dt/2`; displacement is `(v*cos(a)*dt, v*sin(a)*dt, omega*dt)`. Full `F P F^T` propagation preserves robot/map correlations.
- **Process noise:** `G diag(translation_noise^2, rotation_noise^2) G^T / dt` while moving; stationary commands add no process noise. Noise amplitudes have units m/sqrt(s) and rad/sqrt(s).
- **Measurement:** robot-relative tag-center `[forward, left]`. [observations.py](../../robot_control/robot_control/slam/observations.py) maps optical `+z` forward and `+x` right into planar robot coordinates, including mounting yaw and translation. Height is discarded.
- **New tag:** inverse observation initializes its map position and expands covariance with robot and existing-map cross terms. The first observation is used once, without an immediate correction from the same data.
- **Known tag:** `h = Rot(theta)^T (m - p)`, with robot and landmark Jacobian blocks. A squared Mahalanobis gate rejects outliers; accepted observations update the joint state with a linear solve and Joseph covariance form.
- **Missing/reappearing tag:** remains in the map while invisible; its ID reconnects later observations. There is no landmark deletion or multi-frame confirmation of a new ID.

### Runtime and active configuration

[hw3_slam.launch.py](../../robot_control/launch/hw3_slam.launch.py) starts camera rectification, unrestricted tag detection, motor bridge, and SLAM. It loads the YAML first; only explicitly supplied launch overrides replace supported settings. `drive=false` is the default.

[slam_node.py](../../robot_control/robot_control/slam_node.py) predicts the previously commanded motion before changing it. At 20 Hz, it selects a pivot or forward pulse toward polygon vertices, stops, settles, and observes before deciding again. It accepts TF observations only while stopped and only when image capture occurred after settling; stale, future, duplicate, and out-of-order frames are skipped.

| Setting | Active YAML value at review |
|---|---|
| Route | square, side 0.90 m, 1 lap, driving disabled |
| Forward command / modeled speed | `[0.25, 0.20]` / 0.219070 m/s |
| Left command / rate magnitude | `[-0.45, 0.30]` / 2.117690 rad/s |
| Right command / rate magnitude | `[0.45, -0.30]` / 1.969739 rad/s |
| Camera offset / mounting yaw | forward 0.0889 m, left 0, yaw 0 |
| Measurement standard deviation | `0.08 + 0.04 * planar_range_m`, metres |
| Translation / rotation noise | 0.08 m/sqrt(s) / 0.15 rad/sqrt(s) |
| Innovation gate | 9.21, squared Mahalanobis distance |
| Pulse / settle / observe | 0.15 / 0.35 / 0.25 s |
| End-of-lap observation pause | 1.0 s |
| Position / heading tolerance | 0.08 m / 0.10 rad |
| Observation age / tag range | 0.6 s / 4.0 m maximum |
| Run timeout | 600 s |
| Position / heading std stop limits | 0.6 m / 0.8 rad |

Python fallback measurement/rotation settings differ from this YAML. The experiment document's opening configuration description is also older. Read the launched configuration and per-run JSONL config event to establish what a run actually used.

Faults include competing command publishers, excessive uncertainty, timeout, and a callback timing gap over the bridge watchdog interval during movement. The node publishes zero commands after finishing or faulting. It publishes `slam/pose` (`PoseWithCovarianceStamped`, frame `slam_map`) and logs configuration/version, predictions, observation diagnostics, commands, estimates, arrivals, completion, faults, and shutdown under `~/.ros/hw3_slam`.

The online filter has no obstacle map or collision checking and no explicit no-tag timeout. It can continue predicting without visible tags until another limit stops it. Its uncertainty thresholds are not suitable HW4 obstacle-clearance guarantees.

### Final report context and interpretation

The [final report](../hw3/ReportProgress/HW3%20Report%20-%20FINAL.pdf) describes an **8 ft x 8 ft** measured area, two tags per side, with centers 3 ft from the corners and 2 ft apart. World origin is bottom-left, +x right, +y up. Its measured tag map is:

| Tag | x (ft) | y (ft) |
|---|---:|---:|
| 4 | 3 | 0 |
| 9 | 5 | 0 |
| 3 | 8 | 3 |
| 0 | 8 | 5 |
| 8 | 3 | 8 |
| 2 | 5 | 8 |
| 7 | 0 | 3 |
| 5 | 0 | 5 |

Square runs use 0.90 m edges and measured start `(2, 2)` ft; octagon runs use 0.45 m edges and start `(3, 3)` ft, both facing +x. World alignment is offline evaluation information, not an online SLAM map prior.

| Report experiment | Mean landmark error | Estimated path RMS |
|---|---:|---:|
| Single square | 11.65 cm | 5.31 cm |
| Single octagon, Section 4.3 | 7.76 cm | 4.49 cm |
| Three square laps | 10.24 cm | 4.66 cm |
| Three octagon laps | 6.31 cm | 4.79 cm |

These final-report figures use full-recording **offline reconstruction**, fitting motion and accepted observations with a soft tag 4-9 parallel prior of 2 degrees. [smooth.py](../../robot_control/robot_control/slam/smooth.py) uses nonlinear least squares and one pose per stopped interval. [report.py](../../robot_control/robot_control/slam/report.py) supports separate online and smoothed plots; its default baseline includes the parallel prior. Do not present this reconstruction as the online controller's performance.

The estimated path RMS measures cross-track deviation from the ideal polygon with arc-length weighting, not independently measured physical tracking error. Landmark errors compare reconstructed tag centers with measured coordinates. Covariance describes model uncertainty, not measured error. Results include stationary observations after driving stopped; single- and multi-lap recordings used different settings, so they do not isolate the effect of additional laps.

**Report inconsistency:** Section 4.6 instead cites single-octagon values 6.35 cm landmark error and 4.50 cm path RMS. Preserve the Section 4.3 table values above and flag the discrepancy; do not silently reconcile it. [report_lap_metrics.csv](../hw3/report_lap_metrics.csv) contains other online/pair-spacing metrics, not substitutes for these reconstructed world-map errors.

Noise trials in [EXPERIMENTS.md](../hw3/EXPERIMENTS.md) show distance-dependent, anisotropic camera scatter, systematic bias larger than still-image scatter, temporal correlation, and turn overshoot. Small stationary scatter is not evidence of small total localization error.

## Corrections to historical starter descriptions

- `velocity_control.py` is implemented and registered; the starter statement that it is missing is outdated.
- [Camera code](../../robot_vision/src/robot_vision_camera.cpp) now labels BGR output `bgr8`. Rectification writes the new intrinsics into K/P, zeroes distortion, and sets identity R. The starter color/rectified-CameraInfo bugs are already addressed; physical calibration still requires verification.
- HW3 [detector configuration](../../robot_control/config/hw3_tags.yaml) uses 0.1524 m (6-inch black edge) tags without the starter ID whitelist. Do not confuse it with the separate vendored detector config.
- No evidence in this review establishes the current physical calibration or what software is deployed on the robot. This was a local source/document review, not a robot experiment.

## HW4 requirements, decisions, and missing inputs

The [assignment](CSE276A_26_HW4.pdf) is due **November 25, 2026, before midnight**. It specifies a 10 ft x 10 ft workspace (3.048 m per side), perimeter landmarks, diagonal start/goal placement, and a central obstacle approximately 1x1 to 2x2 ft. Landmarks may also be placed around the obstacle. Measured landmark positions may be available to the robot.

For **each** maximum-safety and minimum-time/distance solution, motivate representation and algorithm, visualize the representation and path, and describe implementation/results. The report also describes architecture and links two run videos. Submit complete code in a ZIP. The [rubric](rubric.txt) allocates 10 points per objective: representation 1, algorithm 4, visualization 2, video 1, result 1, code 1.

Confirmed with the user:

- Use a **known-map, pose-only EKF**; measured landmark coordinates stay fixed.
- Compare **maximum minimum clearance** with **minimum geometric distance**. Time is a measured secondary result, not the second planner's optimization objective.
- Use a shared grid: widest-path search for safety, A* for distance.
- Physical measurements are pending. Do not reuse the HW3 8 ft map as the HW4 map or treat illustrative dimensions as measured facts.

Before robot execution, measure bounds, obstacle footprint/location, start pose, goal, unique tag IDs/positions and black-edge sizes, full robot footprint about its control center, protrusions/cables, and camera extrinsics. Recheck short-pulse translation/turning on the actual floor and battery. Tag placement must support localization around the obstacle; AprilTags localize the robot but do not detect unmodeled obstacles.

See [implementation.md](implementation.md) for configuration, collision checks, execution guards, tests, and submission evidence. HW1-HW3 behavior is to remain available unchanged.
