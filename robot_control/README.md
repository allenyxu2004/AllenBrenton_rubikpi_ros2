# HW2 tag route

Corrective pivots use nominal 0.05 s pulses. After a turn, the controller
publishes zero commands for at least 0.4 s and waits for a target image captured
after that settling period before turning again or driving. Delayed images
captured during the turn cannot trigger another correction. Initial tag search
can continue without observations, with the same minimum pause between turns.
Pulse timing is limited by the ROS timer and serial bridge scheduling.

Build and run on the robot:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_control robot_vision_camera apriltag_ros
source install/setup.bash
ros2 launch robot_control tag_route.launch.py
```

This starts autonomous motion, camera/Foxglove, detection and the motor bridge.
Start with tag 9 visible and no other motor-command publisher. Ctrl-C stops it.

Edit `robot_control/route_config.py` for the route, distances and motion settings.
The route approaches tags 9 → 8 → 7 → 5 → 9, stopping square to each at 0.5 m,
except the final stop at 1.5 m. These are robot-centre standoffs along the tag
normal, with 8 cm position and 3° heading tolerances. Alignment stages search left.
Tracking loss stops motion; failure stays stopped until the controller is restarted.
Existing HW1 motor calibration is imported without changing the HW1 code.

Each run saves `~/.ros/route_reports/tag_route_<Los_Angeles_timestamp>.jsonl`.
Generate waypoint-error CSV:

```bash
python3 scripts/parse_route_log.py /path/to/report.jsonl --output route_report
```

For trajectory CSV and SVG/PNG plots, install the plotting dependency and supply
measured tag poses (metres, outward normal heading counterclockwise from world +x):

```bash
python3 -m pip install -r scripts/requirements-plot.txt
python3 scripts/parse_route_log.py /path/to/report.jsonl --tag-map tags.json --output route_report
```

`tags.json` format (replace these examples; include route tags 9, 8, 7 and 5):

```json
{"tags":{"tag_9":{"x_m":2.0,"y_m":0.0,"normal_yaw_deg":180.0}}}
```

Choose the initial robot centre as world origin, +x forward and +y left. The
camera must be level and forward-facing; tags must be upright with 0.1524 m black
edges. Recalibrate the camera before trusting poses. Reports contain visual
estimates, not independently measured physical error. Trajectory samples use
0.5 s spacing (`--interval` overrides it); tracking gaps remain gaps. Partial
runs produce only the waypoints actually reached. Old launch logs are not inputs.

## HW4 known-map obstacle navigation

HW4 uses **Voronoi for safety** and **visibility graph for geometric distance**.
NetworkX supplies A*/graph search and SciPy constructs Voronoi sites/ridges.
All collision geometry uses analytic rectangle/segment calculations, with no
Shapely dependency. Both planners use radius + margin + waypoint tolerance
(currently 0.28 m); the extra covariance/stopping planning reserves are removed.

[implementation.md](../Docs/hw4/implementation.md) contains the current design,
[layout](../Docs/hw4/layout.png) and [roadmap preview](../Docs/hw4/roadmaps.png).
The [course YAML](config/hw4_course.yaml) has the user-confirmed 10 ft square,
start `(2,8)` ft facing -y, goal `(8,2)` ft, eight perimeter tags and four obstacle
face tags (24 bottom, 23 left, 22 top, 21 right). Tag 7 stays at `(0,1)` ft.

Version 0.1.42 anchors the configured initial pose with zero covariance. Camera
observations cannot relocate the stationary robot. Initial pivots change heading
while x/y remain fixed; the first translational prediction releases the position
anchor, after which EKF prediction/corrections operate normally. The anchor is
never reapplied at later stops. Logs record `start_anchor` and
`start_anchor_released`; the old initial-pose uncertainty parameters are removed.

Execution follows HW2/HW3: 1-3 ticks of calibrated pivot/drive, 0.35 s settling,
0.25 s observation, then the next correction. Three ticks deliver three command
intervals even if the third timer callback is slightly early. Correction uses
fixed-map EKF observations while stopped. Grazing detections over 75 degrees
from a front-on view are skipped using tag quaternion/sightline geometry;
other tags and motion prediction continue. The same tag is accepted again when
its view improves. No tag ID is blacklisted and there is no two-second tag-loss
or 5 cm route-deviation stop.

The core motor watchdog, timing/publisher/bridge faults, run timeout and HW3
uncertainty limits (0.60 m position, 0.80 rad heading) remain. Known-obstacle
footprint, waypoint-connector and forward-pulse checks remain; blocked connectors
replan while stopped. Normal pose shifts steer the next pulse. Goal arrival uses
the latest estimate after the normal settle/observe cycle. Completion/fault
continues zeros until shutdown. Verification flags are no longer required;
`drive=false` remains the default.

Preview and test on the laptop after installing project dependencies (`uv sync`):

```powershell
$env:PYTHONPATH = Join-Path (Get-Location) 'robot_control'
.\.venv\Scripts\python.exe -m robot_control.hw4.plot robot_control/config/hw4_course.yaml --output hw4_paths.png
.\.venv\Scripts\python.exe -m unittest discover -s robot_control/test -p test_hw4.py -v
```

The plot also creates JSON plans and CSV metrics. Add `--safety-log RUN.jsonl`
and/or `--distance-log RUN.jsonl` for recorded paths/online estimates. Existing
outputs require a new filename or explicit `--overwrite`. Estimated distances
include EKF correction jumps; independent physical measurements stay separate.

Connect/build using the [robot workflow](../Docs/hw4/memory.md#ssh-build-and-run-on-the-robot).
Use the existing checkout in `ros2_ws`, preserve robot-side changes, source the
installed ROS distribution (Jazzy on the checked robot), build from the workspace
root and source the overlay. Runtime needs SciPy, NetworkX, NumPy, PyYAML and
ROS `nav_msgs` alongside the existing camera/detector/serial dependencies.

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_control robot_vision_camera apriltag_ros
source install/setup.bash
ros2 launch robot_control hw4_navigation.launch.py objective:=safety drive:=false
```

`drive=false` starts camera/detector/localization/planning without a motor-command
publisher or bridge. Inspect `/hw4/pose`, `/hw4/path` and the console/log. Stop the
preview before launching another camera/controller. Set the robot at the marked
start; run these motion commands separately, resetting between runs:

```bash
ros2 launch robot_control hw4_navigation.launch.py objective:=safety drive:=true
# Separate run after stopping/resetting:
ros2 launch robot_control hw4_navigation.launch.py objective:=distance drive:=true
```

Use `course:=/path/to/course.yaml` and `controller:=/path/to/controller.yaml`
for alternate maps/settings. [hw4_navigation.yaml](config/hw4_navigation.yaml)
selects calibration, pulse/settle times, observation incidence and uncertainty
limits. Keep only one motor-command publisher and one serial bridge. Ctrl-C
sends stop; restart after a latched core fault. The map assumes fixed obstacles.

Logs in `~/.ros/hw4` record version/settings, plans, observations/incidence,
predictions, pulses, commands and outcome. The version 0.1.40 safety run reported completion in 104.85 s with a 2.36 cm
estimated goal residual. Its [saved logs and trajectory figures](../Docs/hw4/implementation.md#completed-safety-run-november-21-2026-436-pm-pacific) distinguish commanded-motion predictions from stationary localization corrections. These are online estimates, not independent physical tracking measurements. A distance-mode robot trial remains pending.
