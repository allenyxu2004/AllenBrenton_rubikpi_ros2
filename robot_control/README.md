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

The implemented planners are **Voronoi for safety** and **visibility graph for
shortest distance**. NetworkX runs A* and the safety bottleneck searches;
SciPy constructs the sampled-boundary Voronoi diagram. Rectangle intersections,
exposed union boundaries and exact whole-segment clearance use analytic Python
calculations. There is no Shapely dependency or geometry operation.

[The implementation plan](../Docs/hw4/implementation.md) records the design,
[layout](../Docs/hw4/layout.png) and remaining physical checks. The installed
[course YAML](config/hw4_course.yaml) contains the user-provided 10 ft square,
start `(2,8)` ft facing -y, goal `(8,2)` ft, eight perimeter tags and obstacle
face tags 24/23/22/21 clockwise from the bottom. The 14-inch obstacle's exact
center/alignment, physical tag scale, robot radius and motion/error bounds remain
provisional. Rotated objects require conservative enclosing rectangles.

Both planners use the same reserve: footprint radius + engineering margin +
tracking allowance + uncertainty budget + delayed-stop/drift reserve. Safety
maximizes bottleneck clearance on the constructed Voronoi roadmap, then breaks
ties by distance. Endpoint connectors count in its objective. Visibility A*
minimizes geometric distance in the conservative rectangular planning model.
It does not minimize elapsed run time. Only collinear points are removed.

Preview on a laptop from the repository root after `uv sync`:

```bash
PYTHONPATH=robot_control uv run python -m robot_control.hw4.plot \
  robot_control/config/hw4_course.yaml --output hw4_paths.png
```

PowerShell equivalent with an existing project virtual environment:

```powershell
$env:PYTHONPATH = Join-Path (Get-Location) 'robot_control'
.\.venv\Scripts\python.exe -m robot_control.hw4.plot robot_control/config/hw4_course.yaml --output hw4_paths.png
.\.venv\Scripts\python.exe -m unittest discover -s robot_control/test -p test_hw4.py -v
```

The figure shows both roadmap representations, reserved boundaries, physical
obstacle and selected paths. Companion JSON and CSV contain plans/metrics.
Existing outputs are protected; choose another filename or explicitly pass
`--overwrite`. Add `--safety-log RUN.jsonl` and/or `--distance-log RUN.jsonl`
to overlay recorded plans and online estimates. Incomplete/faulted logs are
identified. Estimate-derived distances include EKF correction jumps and are
separate from independent physical measurements.

Connect/build using the [robot workflow](../Docs/hw4/memory.md#ssh-build-and-run-on-the-robot):
`ssh ubuntu@ubuntu.local`, discover the existing checkout and installed ROS
distro, transfer the changed sources while preserving robot-side work, source
that distro, and build from the colcon workspace root. The new runtime needs
SciPy, NetworkX, NumPy, PyYAML and ROS `nav_msgs` in addition to the existing stack.
On Ubuntu these planner libraries are available as `python3-scipy` and
`python3-networkx`; use the distro's normal dependency setup/rosdep.

```bash
# Robot shell, after sourcing the installed ROS distro:
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_control robot_vision_camera apriltag_ros
source install/setup.bash
ros2 launch robot_control hw4_navigation.launch.py objective:=safety drive:=false
```

In another sourced terminal, inspect `/hw4/pose`, `/hw4/path` and the launch
console/log. `drive=false` starts the camera, detector and navigator but creates
no motor-command publisher and launches no motor bridge. Plans are generated
once by a worker while stationary. Confirm the estimated start, tag scale,
post-settle observations, path and visibility around both sides of the obstacle.
Stop the preview before another camera/navigation launch.

Before driving, verify the actual course, footprint, extrinsics, pulse delivery,
coasting/drift/error bounds, serial readiness and stopping. Record those checks
in `verification_notes` in a surveyed course file and set both
`measurements_verified` and `calibration_verified` only after completing them.
The supplied file deliberately keeps both false; `drive=true` is rejected.
A motor subscriber count does not establish serial health. A bridge-only
stationary check may use `ros2 run robot_control motor_control`; stop it before
the combined drive launch so only one process opens serial.

Motion calibration and pulse/settle settings live in
[hw4_navigation.yaml](config/hw4_navigation.yaml), selected through the `controller`
launch argument. Re-measure on the actual floor/battery. Bounds in the course
must cover the controller's predicted uncertainty and stop envelope, including
0.15 s watchdog expiry, a 0.05 s serial scheduling interval and measured coast.
Do not reduce margins to make a rejected pulse run.

After verification, these commands start autonomous motion. Run separately,
with no competing controller and the robot reset to its marked start:

```bash
ros2 launch robot_control hw4_navigation.launch.py course:=/path/to/verified_course.yaml objective:=safety drive:=true
# Stop the previous launch and reset before this separate run:
ros2 launch robot_control hw4_navigation.launch.py course:=/path/to/verified_course.yaml objective:=distance drive:=true
```

The controller predicts calibrated pulses and corrects against fixed mapped tag
centers while stopped. Each pulse checks a whole swept envelope including
predicted positional/heading uncertainty and stopping/drift bounds. Tracking
violations trigger bounded stopped replanning; a blocked minimum pulse latches
a fault. Tags must supply accepted corrections within 2 s (or a tighter local
budget). Goal success requires a fresh post-settle correction. Completion/fault
continues zeros at 20 Hz until shutdown; restart deliberately after a fault.
Ctrl-C sends a stop. Keep the course static: this system cannot detect new
obstacles. Audit tag facing/occlusion in the physical course.

Logs under `~/.ros/hw4` contain version, resolved settings/map, plan generations,
roadmaps, observations, covariance, commands, envelope decisions and outcome.
Local offline checks do not validate robot ROS integration, serial delivery or
physical safety. Finish stationary checks, isolated motion/stop trials and
staged obstacle trials before recording repeated full runs and report videos.
