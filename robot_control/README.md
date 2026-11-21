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

## HW4 known-map navigation

Edit [config/hw4_course.yaml](config/hw4_course.yaml) with your measured
rectangle boundaries, start position and heading, goal, AprilTag IDs and
centers, tag size, and robot footprint. Coordinates are metres in the frame
shown in the YAML. Use equally sized 36h11 tags. At the start, face the
configured heading with at least one mapped tag visible. Check that tags
remain visible along both paths and at turn locations.

Preview both routes from the repo root. Run `uv sync` to install the
laptop dependencies (NetworkX, Shapely, NumPy, PyYAML, and Matplotlib).

```sh
PYTHONPATH=robot_control uv run python -m robot_control.hw4.plot \
  robot_control/config/hw4_course.yaml --output hw4_paths.png
```

The dotted lines are the grid search paths. Solid lines show the collision-checked
shortcuts the robot follows. NetworkX A* minimizes grid distance. The safety
planner weights edges by clearance and uses a maximum spanning tree to find the
best minimum clearance along a start-to-goal route. It then runs A* on edges at
or above that threshold to break ties by distance. Shapely computes the
distance from each candidate segment to the obstacle geometry. Shortcutting
preserves the safety route's clearance threshold. Reported clearance is measured
from the circular robot footprint
to the nearest obstacle or boundary. The grey grid cells include the configured
margin and waypoint tolerance. If both routes coincide, check whether the
endpoint clearance is limiting the safety objective.

On the RubikPi, install the planner libraries, then build and source the
workspace. The launch defaults to the installed example course and keeps
the motors off. Pass your measured YAML by absolute path to use edits without
rebuilding.

```sh
sudo apt install python3-networkx python3-shapely
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_control hw4_navigation.launch.py course:=/path/to/measured_course.yaml
ros2 topic echo /hw4/pose
```

Check the reported pose against the physical start before enabling motion.
Stop the preview launch and any competing motor or camera launch. Start
the robot at the same marked pose for each run.

```sh
ros2 launch robot_control hw4_navigation.launch.py course:=/path/to/measured_course.yaml objective:=distance drive:=true
ros2 launch robot_control hw4_navigation.launch.py course:=/path/to/measured_course.yaml objective:=safety drive:=true
```

Run those commands separately. The navigator uses the HW3 motor calibration,
predicts pose from the commanded pulses, and corrects it with fixed tag centers
while stopped. It sends zero wheel commands on completion or fault and relies on
the motor bridge watchdog if commands stop arriving. If a mapped tag cannot be
seen for several seconds, check tag placement and visibility in the preview.

The node saves a JSONL log for each run in `~/.ros/hw4/`. Add
`--distance-log <file>` and `--safety-log <file>` to the plot command to overlay
the estimated trajectories and print the elapsed drive time, estimated distance,
estimated minimum clearance, and outcome. Measure physical clearance separately
if you need ground truth for the report.
