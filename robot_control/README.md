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
