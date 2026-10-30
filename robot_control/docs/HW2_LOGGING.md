# Capture and parse route reports

The updated `approach_tag` / `tag_route` controller emits `HW2_REPORT` JSON in
normal INFO output automatically, and saves those records to a new file under
`~/.ros/route_reports/` on the machine running the controller (the robot when
launched over SSH). No `tee`, terminal setting or parameter is required.

Names use `America/Los_Angeles` time, with automatic daylight saving adjustment:
`tag_route_20261029T143052.123456-0700_a1b2c3d4.log`.
The offset is `-0700` for PDT and `-0800` for PST.
The fractional seconds and run ID distinguish runs. Startup prints the full path.
Each record is flushed immediately; normal shutdown closes the file. These files
contain the structured report records; use the optional `tee` command below to
also capture all launch output, camera diagnostics and errors. Files are retained
until you remove them. To find the latest report:

```bash
ls -t ~/.ros/route_reports/*.log | head -1
```

For standalone `ros2 run`, you can change the directory with
`--ros-args -p report_log_directory:=/path/to/logs`.
This does not add world localization to the driving controller. Measurements are
visual estimates using its existing camera alignment and 0.0889 m forward offset.

On the robot, rebuild/source the workspace after updating this repository:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_control
source install/setup.bash
ros2 launch robot_control tag_route.launch.py
```

Optional full terminal capture (under `~/ros2_ws/logs/` when run there):

```bash
mkdir -p logs
set -o pipefail
ros2 launch robot_control tag_route.launch.py 2>&1 | tee "logs/route_$(date +%Y%m%d_%H%M%S).log"
```

`tee` saves the same output you see, including stderr. Use Ctrl-C normally to
stop the launch. Keep the whole file, including the start record. Run the parser
on the robot or copy the log to your laptop. Example from the repository root:

```bash
python3 scripts/parse_route_log.py /path/to/route.log --output route_report
python3 scripts/parse_route_log.py /path/to/route.log --tag-map measured_tags.json --interval 0.5 --output route_report
```

CSV extraction uses Python's standard library. Plotting uses Matplotlib and works
over plain SSH without a display. Install it on the machine generating plots:

```bash
python3 -m pip install -r scripts/requirements-plot.txt
```

Files:

- `waypoint_errors.csv`: exact final accepted visual residual at each successful
  approach stage, with timestamp, tag, standoff, translation norm in metres and
  signed heading correction in degrees. Alignment-only actions are not waypoints.
- `trajectory.csv`: estimated robot-centre world x/y/heading, sampled nearest
  every 0.5 seconds (or `--interval 1`), retaining actual measurement timestamps.
- `trajectory.svg` and `trajectory.png`: Matplotlib plots of those dots in
  metres, with equal x/y scale, labelled axes, gridlines and a legend. Open the
  SVG in a browser or insert the PNG into a report. Individual sample timestamps
  and coordinates remain available in `trajectory.csv`.

Only successful approaches generate arrival rows; failed/unreached waypoints
are not silently filled in. A partial run prints a warning. Multiple runs in
one file default to the last run; use `--run UUID` to select another.

## Measured tag map required for a world trajectory

Create a JSON file with this structure. These numbers illustrate the format;
**replace them with measured positions and orientations before using results**:

```json
{
  "tags": {
    "tag_9": {"x_m": 2.0, "y_m": 0.0, "normal_yaw_deg": 180.0},
    "tag_8": {"x_m": 2.0, "y_m": 2.0, "normal_yaw_deg": -90.0}
  }
}
```

Include every visible tag used in your run (currently route tags 9, 8, 7, 5).
Choose the start robot centre as world (0,0), +x along its initial heading,
+y to its left. `x_m,y_m` specify the tag centre projected onto the floor.
`normal_yaw_deg` is the direction the printed face points outward, measured
counterclockwise from world +x: 0 points +x, 90 points +y, 180 points -x.
Do not substitute the direction the robot faces when looking at the tag.

The calculation uses each observed tag normal to recover robot heading, then
rotates/subtracts the robot-to-tag vector from that tag's measured world position.
It assumes upright tags and a level, forward-facing camera with no lateral offset.
It rejects normals tilted over 30 degrees. Unmapped tags are skipped and reported.
Multiple simultaneous observations use whichever is nearest the sampling instant;
there is no fusion or smoothing. Different tag-map/calibration errors may cause jumps.
Periods without mapped observations remain gaps; these logs cannot recover motion
while no landmark is visible. A complete continuous estimate requires an additional
localization/dead-reckoning implementation.

## Interpretation of errors

Arrival errors are the controller's visual estimate relative to its tag-normal
standoff goal, **not independently measured physical ground truth or localization
accuracy**. For physical waypoint error, measure the stopped robot against the
floor waypoint and heading marker. Camera calibration and tag-map errors affect
all visual results. The original periodic human-readable logs remain available.
The new format applies to `approach_tag` and `tag_route`, not the separate HW1
`velocity_control` executable.
