# HW4 implementation plan

Status: agreed design, not yet implemented. Based on the [assignment](CSE276A_26_HW4.pdf), [rubric](rubric.txt), and [project memory](memory.md). The immediate change creates documentation only; the milestones below describe future HW4 software and experiments.

## 1. Goals and architecture

Implement two planners for the same measured static workspace and robot:

- **Safety:** maximize the minimum clearance along the entire path; among paths with that clearance, minimize distance.
- **Distance:** minimize geometric path length in the feasible graph. Record elapsed time but do not claim minimum-time planning.

Use a shared 8-connected grid, a fixed-landmark pose EKF, and the established pulse/settle/observe control cycle. Keep HW1-HW3 controllers and analysis behavior intact. Add HW4-specific pure Python modules under `robot_control.hw4`, a ROS orchestration node, configuration, launch file, and offline report tool. Only the motor bridge opens serial.

Data flow: rectified camera images/CameraInfo -> AprilTag camera-to-tag TF -> planar observations -> known-map EKF -> selected planner/path follower -> `motor_commands` -> existing motor bridge. The map feeds localization and planning; online estimates, commands, and planner decisions feed JSONL logs and report plots.

## 2. Measure the workspace and define configuration

First measure the physical 10 ft x 10 ft workspace, central rectangular obstacle, diagonal start/goal, tag centers, and robot footprint. Use metres/radians internally, a bottom-left world origin, +x right, +y up, and counterclockwise heading. The control center must match the point used for camera extrinsics, calibration, and footprint measurements.

Create a portable environment YAML with these fields:

| Field group | Required contents |
|---|---|
| Measurement status | `measurements_verified`, initially false |
| Geometry | workspace `[xmin, ymin, xmax, ymax]`; axis-aligned obstacle rectangles; robot radius; grid resolution; safety margin |
| Mission | start `[x, y, theta]`; goal `[x, y]`; unique landmark IDs mapped to `[x, y]` |
| Localization | initial pose standard deviations, camera extrinsics, measurement/process noise, gate, accepted-observation timeout |
| Control | straight/left/right command pairs and physical rates; pulse, settle, observe durations; tolerances; run timeout |

Use the measured maximum radius from the control center to any robot part over a pivot. Half the footprint diagonal is sufficient only when that footprint is centered there. Include protrusions and cable motion. This disc is conservative and allows heading-independent collision checks during pivots.

Defaults: 0.05 m grid, 0.10 m additional engineering margin, `mode=safety`, `drive=false`. Carry HW3 timing/rate/noise values forward only as explicitly provisional starting values listed in memory. Initial pose standard deviations start at 0.05 m, 0.05 m, and 0.10 rad; replace with justified measured values when available. Use 0.03 m waypoint and 0.10 rad heading tolerances initially, subject to pulse-resolution validation.

Validate finite values, positive dimensions/noise, unique IDs, command bounds, start/goal feasibility, and consistency with assignment dimensions. Illustrative geometry is allowed for offline development, clearly labelled; reject `drive=true` until measurements and calibration have been verified. No physical measurements are invented by this plan.

Remeasure the tag black edge and camera mounting. Use the HW3 0.1524 m detector setting only if those physical tags are retained. Verify rectification and tag translation at known distances; the old camera software bugs are already fixed.

## 3. Geometry and planners

### Shared feasible graph

Represent the robot center in configuration space. Inflate each obstacle by the robot radius plus margin using Euclidean distance to its rectangle, and inset workspace walls by the same amount. Boundary contact is infeasible. Retain the continuous geometry alongside the grid; checking node occupancy alone is insufficient.

Create lattice nodes at 0.05 m spacing inside the bounds, keep feasible nodes, and connect 8-neighbors only when the complete segment is feasible. Prohibit diagonals between blocked orthogonal neighbors. Edge length is Euclidean distance. Exact segment-to-rectangle and segment-to-wall distances determine clearance, including between samples.

Add exact start and goal vertices, connecting each to every feasible lattice vertex within `sqrt(2) * resolution` if the full connector is collision-free. Add a direct start-goal edge when feasible. Include these connectors in collision checks, objective values, path length, and plotted paths. If a feasible endpoint has no graph connection and no direct route, return an explicit resolution/connectivity failure; never silently move the endpoint. Start equal to goal returns a stationary path after feasibility validation.

Define edge clearance as its minimum distance from physical obstacles/walls minus robot radius and engineering margin. This is residual clearance beyond the common planning buffer. Use the same graph and buffer for both planners so the objective is the only planned difference.

### Safety planner

Run a widest-path variant of Dijkstra: `capacity[start] = infinity`; extending over edge `e` gives `min(capacity[current], clearance(e))`; retain the largest capacity at each vertex. The goal capacity is the optimum bottleneck `c*` on this graph.

Then run shortest-path Dijkstra on edges with clearance at least `c*` (allow a numerical comparison tolerance of 1e-9 m). This second pass implements the shortest-distance tie-break without discarding useful alternatives through a single lexicographic label. Store `c*`, length, expansions, and planning time.

The objective includes start/goal clearance. Endpoint bottlenecks can make both modes select the same path; report this honestly rather than tuning different obstacles or margins to force a difference.

### Distance planner

Run A* with Euclidean edge cost and Euclidean distance-to-goal heuristic. Use deterministic coordinate ordering for equal priorities. State that this finds a shortest route in the configured feasible graph, not an exact continuous-space shortest path or fastest physical execution.

For both planners, remove only collinear redundant vertices; preserve the union of their validated segments and verify the resulting clearance. Do not add shortcut smoothing that changes the selected objective. Return a structured infeasible result when disconnected, with no drive commands.

## 4. Known-map localization and guarded execution

### Pose EKF

Add a separate pure Python filter with state `[x, y, theta]` and 3x3 covariance. Initialize in the measured world frame with nonzero configured covariance. Landmark coordinates are fixed inputs and never added to the state. Unknown IDs are logged/skipped.

Reuse the HW3 optical conversion and angle helpers. Implement the same midpoint motion model, continuous-noise scaling, range-dependent measurement noise, innovation gating, linear solves, and Joseph update in the pose-only filter. For a known landmark `m`, use `h = Rot(theta)^T (m - p)` and the existing robot block of the measurement Jacobian. Treat the measured map as fixed in v1; document that measurement covariance does not model correlated survey error or systematic camera bias.

Accept corrections only when stopped and from distinct image timestamps captured after settling. Preserve wrong-frame, stale/future, duplicate, depth, finite-value, range, and innovation checks. Prediction uses actual elapsed commanded intervals, with the existing watchdog/timing-gap fault behavior. No IMU, encoder fusion, batch smoothing, or delayed-state buffer is required for v1.

### Path execution

The ROS node owns localization, planning, and control timing. Keep 20 Hz publishing independent of expensive search: compute plans in a worker over immutable map/pose snapshots while the robot remains stopped; install a result only if its start connection is still valid. Planning exceptions or failure leave the robot stopped.

- Start stationary. Require at least one accepted known-tag correction before beginning a route. Use the configured measured start heading; a single tag-center observation alone does not establish an arbitrary unknown global pose.
- Pivot toward the next vertex, pulse forward, settle, observe, then recompute. Retain a maximum 0.15 s pulse, minimum deliverable command interval of 0.05 s, 0.35 s settling, and 0.25 s observation window initially.
- Before every pulse, validate the corrected pose, complete next-waypoint connector, and nominal swept center segment. A pivot is feasible only if the full footprint disc is clear. Inflate the swept check by a conservative 3-sigma position bound from the predicted covariance over the pulse; use `3*sqrt(max_eigenvalue(P_xy))` and check intermediate predictions at command intervals. This is a model-based guard, not a physical safety guarantee.
- Include the minimum command interval and one 0.05 s bridge scheduling interval in the checked motion envelope. If the shortest deliverable pulse cannot fit safely, remain stopped and log the reason; never shorten it below what the bridge can reliably deliver.
- At a waypoint, validate the connector from the actual estimated position to the following vertex before advancing. Tolerance must not silently cut an obstacle corner. Replan from the current corrected pose if the remaining connection becomes invalid.
- Stop when no known-tag correction has been accepted for 2.0 s, on excessive uncertainty (initial limits 0.10 m position standard deviation and 0.20 rad heading standard deviation), competing publishers, callback timing faults, invalid pose/map, or 600 s route timeout. The pulse-clearance guard can stop earlier. Require restart after a latched fault; no blind search beside the obstacle.
- Complete only after a final stopped observation window confirms position within goal tolerance. No final heading is required by this plan. Publish zero commands continuously after completion/fault and send a stop during shutdown.

Use the same execution settings for both objectives. Verify bridge readiness and actual zero-motion behavior during the stationary hardware check; ROS subscription presence does not establish serial health. Static map collision checks do not detect newly introduced physical obstacles.

### Interfaces and installation

Provide `hw4_navigation` and `hw4_report` console entry points and `hw4_navigation.launch.py`. Launch arguments are `mode=safety|distance`, `drive=false`, environment config, controller config, and detector config. Resolve installed defaults through package shares. Launch the existing camera, detector, and bridge plus the HW4 node; do not launch the HW3 controller simultaneously.

Publish `hw4/pose` (`geometry_msgs/PoseWithCovarianceStamped`) and `hw4/path` (`nav_msgs/Path`) in frame `hw4_map`. Add `nav_msgs` to package dependencies and install all new launch/config files through `setup.py`. Keep the offline planning/report API free of ROS dependencies: geometry/config -> plan result; JSONL -> plots and metrics.

### Robot runbook to deliver with HW4

Use the [SSH/build/run workflow in memory.md](memory.md#ssh-build-and-run-on-the-robot) and the existing [HW3 run guide](../hw3/RUN_ON_ROBOT.md) as the operational baseline. SSH is `ssh ubuntu@ubuntu.local`; the user supplies the password interactively. Document the discovered robot checkout, selected installed ROS distribution, source-transfer step, workspace-root build, and overlay sourcing. Do not assume laptop changes are already deployed.

After registering/installing HW4 assets, the existing three-package build command in memory.md must build the new stack. Verify `ros2 pkg executables robot_control` lists `hw4_navigation` and `hw4_report`, and check the launched version/config event against the deployed source.

The following commands are the **future interface contract, unavailable until HW4 is implemented**. Defaults resolve to installed measured configuration; `drive=true` must still reject unverified measurements/calibration:

```bash
# Stationary startup and plan inspection; no autonomous motion.
ros2 launch robot_control hw4_navigation.launch.py mode:=safety drive:=false
# In another sourced SSH terminal:
ros2 topic echo /hw4/pose --once
ros2 topic echo /hw4/path --once
ros2 topic info /motor_commands --verbose
```

After stopping the stationary launch, verifying the measured configuration, and resetting to the marked start, run each mode separately:

```bash
# Each command starts autonomous motion. Stop/reset between runs.
ros2 launch robot_control hw4_navigation.launch.py mode:=safety drive:=true
ros2 launch robot_control hw4_navigation.launch.py mode:=distance drive:=true
```

Document Ctrl-C/shutdown, fault restart, the startup log filename, and SCP retrieval from `~/.ros/hw4_navigation` using `ubuntu@ubuntu.local`. Include a verified offline `hw4_report` invocation when its CLI is implemented. Confirm the stationary check produces a path without moving and that both modes work from a fresh SSH shell after the documented source commands. Until then, use the implemented HW3 commands in memory.md only in an obstacle-free test area.

## 5. Logging, visualization, and validation

Each run writes an exclusive JSONL log under configurable `~/.ros/hw4_navigation`. Record software version, complete resolved parameters and measured map, mode, plan/replan reason, vertices, bottleneck, length, planning time/expansions, prediction intervals, pose/covariance, source timestamps, accepted/rejected observations, motor commands, faults, goal confirmation, and shutdown. Record plan generation IDs so trajectory decisions can be traced to the active route.

The offline report tool creates versioned, non-overwriting figures and CSVs:

- Shared occupancy grid and clearance heatmap, with physical obstacle/walls, inflation, measured tags, exact start/goal, and each planned path.
- Online trajectory overlaid on the actual active plans, showing replans and faults. No offline smoothing is used as evidence of live collision avoidance.
- Planned length/minimum clearance, estimated traveled length/clearance, elapsed time, success/failure, final estimated residual, and physical endpoint/clearance measurements entered separately. EKF corrections can create apparent trajectory jumps; label estimate-derived travel metrics accordingly.

### Software checks

Test continuous geometry at walls, rectangle corners, inflated boundaries, diagonal edges, endpoint connectors, narrow passages, and pivot footprints. Test invalid configurations, blocked endpoints, disconnected maps, start equal to goal, and graph-resolution connection failures.

Compare A* cost with Dijkstra. Use small known graphs to prove safety bottleneck selection and the second-pass distance tie-break, including endpoint-limited and equal-objective cases. Verify every returned segment independently for collision/clearance and repeat runs for deterministic output.

Test pose prediction and correction on synthetic fixed-tag observations, heading wrap, nonzero initial covariance, unknown IDs, invalid data/outliers, covariance symmetry/positive semidefiniteness, and finite-difference Jacobians. Confirm landmarks remain fixed.

Test controller decisions with fake clocks/observations: stationary default, first accepted tag requirement, post-settle timestamps, stale/duplicate frames, tag timeout, timing-gap and competing-publisher faults, invalid worker results, covariance guard, minimum pulse envelope, waypoint corner checks, replanning, and final stopped goal confirmation. Test that report generation handles incomplete/faulted logs without claiming success.

### Robot trials and submission acceptance

1. Verify configuration and calibration without driving; check rectified CameraInfo, known tag distances, world pose, timestamp handling, and motor bridge readiness.
2. Test isolated straight and pivot pulses in open space, including minimum pulse size, stopping/coasting, lateral drift, and tag-loss stop behavior. Adjust the measured motion envelope/margin before obstacle trials.
3. Inspect both planned paths and footprint buffers on the measured map. Execute short sections with ample clearance, then full routes.
4. Run each mode at least three times from the same marked start with matching calibration, geometry, and timing. Record failures as well as successes; independently measure final position and minimum body-to-obstacle gap using floor marks/video and tape measurements where practical.
5. Produce the report architecture diagram, representation/algorithm motivation, both representation/path visualizations, quantitative results with estimate/physical provenance, limitations, two run-video links, and complete code ZIP. Use the rubric's 10-point checklist for each mode.

Implementation is accepted when both planners produce validated feasible paths on the measured map, controlled runs reach the configured goal without contact or leaving the workspace, faults stop the robot, and the evidence supports the reported claims. Do not promise visibly different paths or a faster distance-mode run; stop/observe overhead and endpoint bottlenecks can prevent either.

## 6. Change boundaries and immediate documentation checks

The present change adds this plan and [memory.md](memory.md), plus the required synchronized patch version bump. It does not implement navigation, alter HW1-HW3 behavior, deploy to the robot, or claim completed trials. Preserve all pre-existing staged/unstaged work and do not commit as part of this request.

Check local Markdown link targets, confirm key facts against their cited code/PDFs, and check the mirrored package version. Future HW4 implementation change sets each receive their own patch bump, with startup/log records reading the shared version constant.
