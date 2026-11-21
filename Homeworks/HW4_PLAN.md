# HW4: Algorithms and implementation plan

Based on [the HW4 assignment](../Docs/hw4/CSE276A_26_HW4.pdf), due November 25, 2026.
These are the design notes and physical-run checklist. See the
[HW4 run instructions](../robot_control/README.md#hw4-known-map-navigation)
for the implemented planner, course configuration, and ROS launch.

## Goal and shared map

Drive from start to goal across a 10 ft × 10 ft workspace containing a central
obstacle measuring 1–2 ft on each side. Demonstrate a minimum-distance route
and a maximum-safety route.

Use a two-dimensional occupancy grid, with each cell marked free or blocked.
Store measured walls, obstacle boundaries, landmark positions, start, and goal
in a single world coordinate frame, using metres. About 5 cm per cell is a
reasonable initial resolution to evaluate.

Expand obstacles by the robot's bounding-circle radius plus an error margin,
and shrink the allowed workspace boundary by the same amount. This lets the
planner treat the robot's center as a point while accounting for chassis size,
pivot turns, localization error, and imperfect tracking.

Both planners should use the same map, footprint, baseline margin, localization,
and motion controller. Choose minimum distance rather than minimum time for the
first objective: HW4 permits either, and distance avoids modeling turn and
observation durations. Actual elapsed time can still be measured.

## Algorithm 1: A* for minimum distance

**Idea:** find the shortest collision-free route through the grid.

1. Start at the robot's start cell.
2. Consider neighboring free cells, including diagonal moves.
3. Prioritize candidates using distance already traveled plus straight-line
   distance remaining to the goal.
4. When the goal is removed from the search queue, reconstruct the path.

Horizontal and vertical moves cost one cell width; diagonal moves cost
`sqrt(2)` times that width. Prevent diagonal moves through blocked corners.

**Expected result:** a short route that may pass close to the expanded obstacle.
It is shortest within the grid and movement rules, rather than necessarily the
exact shortest continuous-space route.

**Why it fits:** A* is straightforward to implement and visualize for a small,
known workspace, and its distance objective directly matches the assignment.

## Algorithm 2: Widest-path search for maximum safety

**Idea:** maximize the minimum clearance anywhere along the route.

First calculate a clearance map: each free cell stores its distance to the
nearest expanded obstacle or forbidden workspace boundary. This represents
remaining clearance beyond the footprint and baseline margin already applied.

1. Weight each grid edge by its minimum clearance along the entire segment.
2. Build a maximum spanning tree using those edge weights.
3. Find the start-to-goal path in that tree. The smallest edge weight along
   that path is the best achievable minimum clearance.

Then run A* restricted to edges meeting that optimal clearance threshold. This
selects the shortest route among those achieving the maximum safety score.

**Expected result:** a route with more room at its tightest passage, potentially
at the cost of extra distance.

**Why it fits:** “maximum safety” has a precise, measurable definition, and the
search objective differs clearly from minimum-distance A*. This optimizes
geometric clearance, not every possible source of execution risk.

**Limitation:** start and goal clearance count toward the minimum. If an endpoint
is the tightest location, both planners may achieve the same score or path.
Inspect the layout before assuming the two routes will look different.

## Work breakdown

### 1. Measure and configure the environment

- [ ] Set up the specified 10 ft × 10 ft area; HW4 does not repeat HW3's 8 ft allowance.
- [ ] Measure obstacle boundaries, landmark positions, robot dimensions, start, and goal.
- [ ] Define world axes, origin, and initial robot heading.
- [ ] Put measurements and planning parameters in a configuration file.

**Done when:** a top-down plot matches the measured physical layout.

### 2. Build the shared map

- [ ] Convert geometry into a grid, marking blocked cells conservatively.
- [ ] Apply the footprint and baseline margin to obstacles and walls.
- [ ] Compute clearance, accounting for occupied cell area rather than treating
  obstacle cells as isolated points.
- [ ] Reject blocked or out-of-bounds endpoints and handle unreachable goals.

**Done when:** plots show the original obstacle, inflated boundaries, and clearance map.

### 3. Implement and compare planners offline

- [ ] Implement A* with distance costs and corner-cutting prevention.
- [ ] Implement widest-path search and its thresholded A* tie-break.
- [ ] Plot both paths on the same map.
- [ ] Calculate planned distance and minimum clearance for each path.
- [ ] Check an open map, a narrow passage, and an unreachable goal.
- [ ] Verify all path transitions are collision-free, including diagonals.

**Done when:** both planners return valid paths consistent with their objectives.
No robot access is needed for this step.

### 4. Establish localization in the measured world frame

- [ ] Reuse HW3 AprilTag observation processing and calibrated motion prediction.
- [ ] Use measured landmark coordinates for localization, as HW4 permits.
- [ ] Initialize robot position and heading consistently with the planning map.
- [ ] Check estimated pose at measured locations and during turns.
- [ ] Define when stale observations or excessive uncertainty should stop motion.

**Done when:** pose accuracy supports the chosen clearance margin. Do not assume
the current HW3 SLAM node already supports fixed-map localization.

### 5. Connect paths to the motion controller

- [ ] Adapt the waypoint policy in
  `robot_control/robot_control/slam/trajectory.py` to planner output.
- [ ] Start with the existing short turn/forward pulses and stop-and-observe behavior.
- [ ] If simplifying paths, collision-check each replacement segment and preserve
  the safety planner's clearance threshold.
- [ ] Publish `motor_commands` continuously at about 20 Hz while moving to satisfy
  the motor watchdog; publish zeros on completion or failure.
- [ ] Stop if tracking error threatens clearance; recover localization and replan
  before continuing.

**Done when:** the robot reaches the goal reliably without clipping obstacles or walls.
Localization and physical tracking will likely require the most debugging.

### 6. Run and document experiments

- [ ] Keep layout, start, goal, localization, and controller identical across planners.
- [ ] Log estimated trajectory, elapsed time, completion status, and tracking error.
- [ ] Compare planned path length and clearance with estimated executed values.
- [ ] Label clearance calculated from estimated poses as an estimate; assess actual
  clearance with external measurements or video where practical.
- [ ] Repeat runs if possible to check consistency.
- [ ] Record a clear demonstration video for each objective.

**Done when:** results explain the distance/clearance trade-off and execution limitations.

### 7. Prepare the submission

- [ ] Describe architecture, representation, algorithms, and objective definitions.
- [ ] Include the measured layout, clearance visualization, planned paths, and results.
- [ ] Explain why each method fits its objective and discuss limitations.
- [ ] Include links to two run videos and a ZIP containing the complete code.

## First milestone

Generate both paths on the laptop before integrating motion. Then validate
localization and tracking with generous clearance before attempting the shortest
route. Reuse HW3 infrastructure while prioritizing reliable physical execution.
