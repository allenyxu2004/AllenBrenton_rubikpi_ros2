# HW4 chat history

This file records the HW4 planning and robot-trial decisions from the available conversation history. It is a chronological summary rather than a raw platform export; earlier turns were not available as a complete verbatim transcript when this file was prepared.

## Planning and implementation

- Asked to review `docs/hw4` obstacle documents, compare them with the repository implementation, and plan safe and fast obstacle navigation. Clarifying questions about the environment were requested.
- Selected a Voronoi planner for the safe route and a visibility graph for the fast route.
- Defined the workspace as 10 ft × 10 ft. Perimeter AprilTags: tag 6 at (1,0), tag 5 at (9,0), tag 4 at (10,1), tag 3 at (10,9), tag 2 at (9,10), tag 0 at (1,10), tag 8 at (0,9), and later tag 7 at (0,1). Tags should be at the corners/edges as specified.
- Defined the central obstacle as a 14 in × 14 in square. Its four tags are tag 24 at the bottom center, then clockwise tags 23, 22, and 21.
- Set the start to (2,8) ft, heading toward decreasing y, and the goal to (8,2) ft.
- Requested a layout diagram in `implementation.md`. The planning diagrams were updated first, and implementation proceeded after the plan was approved.
- Implementation constraints: external libraries may be used for A*; do not use Shapely geometric operations. Changes should be split into atomic commits and pushed.

## Robot execution and estimator changes

- Requested deployment to `ubuntu@ubuntu.local`, pulling the latest repository inside `ros2_ws`, and running the robot safety path. The user confirmed the configured physical setup and authorized motion.
- The first safety run stopped after its first tick. The logs were analyzed. The user clarified that tag 7 is correctly placed and can be difficult to see when the robot is parallel to it; the user asked not to force a stop in such cases and asked for behavior similar to HW2/HW3 without extra safety measures.
- Changes were pushed and pulled onto the robot; the robot was started on request.
- Requested saving the run logs and generating a diagram in the HW4 folder. Then asked why the plotted start appeared to travel backward. The log interpretation was reviewed, and bad/ misleading entries were filtered for clearer plots.
- Asked whether the environment knows the tag locations and uses them as ground truth. The distinction between configured map landmarks and the robot's estimated pose was explained; this motivated anchoring the known starting pose.
- Requested anchoring the starting pose. The startup anchor uses the known initial position and heading while stationary, holds x/y through initial pivots, and releases after the first translation prediction.
- Requested pushing the anchoring changes, updating the robot, rerunning the safety test, and saving its results in the same HW4 location. The user confirmed the robot had been reset to (2,8) ft facing −y and authorized the full trial.

## Anchored safety run

- Anchored run used package version 0.1.42 on the robot. It completed in 103.14986 s with 136 motion pulses, estimated final error 3.3419 cm, no startup drift, and no faults or replans.
- Logs, summary, trajectory and motion-segment plots, and CSV data were saved under `Docs/hw4`, with filenames beginning `hw4_safety_20261121T165651` and console log `safety_v0142_20261122T005649.console.log`.
- The older, unanchored v0.1.40 safety run was also retained under `Docs/hw4` for comparison.

## Anchored fast run

- Requested running the fast version with the same anchoring process, saving logs, and generating matching images. The user confirmed the robot was reset to (2,8) ft facing −y and the route was clear.
- The visibility-graph distance objective ran on the robot using version 0.1.42. It completed in 86.50008 s: 16.65 s faster than the anchored safety run. It used 113 pulses, had zero faults and replans, planned a 2.892346 m center path with 0.204082 m planned body clearance, and ended with estimated error of 3.3161 cm.
- The recorded estimate trajectory was 3.33287 m; commanded-motion model trajectory was 3.11242 m. Estimate-derived clearance was 0.153567 m. Startup drift was zero, and the anchor released at 3.76884 s after the first forward command at 3.71836 s.
- Saved artifacts in `Docs/hw4`:
  - `hw4_distance_20261121T170234.710691-0800.jsonl`
  - `distance_v0142_JbR3A9fE.console.log`
  - `hw4_distance_20261121T170234_summary.json`
  - `hw4_distance_20261121T170234_trajectory.png` and `.svg`
  - `hw4_distance_20261121T170234_motion_segments.png` and `.svg`
  - `hw4_distance_20261121T170234_trajectory.csv`
  - `hw4_distance_20261121T170234_motion_segments.csv`
  - `hw4_distance_20261121T170234_localization_corrections.csv`
- The logs were copied after the run completed and checked against the robot copies. The generated plots were validated, including visual inspection of the motion-segment plot. The robot was stopped after the trial.

## Repository guidance recorded during the work

- For each future change set, increment the patch version in `robot_control/robot_control/version.py` and mirror it in `robot_control/package.xml` before committing. Bump once per change set; leave vendored package versions unchanged.
- Use conventional commit subjects (`feat:`, `fix:`, etc.), subject only, with no AI attribution. Keep commits atomic.
- The robot workspace is ROS 2 Humble by project assumptions, although the deployed board reported Jazzy during the work. Controllers publish `motor_commands` continuously and must not open the serial port themselves.
- The robot start anchor trusts the confirmed placement exactly. Motion trials require the robot to be physically at the specified start pose with a clear route.
