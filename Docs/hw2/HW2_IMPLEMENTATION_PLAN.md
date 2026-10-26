# CSE 276A Homework 2: Visual Feedback Closed-Loop Navigation Plan

**Due Date:** Thursday, 28 October 2026, before midnight

---

## 1. Assignment Overview & Requirements

The goal of Homework 2 is to upgrade the HW1 open-loop waypoint controller (`../../robot_control/robot_control/velocity_control.py`) into a **closed-loop controller** using visual feedback from the onboard camera and known landmarks in the environment.

### Required Waypoint Trajectory
The robot must navigate through the following 4 waypoints in world coordinates $(x, y, \theta)$:
1. $(0, 0, 0)$ — Start pose
2. $(1, 0, 0)$
3. $(1, 2, \pi)$
4. $(0, 0, 0)$ — Return to start

*Note: Units must be at least $0.5\text{ m}$ per unit (e.g., $1\text{ unit} = 0.5\text{ m}$ or $1.0\text{ m}$). Physical markers must be placed on the ground at each waypoint for video verification.*

### Landmark Options
- **Option 1 (Standard):** Use AprilTags (`apriltag_ros`, already vendored in this repo) as artificial landmarks at known world poses.
- **Option 2 (+20% Extra Credit):** Build a custom object recognition package to detect natural landmarks (e.g., TV monitors, coke cans) and estimate relative pose.

---

## 2. Step-by-Step Implementation Plan for This Repository

### Step 1: Camera Calibration & Image Rectification (`robot_vision`)
The RubikPi OV5647 camera has significant optical distortion, and `apriltag_ros` expects an undistorted/rectified image (`P` matrix from `CameraInfo` assuming zero distortion).
1. **Calibrate the camera:**
   - Capture calibration images of a checkerboard on the RubikPi using `robot_vision_camera` or ROS 2 `camera_calibration`.
   - Compute accurate `camera_matrix`, `distortion_coefficients`, `rectification_matrix`, and `projection_matrix`.
2. **Update configuration:**
   - Replace the untuned parameters in `../../robot_vision/config/camera_parameter.yaml`.
3. **Enable rectification:**
   - In `../../robot_vision/launch/robot_vision_camera.launch.py`, set `image_rectify` default to `'true'` (or pass `image_rectify:=true` at launch).
   - Check `../../robot_vision/src/robot_vision_camera.cpp` to verify that when `image_rectify` is enabled, the `CameraInfo` projection matrix `P` matches the rectified image intrinsics used by `apriltag_ros`.

### Step 2: Landmark Setup & AprilTag Configuration (`apriltag_ros`)
1. **Design the Landmark Map:**
   - Place AprilTags (36h11 family) around the workspace perimeter so that at least one tag is in the camera's field of view along the path $(0,0,0) \to (1,0,0) \to (1,2,\pi) \to (0,0,0)$.
   - Measure each tag's side length (in meters) and exact 3D pose $(x, y, z, \text{roll}, \text{pitch}, \text{yaw})$ in the **World Frame**.
2. **Update `../../apriltag_ros/cfg/tags_36h11.yaml`:**
   - Currently, only `ids: [0, 1]` and `frames: [tag_0, tag_1]` with size `0.165` m are configured (all other tag IDs are ignored when `tag.frames` is populated).
   - Add all tag IDs, frame names (`tag_0`, `tag_1`, `tag_2`, ...), and measured tag sizes to `../../apriltag_ros/cfg/tags_36h11.yaml`.
3. **Create a Landmark Map Config (`robot_control/config/landmarks.yaml`):**
   - Store the known world pose $T_{\text{world}\leftarrow\text{tag}_i}$ for each `tag_i` so the localization node can compute the robot pose in the world frame.

### Step 3: Coordinate Frames & Pose Estimation (`robot_control`)
Define and implement the coordinate transformations required by the assignment:
1. **Coordinate Frames to Define:**
   - **World Frame (`world`):** Origin at $(0, 0, 0)$ start waypoint, $+X$ forward along the initial heading, $+Y$ left, $+Z$ up.
   - **Vehicle Frame (`base_link`):** Origin at the differential-drive wheel axle midpoint, $+X$ forward, $+Y$ left, $+Z$ up.
   - **Sensor Frame (`camera_frame`):** Published by `robot_vision_camera` and used by `apriltag_ros` (`+Z` optical axis forward out of the lens, $+X$ right, $+Y$ down).Measure the fixed extrinsic transform $T_{\text{vehicle}\leftarrow\text{camera}}$ on the RubikPi chassis.
2. **Visual Pose Estimator (`robot_control/robot_control/localization.py` or inside controller):**
   - Subscribe to `/tf` (or use `tf2_ros.Buffer` / `TransformListener`) to read the transform $T_{\text{camera}\leftarrow\text{tag}_i}$ published by `apriltag_node`.
   - Compute the vehicle pose in the world frame from each visible tag:
     $$T_{\text{world}\leftarrow\text{vehicle}} = T_{\text{world}\leftarrow\text{tag}_i} \cdot \left(T_{\text{camera}\leftarrow\text{tag}_i}\right)^{-1} \cdot \left(T_{\text{vehicle}\leftarrow\text{camera}}\right)^{-1}$$
   - Extract the 2D pose $(x_{\text{meas}}, y_{\text{meas}}, \theta_{\text{meas}})$ in the world frame.
3. **Noise Filtering & Missing-Landmark Handling (Dead-Reckoning Fusion):**
   - **When landmarks ARE visible:** Filter noisy PnP pose estimates (e.g., reject high decision-margin / distant outliers, average across multiple visible tags weighted by distance, and blend with dead-reckoning using a complementary filter, low-pass filter, or Extended Kalman Filter).
   - **When NO landmarks are visible:** Fall back smoothly to the dead-reckoning model (`EstimatedPose.integrate(speed, turn_rate, dt)` in `../../robot_control/robot_control/kinematic_control.py`, optionally aided by `imu_data` yaw rate from `motor_control.py`). When a tag re-enters the field of view, update/correct the pose estimate.

### Step 4: Closed-Loop Waypoint Controller (`robot_control`)
Update `../../robot_control/robot_control/velocity_control.py` (or create a dedicated `closed_loop_control.py` entry point):
1. **Update Waypoints File (`../../robot_control/config/waypoints.txt`):**
   - Set the HW2 waypoint sequence (scaled by your chosen unit length $U \ge 0.5\text{ m}$):
     ```text
     0, 0, 0
     1, 0, 0
     1, 2, 3.14159265
     0, 0, 0
     ```
2. **Closed-Loop Feedback Loop (20 Hz):**
   - Replace the purely open-loop `EstimatedPose` integration loop in `velocity_control.py` with a continuous 20 Hz control loop (satisfying the 0.15 s watchdog in `motor_control.py`).
   - At each step, compute position error $(\Delta x, \Delta y) = (x_{\text{goal}} - \hat{x}, y_{\text{goal}} - \hat{y})$, distance $\rho = \sqrt{\Delta x^2 + \Delta y^2}$, and heading error $\alpha = \text{normalize\_angle}(\text{atan2}(\Delta y, \Delta x) - \hat{\theta})$.
   - Drive toward the waypoint using feedback (either proportional steering $[L, R]$ or closed-loop rotate-then-drive segments that continuously re-evaluate $\hat{x}, \hat{y}, \hat{\theta}$ from visual feedback).
   - Once within `POSITION_TOLERANCE` of $(x_{\text{goal}}, y_{\text{goal}})$, rotate in closed-loop until $|\text{normalize\_angle}(\theta_{\text{goal}} - \hat{\theta})| \le \text{HEADING\_TOLERANCE}$.

### Step 5: Data Logging & Trajectory Plotting (`robot_control` / `scripts`)
The report requires quantitative metrics and a 2D trajectory plot generated from your algorithm's estimated positions:
1. **Runtime Logger:**
   - Log $(\text{timestamp}, \hat{x}, \hat{y}, \hat{\theta}, \text{visible\_tags})$ every $0.5\text{ s}$ (or $1.0\text{ s}$) to a CSV file during the run.
   - Accumulate total moving distance (translation + rotation) across the run.
   - At each of the 4 waypoints, pause and record the estimated pose vs. target waypoint (and measure physical ground-truth error against the floor marker for the report).
2. **Plotting Script (`scripts/plot_trajectory.py`):**
   - Read the logged CSV and plot the 2D trajectory dots $(x, y)$ sampled every $0.5\text{ s}$ or $1.0\text{ s}$ alongside the ground-truth waypoints and landmark positions.

### Step 6: Unified Launch & Package Configuration
1. **Update `../../robot_control/setup.py`:**
   - Ensure any new config files (e.g., `config/landmarks.yaml`, `config/hw2_waypoints.txt`) are listed in `data_files`.
   - Register any new console script entry points in `entry_points['console_scripts']`.
2. **Create a HW2 Launch File (`robot_control/launch/hw2_closed_loop_launch.py`):**
   - Launch `motor_control`, `robot_vision_camera.launch.py` (with `image_rectify:=true`), `apriltag_launch.py`, and the closed-loop waypoint controller node.

---

## 3. Deliverables Checklist (`hw2_your_name.zip`)

- [ ] **Report (~3 pages, PDF):**
  - [ ] **(a)** Closed-loop controller block diagram + description of each node and edge messages.
  - [ ] **(b)** Landmark description, placement map, and justification for placement.
  - [ ] **(c)** Pose estimation math (coordinate transforms) and noise-filtering justification.
  - [ ] **(d)** Handling of periods with no visible landmarks in the camera FOV.
  - [ ] **(e)** *(If Option 2 custom detector)* Motivation, algorithm details, robustness, and accuracy.
  - [ ] **(f)** Analysis of motion smoothness.
  - [ ] **(g)** Total moving distance (including rotation) in chosen units ($\ge 0.5\text{ m}$).
  - [ ] **(h)** Localization error (translation and rotation) recorded at each of the 4 waypoints.
  - [ ] **(i)** 2D trajectory plot showing estimated $(x, y)$ dots every $0.5\text{ s}$ or $1.0\text{ s}$.
  - [ ] **(j)** Reflection on overall system performance.
  - [ ] Coordinate frame definitions (sensor, vehicle, world) and start pose.
  - [ ] Unlisted YouTube video link.
- [ ] **Video (Unlisted YouTube):**
  - [ ] Shows the robot driving through $(0,0,0) \to (1,0,0) \to (1,2,\pi) \to (0,0,0)$ with visible ground markers at each waypoint.
- [ ] **Code:**
  - [ ] All related ROS 2 packages in the workspace (`robot_control`, `robot_vision`, `apriltag_ros`).
