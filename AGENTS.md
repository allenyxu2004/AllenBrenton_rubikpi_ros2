# AGENTS.md

ROS 2 starter code for UCSD CSE 276A (Intro to Robotics). It runs on a RubikPi (Qualcomm) board with an OV5647 camera. A differential-drive base is driven over USB serial by a Waveshare-style motor/IMU driver board. The ROS distro is assumed to be Humble.

## Layout

| Dir | ROS package name | Build | Purpose |
|---|---|---|---|
| `robot_control/` | `robot_control` | ament_python | Motor + IMU serial bridge, keyboard teleop |
| `robot_vision/` | **`robot_vision_camera`** (differs from dir name) | ament_cmake, C++17 | GStreamer camera publisher + foxglove_bridge |
| `apriltag_ros/` | `apriltag_ros` | ament_cmake, C++14, `-Werror` | Vendored christianrauch/apriltag_ros v3.3.0 |

## Build & run

This repo is meant to sit inside a colcon workspace's `src/` on the robot. A path baked into the code suggests `/home/ubuntu/ros2_ws/`.

```bash
colcon build --symlink-install
source install/setup.bash

ros2 launch robot_control robot_teleop_launch.py          # motor_control + keyboard_control
ros2 launch robot_vision_camera robot_vision_camera.launch.py  # camera + foxglove_bridge :8765
ros2 launch apriltag_ros apriltag_launch.py               # tag detection on /camera/image_raw
```

Packages needed on the board but not declared in any `package.xml`:
- pip: `pyserial`, `pynput`, `numpy`
- `ros-humble-foxglove-bridge`
- the Qualcomm `qtiqmmfsrc` GStreamer plugin

The user also needs to be in the `dialout` group to open `/dev/ttyUSB0`.

## robot_control

### Interface

- `motor_control` (node `motor_controller_node`)
  - Subscribes to `motor_commands` (`std_msgs/Float32MultiArray`). `data = [L, R]` are the left and right wheel-side speeds.
  - Each value is clipped to `[-0.5, 0.5]`. The units are whatever the firmware defines; they have not been calibrated.
  - Publishes `imu_data` (`sensor_msgs/Imu`) with `frame_id` `imu_link`.
- Serial protocol: newline-terminated JSON on `/dev/ttyUSB0` at 115200 baud. The port is hardcoded.
  - Motor command: `{"T":1,"L":..,"R":..}`, sent at 20 Hz.
  - IMU request: `{"T":126}`, sent at 50 Hz.
  - IMU reply: `T==1002`, carrying `r,p,y` (degrees) and `gx..az`.
- **Safety watchdog:** if no `motor_commands` arrives for 0.15 s, the motors are zeroed. Any controller must publish continuously, at about 20 Hz.
- `keyboard_control` (node `keyboard_controller_node`) publishes `motor_commands` at 20 Hz from `pynput` key input:

  | Key | Action | L | R |
  |---|---|---|---|
  | W | forward | 0.3 | 0.3 |
  | S | back | -0.3 | -0.3 |
  | A | pivot left | -0.5 | 0.5 |
  | D | pivot right | 0.5 | -0.5 |
  | X | stop | 0 | 0 |
  | ESC | quit | | |

  `pynput` needs a display session; it fails over plain SSH. It also suppresses keyboard input system-wide while it runs.
- `teleop_example.py` is a standalone script with no ROS. It writes the raw serial protocol directly. Never run it alongside `motor_control`, because both open the same port.

### Missing pieces (intended student work)

- `setup.py` declares a `velocity_control` entry point, but `robot_control/velocity_control.py` does not exist. Running it fails until it is written.
- The package has:
  - no `cmd_vel`/Twist interface
  - no odometry
  - no wheel radius or track width
- Converting (v, ω) to L/R needs measured robot geometry and a calibration of the L/R units.

### Known quirks

- The IMU gyro is passed through without conversion, so it may be in deg/s instead of rad/s.
- Acceleration is scaled assuming the board reports mg.
- If the serial port fails to open, the node keeps running and only logs a warning.
- The ament lint tests (flake8, pep257, copyright) likely fail on the current code.

## robot_vision (`robot_vision_camera`)

### Capture and topics

- Single node `robot_vision_camera`, executable `robot_vision_camera_node`.
- GStreamer pipeline: `qtiqmmfsrc camera=<id>` → NV12 → `videoconvert`/`videobalance` → appsink. There is no `/dev/video*` device.
- Defaults: 1280x720 at 10 fps.
- Topics:
  - `/camera/image_raw` (`sensor_msgs/Image`)
  - `/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`, JPEG)
  - `/camera/camera_info` (`sensor_msgs/CameraInfo`)
- `frame_id` is hardcoded to `camera_frame`.

### Calibration

- Intrinsics come from `config/camera_parameter.yaml`, which uses OpenCV FileStorage format, not ROS camera_info format.
- The commit labels this calibration "untuned". Recalibrate before trusting tag poses.
- The node's built-in default for `camera_parameter_path` is a hardcoded absolute path. The launch file overrides it with the installed share path, so always start the node through the launch file.
- `image_rectify` (default false) undistorts the image. The published `CameraInfo` still describes the distorted camera when rectify is on.

### Known quirks

- **Color channels:** the output format is BGR, but the message is labelled `rgb8`, so R and B are swapped in the raw image. Grayscale and AprilTag detection are unaffected.
- The launch arg `max_threads` is passed in but never used by the node.
- Ctrl-C can hang, because the pipeline thread stays blocked waiting on the GStreamer bus.
- Some launch args are strings; this works in practice on Humble.

## apriltag_ros

### Usage

- The C++ code is unmodified upstream. Course-specific changes are confined to `cfg/tags_36h11.yaml` and `launch/apriltag_launch.py`.
- `apriltag_launch.py` runs the `apriltag_node` executable and remaps:
  - `image_rect` → `/camera/image_raw`
  - `camera_info` → `/camera/camera_info`

  Override these with the launch args `image_topic`, `camera_info_topic` and `config_file`.
- Outputs:
  - `detections` (`apriltag_msgs/AprilTagDetectionArray`)
  - a TF from the image `frame_id` (`camera_frame`) to `tag_<id>`
- Config: family 36h11 with 0.165 m tags.
  - Only ids 0 and 1 are kept, as frames `tag_0` and `tag_1`. Because `tag.frames` is set, all other ids are dropped from both the detections and the TF.
  - Edit `tag.ids`, `tag.frames` and `tag.sizes` together; their lengths must match.
- Pose: `pnp` by default; `homography` is the alternative. It uses the `P` matrix from CameraInfo and assumes zero distortion, so it expects a rectified image.
- `camera_36h11.launch.yml` is the upstream example and targets `camera_ros`/`image_proc`. Don't use it with `robot_vision`.

### Code style

- C++ code here is formatted with `apriltag_ros/.clang-format` (4-space indent, `if(`, no column limit).
- The build uses `-Werror`, so new warnings break it.

## Conventions

- Commits: conventional format (`feat:`, `fix:`, …), subject line only, no AI attribution.
- Don't hardcode new absolute paths; use `get_package_share_directory` or launch substitutions.
- New controllers should publish `motor_commands`, not open the serial port.
- When adding executables or launch/config files, update `setup.py` `entry_points`/`data_files` (Python) or the CMake `install()` rules (C++).
