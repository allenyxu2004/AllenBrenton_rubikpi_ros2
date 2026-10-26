# Camera Setup Guide

## Automated Setup Script

You can run the setup, build, and camera launch using [`scripts/setup_camera.sh`](../../scripts/setup_camera.sh):

```bash
chmod +x scripts/setup_camera.sh
./scripts/setup_camera.sh
```

### Script Options

| Flag | Description |
| :--- | :--- |
| `--skip-apt` | Skip `sudo apt update` and dependency installation |
| `--build-only` | Install dependencies and build `robot_vision_camera` without launching |
| `-w`, `--ws DIR` | Specify custom ROS 2 workspace path (default: `~/ros2_ws`) |

```bash
# Build only (skip launching the node)
./scripts/setup_camera.sh --build-only

# Rebuild and launch without running apt again
./scripts/setup_camera.sh --skip-apt
```

---

## Manual Step-by-Step Setup

### 1. Install Dependencies

```bash
sudo apt update
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libopencv-dev
```

### 2. Build the Camera Package

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select robot_vision_camera
source install/setup.bash
```

### 3. Launch the Camera Node

```bash
ros2 launch robot_vision_camera robot_vision_camera.launch.py
```

*(Optional)* In a second shell, verify topics are publishing:

```bash
ros2 topic list
```
