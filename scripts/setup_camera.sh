#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ros2_ws}"
SKIP_APT=false
BUILD_ONLY=false

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Installs camera dependencies, builds robot_vision_camera, and launches the camera node.

Options:
  --skip-apt      Skip 'sudo apt update' and package installation
  --build-only    Only install dependencies and build; do not launch the camera node
  -w, --ws DIR    Path to ROS 2 workspace (default: ~/ros2_ws)
  -h, --help      Show this help message
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-apt)
            SKIP_APT=true
            shift
            ;;
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        -w|--ws)
            WORKSPACE_DIR="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if [[ "$SKIP_APT" == false ]]; then
    echo "==> Updating apt and installing GStreamer & OpenCV dependencies..."
    sudo apt update
    sudo apt install -y \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libopencv-dev
fi

# Source ROS 2 installation (prefer jazzy, fallback to humble)
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    echo "==> Sourcing /opt/ros/jazzy/setup.bash..."
    source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    echo "==> Sourcing /opt/ros/humble/setup.bash..."
    source /opt/ros/humble/setup.bash
else
    echo "Error: Could not find ROS 2 setup.bash in /opt/ros/{jazzy,humble}" >&2
    exit 1
fi

echo "==> Building robot_vision_camera in ${WORKSPACE_DIR}..."
cd "$WORKSPACE_DIR"
colcon build --packages-select robot_vision_camera

echo "==> Sourcing workspace overlay..."
source install/setup.bash

if [[ "$BUILD_ONLY" == true ]]; then
    echo "==> Build complete (--build-only specified)."
    exit 0
fi

echo "==> Launching robot_vision_camera.launch.py..."
exec ros2 launch robot_vision_camera robot_vision_camera.launch.py
