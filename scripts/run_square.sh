#!/usr/bin/env bash
# Run from a terminal with ROS and the colcon workspace sourced.
set -euo pipefail

usage() {
    echo "Usage: bash scripts/run_square.sh [SIDE_METRES [LAPS]]"
    echo "Defaults: SIDE_METRES=0.9, LAPS=1. Starts autonomous motion."
}

if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    usage
    exit 0
fi
if (( $# > 2 )); then
    usage >&2
    exit 1
fi

side=${1:-0.9}
laps=${2:-1}
if [[ ! $side =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
    ! awk -v side="$side" 'BEGIN { exit !(side > 0) }'; then
    echo "Side must be a positive number in metres." >&2
    exit 1
fi
if [[ ! $laps =~ ^[1-9][0-9]*$ ]]; then
    echo "Laps must be a positive integer." >&2
    exit 1
fi
if ! command -v ros2 >/dev/null 2>&1; then
    echo "Source ROS and your colcon workspace's install/setup.bash first." >&2
    exit 1
fi

echo "Starting square: side=${side} m, laps=${laps}. Ctrl-C stops the run."
exec ros2 launch robot_control hw3_slam.launch.py \
    drive:=true shape:=square "side_m:=$side" "laps:=$laps"
