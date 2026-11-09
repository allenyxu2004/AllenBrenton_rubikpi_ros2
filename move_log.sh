#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: bash move_log.sh <timestamp_square.jsonl|timestamp_octagon.jsonl>" >&2
    exit 1
fi

# Allow only our log filenames, with no paths, wildcards, or shell characters.
if [[ ! $1 =~ ^[0-9]+_(square|octagon)\.jsonl$ ]]; then
    echo "Invalid filename: expected a numeric timestamp followed by _square.jsonl or _octagon.jsonl." >&2
    exit 1
fi

if ! command -v uv >/dev/null 2>&1; then
    echo "Plotting requires uv; install it before running this script." >&2
    exit 1
fi

# Resolve imports from this script's location, regardless of the current folder.
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
destination='/Users/brenton/Desktop/cse276a/hw3'
mkdir -p "$destination"

if ! scp "276a:~/.ros/hw3_slam/$1" "$destination/"; then
    echo "Copy failed: check the filename and your SSH connection to 276a." >&2
    exit 1
fi

echo "Saved: $destination/$1"

if ! PYTHONPATH="$repo_root/robot_control${PYTHONPATH:+:$PYTHONPATH}" \
    uv run --no-project --with numpy --with matplotlib \
    python -m robot_control.slam.evaluate "$destination/$1"; then
    echo "Plotting failed; the copied log is still at $destination/$1" >&2
    exit 1
fi
