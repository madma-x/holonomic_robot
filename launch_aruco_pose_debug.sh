#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
elif [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "Error: no ROS setup found at /opt/ros/jazzy/setup.bash or /opt/ros/humble/setup.bash." >&2
  exit 1
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u
  source "$WS_DIR/install/setup.bash"
  set -u
else
  echo "Error: Workspace setup not found at $WS_DIR/install/setup.bash" >&2
  echo "Build first with: colcon build" >&2
  exit 1
fi

echo "Launching aruco_pose_debug..."
ros2 launch aruco_manager aruco_pose_debug.launch.py "$@"
