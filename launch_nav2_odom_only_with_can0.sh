#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

run_privileged() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    if ! sudo -n "$@"; then
      echo "Error: passwordless sudo is not configured for CAN setup commands." >&2
      echo "Configure sudoers NOPASSWD for ip link commands, or run this launcher as root." >&2
      exit 1
    fi
  fi
}

if ! command -v ip >/dev/null 2>&1; then
  echo "Error: ip command not found (install iproute2)." >&2
  exit 1
fi

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

echo "Configuring can0 at 1 Mbps..."
if ip link show can0 >/dev/null 2>&1; then
  run_privileged ip link set can0 down || true
  run_privileged ip link set can0 type can bitrate 1000000
  run_privileged ip link set can0 up
else
  echo "can0 not found; attempting to create it..."
  run_privileged ip link add dev can0 type can bitrate 1000000
  run_privileged ip link set can0 up
fi

echo "Launching nav2_odom_only..."
ros2 launch robot_application full_system.launch.py "$@"
status=$?
if [ "$status" -ne 0 ]; then
  echo
  echo "Launch failed with exit code $status"
  read -r -p "Press Enter to close..."
fi

exit "$status"