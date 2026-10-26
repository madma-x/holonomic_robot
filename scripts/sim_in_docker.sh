#!/usr/bin/env bash
# Build and launch the table simulator inside your ROS 2 Jazzy container.
#
# From the Mac (repo root):
#   IMAGE=<your jazzy image> ./scripts/sim_in_docker.sh [launch args...]
# Already inside the container (repo mounted at $WS):
#   ./scripts/sim_in_docker.sh --inside [launch args...]
#
# RViz needs a display: with a VNC/noVNC desktop image (e.g. tiryoh/ros2-desktop-vnc:jazzy)
# open http://localhost:6080 and run this script from the desktop's terminal with --inside.
set -eo pipefail
WS="${WS:-/ws}"

if [ "$1" != "--inside" ]; then
  REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
  : "${IMAGE:?set IMAGE to your ROS 2 Jazzy docker image (docker images | grep jazzy)}"
  exec docker run -it --rm \
    -v "$REPO":"$WS" -w "$WS" \
    -p "${VNC_PORT:-6080}":80 \
    --shm-size=512m \
    "$IMAGE" bash -lc "WS=$WS $WS/scripts/sim_in_docker.sh --inside $*"
fi
shift

source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --symlink-install \
  --packages-select table_sim_interfaces table_sim holonomic_robot_description
source install/setup.bash
ros2 launch table_sim sim.launch.py "$@"
