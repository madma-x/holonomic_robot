#!/usr/bin/env bash
# One command for the sim+RViz+rosbag+2D-replay loop described in CLAUDE.md:
# builds and launches the sim (+panel, +adaptive_strategy) in the persistent
# table_sim_jazzy container with a rosbag recording, waits for you to watch
# it on the VNC desktop, then on ENTER stops everything, converts the bag
# and opens the 2D replay viewer right here on the Mac (no ROS needed for
# that part - see table_sim/replay_match.py).
#
# Usage (from the repo root, on the Mac):
#   scripts/run_sim_session.sh run                  # build+launch+record+replay, one shot
#   scripts/run_sim_session.sh stop                  # kill sim/strategy/recording in the container
#   scripts/run_sim_session.sh replay <bag_dir>      # convert+open an existing container-side bag
#
# Env overrides: TEAM=yellow OPPONENT=mirror OUR_ROBOT=cmd_vel RTF=1.0 NO_STRATEGY=1
set -euo pipefail

CONTAINER="${CONTAINER:-table_sim_jazzy}"
TEAM="${TEAM:-yellow}"
OPPONENT="${OPPONENT:-mirror}"
OUR_ROBOT="${OUR_ROBOT:-cmd_vel}"
RTF="${RTF:-1.0}"
HOST_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

dexec() { docker exec -u ubuntu "$CONTAINER" bash -lc "$1"; }
dexec_bg() { docker exec -u ubuntu -d "$CONTAINER" bash -lc "$1"; }

ensure_running() {
  if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != "true" ]; then
    echo "starting $CONTAINER..."
    docker start "$CONTAINER" >/dev/null
    sleep 2
  fi
}

container_repo_path() {
  docker inspect "$CONTAINER" --format '{{json .Mounts}}' | python3 -c "
import json, sys
mounts = json.load(sys.stdin)
host = '$HOST_REPO'
for m in mounts:
    src = m['Source'].rstrip('/')
    if host == src or host.startswith(src + '/'):
        print(m['Destination'])
        break
"
}

cmd_stop() {
  ensure_running
  echo "stopping rosbag recording, sim, strategy, rviz..."
  dexec "pkill -INT -f 'ros2 bag record' || true"
  sleep 1
  dexec "pgrep -af 'sim\\.launch\\.py|strategy\\.launch\\.py|sim_node|strategy_node|rviz2' \
    | awk '{print \$1}' | xargs -r kill -9 || true"
}

cmd_replay_bag() {
  local bag_dir="$1" repo="$2"
  [ -z "$repo" ] && repo="$(container_repo_path)"
  if [ -z "$repo" ]; then
    echo "could not find $HOST_REPO mounted in $CONTAINER - check: docker inspect $CONTAINER --format '{{json .Mounts}}'" >&2
    exit 1
  fi
  local json_name="$(basename "$bag_dir").json"
  echo "converting bag -> frames json (in container, needs rosbag2_py)..."
  dexec "cd $repo && source /opt/ros/jazzy/setup.bash && source install/setup.bash \
    && ros2 run table_sim bag2frames $bag_dir -o /tmp/$json_name"
  echo "copying $json_name to host..."
  docker cp "$CONTAINER:/tmp/$json_name" "/tmp/$json_name"
  echo "opening 2D replay viewer (host, no ROS/Docker needed for this part)..."
  PYTHONPATH="$HOST_REPO/src/table_sim" python3 -m table_sim.replay_match "/tmp/$json_name" \
    --field "$HOST_REPO/src/table_sim/config/field_2027.yaml" \
    --sim "$HOST_REPO/src/table_sim/config/sim.yaml"
}

cmd_replay() {
  local bag_dir="${1:?usage: run_sim_session.sh replay <container-side bag_dir>}"
  cmd_replay_bag "$bag_dir" ""
}

cmd_run() {
  ensure_running
  local repo
  repo="$(container_repo_path)"
  if [ -z "$repo" ]; then
    echo "could not find $HOST_REPO mounted in $CONTAINER - check: docker inspect $CONTAINER --format '{{json .Mounts}}'" >&2
    exit 1
  fi
  echo "repo mounted at $repo in $CONTAINER"
  local bag_dir="/tmp/table_sim_bags/$(date +%Y%m%d_%H%M%S)"

  echo "building (table_sim_interfaces table_sim adaptive_strategy motion_controller)..."
  dexec "cd $repo && source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --packages-select table_sim_interfaces table_sim adaptive_strategy motion_controller"

  echo "launching sim: rviz + panel + goal_driver + record -> $bag_dir ..."
  dexec_bg "cd $repo && source /opt/ros/jazzy/setup.bash && source install/setup.bash && DISPLAY=:1 \
    ros2 launch table_sim sim.launch.py team:=$TEAM opponent:=$OPPONENT our_robot:=$OUR_ROBOT \
    rviz:=true panel:=true goal_driver:=true record:=true bag_dir:=$bag_dir rtf:=$RTF \
    > /tmp/sim.log 2>&1"
  sleep 4

  if [ "${NO_STRATEGY:-0}" != "1" ]; then
    echo "launching adaptive_strategy..."
    dexec_bg "cd $repo && source /opt/ros/jazzy/setup.bash && source install/setup.bash \
      && ros2 launch adaptive_strategy strategy.launch.py team:=$TEAM > /tmp/strategy.log 2>&1"
  fi

  echo
  echo "recording to $bag_dir (inside $CONTAINER)."
  echo "watch it on the VNC desktop (RViz + match panel)."
  echo "press ENTER here once the match is over to stop everything and open the replay..."
  read -r

  cmd_stop
  sleep 1
  cmd_replay_bag "$bag_dir" "$repo"
}

case "${1:-run}" in
  run) cmd_run ;;
  stop) cmd_stop ;;
  replay) shift; cmd_replay "$@" ;;
  *) echo "usage: $0 [run|stop|replay <bag_dir>]" >&2; exit 1 ;;
esac
