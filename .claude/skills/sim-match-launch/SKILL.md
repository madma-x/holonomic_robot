---
name: sim-match-launch
description: Use this skill to launch a full Eurobot match in table_sim (real field_2027.yaml, pick stacks, drop zones, 85s scored clock) - watching adaptive_strategy or a scripted opponent play, or driving manually. For isolated opponent-avoidance testing on an empty table, use avoidance-rviz-test instead.
---

# Launching a match in table_sim

Three different ways to run a match, pick based on what you're checking.
Everything here runs inside the `table_sim_jazzy` container (see CLAUDE.md
for the general Docker workflow) unless noted as headless/host-side.

## 1. Fastest: headless, no Docker/ROS, no RViz

For iterating on `adaptive_strategy` logic itself - runs many matches
against every scripted opponent in seconds, on the Mac host directly:

```bash
PYTHONPATH=/Users/max/Documents/holonomic_robot/src/table_sim:/Users/max/Documents/holonomic_robot/src/adaptive_strategy \
  python3 -m table_sim.headless --brain adaptive_strategy.core:StrategyCore \
  --opponent all --runs 3
```

(Both packages need to be on `PYTHONPATH` - `table_sim` for the `-m
table_sim.headless` entry point itself, `adaptive_strategy` for the
`--brain adaptive_strategy.core:StrategyCore` import. Confirmed working
with just `--brain scripted:mirror` and only `table_sim` on the path too,
if you don't need the strategy module at all.)

Useful flags: `--opponent <name|all>` (`mirror greedy raider center_first
random blocker idle teleop`), `--runs N` (seeds per opponent), `--team
yellow|blue|both`, `--video out.mp4` / `--png out.png` (render the last run
per opponent - needs matplotlib), `--no-avoid` (opponent drives through,
stress-test), `--params <strategy.yaml>` (override weights), `-v` (verbose
per-tick). Also runnable as `ros2 run table_sim headless_match ...` inside
the container if you need ROS-adjacent paths resolved.

Use `--brain scripted:<behaviour>` instead of the strategy module to watch
two scripted bots play each other with zero strategy/Nav2 involved at all -
the quickest sanity check that the sim/scoring itself is working.

## 2. Full RViz session, simplest entry point

`scripts/run_sim_session.sh run` (on the Mac host) builds, launches
sim+RViz+panel+adaptive_strategy+rosbag recording in the container, waits
for you to watch it on the VNC desktop, and on ENTER converts the bag and
opens the 2D replay viewer locally.

**This blocks on an interactive ENTER keypress - do not run `... run`
directly as an agent, it will hang the turn.** Either tell the user to run
it themselves in their own terminal, or do the launch manually (below) and
use the script's `stop`/`replay <bag_dir>` subcommands instead, which don't
block.

Env overrides: `TEAM=yellow OPPONENT=mirror OUR_ROBOT=cmd_vel RTF=1.0
NO_STRATEGY=1 scripts/run_sim_session.sh run`.

## 3. Full RViz session, manual (agent-friendly, non-blocking)

Equivalent to what the script does, without the blocking read - and always
records a bag + saves logs so every run can be replayed/inspected
afterwards (see "After every run" below), not just when you remember to
ask for it. Build first:

```bash
docker exec -u ubuntu table_sim_jazzy bash -lc \
  'cd /ws && source /opt/ros/jazzy/setup.bash && \
   colcon build --symlink-install --packages-select \
   table_sim_interfaces table_sim adaptive_strategy motion_controller'
```

Pick one `BAG_DIR` per run and reuse it for every command below (launch,
log save, and replay all need the same path):

```bash
BAG_DIR=/tmp/table_sim_bags/$(date +%Y%m%d_%H%M%S)
```

Launch the sim with `record:=true` (do NOT pass `nav2:=true` here when also
launching `adaptive_strategy` below - it brings up its own Nav2 stack, see
its launch file's own warning: passing both launches Nav2 twice, duplicate
node names):

```bash
docker exec -u ubuntu -d table_sim_jazzy bash -lc \
  "cd /ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && DISPLAY=:1 \
   ros2 launch table_sim sim.launch.py \
     team:=yellow opponent:=mirror our_robot:=cmd_vel \
     rviz:=true panel:=true goal_driver:=true auto_start:=true rtf:=1.0 \
     record:=true bag_dir:=$BAG_DIR \
   > /tmp/sim.log 2>&1"
```

Then, if driving with `adaptive_strategy` (skip this for `our_robot:=
scripted:<behaviour>`, which needs no strategy/Nav2 at all):

```bash
docker exec -u ubuntu -d table_sim_jazzy bash -lc \
  'cd /ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   ros2 launch adaptive_strategy strategy.launch.py team:=yellow > /tmp/strategy.log 2>&1'
```

`our_robot` choices: `cmd_vel` (driven externally - by `goal_driver`'s
RViz "2D Goal Pose" clicks, by `adaptive_strategy`'s own `/cmd_vel`, or by
hand) or `scripted:<behaviour>` (table_sim drives BOTH robots itself, to
just watch a full match with nothing else running).

### map_server startup race (same root cause as avoidance-rviz-test)

`adaptive_strategy`'s Nav2 stack (via `nav2_sim.launch.py`, same file
`sim.launch.py nav2:=true` uses) can have `map_server` stick on
`unconfigured` if `lifecycle_manager_localization` calls its configure
transition before `/clock` is ticking reliably - every Nav2 goal then
fails silently with `"Costmap timed out waiting for update"`. Check after
launching `adaptive_strategy`:

```bash
docker exec -u ubuntu table_sim_jazzy bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
   ros2 service call /map_server/get_state lifecycle_msgs/srv/GetState "{}"'
```

If not `active`, drive it through by hand (see avoidance-rviz-test's
identical section for the two `change_state` calls).

### Stale processes from an earlier session

Same caveat as avoidance-rviz-test: `pkill -f sim.launch.py` only kills the
launch process, not every spawned node.

```bash
docker exec -u ubuntu table_sim_jazzy bash -lc \
  'ps aux | grep -E "sim.launch|strategy.launch|nav2_sim|sim_node|sim_panel|strategy_node|rviz2|controller_server|planner_server|map_server|lifecycle_manager|bt_navigator|behavior_server|waypoint_follower|odom_sim|robot_state_publisher|ros2 bag record" | grep -v grep'
# then: kill -9 <pids>
```

## Watching / checking the result

- VNC desktop: RViz (robots, table, markers) + the match panel window
  (score, clock, P/Y/B stack counts, pick/drop buttons, start/reset).
- `ros2 topic echo --full-length /sim/score --once` - full JSON score/state
  for both teams, same as avoidance-rviz-test's recipe (note
  `--full-length`, plain `echo` truncates).
- `ros2 action send_goal /face_tool table_sim_interfaces/action/FaceTool
  "{face: 0, mode: 0}"` to manually pick (mode 0) / drop (mode 1) if driving
  by hand.
- `ros2 service call /sim/reset std_srvs/srv/Trigger "{}"` then `/sim/start`
  to rerun without relaunching everything.

## After every run: save logs, stop, replay on the Mac

Do this after every run launched via section 3 (whether the match ran to
completion or you stopped it early) - don't wait to be asked. Keep using
the same `BAG_DIR` from the launch step.

1. Stop the recording first (lets rosbag flush/finalize cleanly), then kill
   the rest:
   ```bash
   docker exec -u ubuntu table_sim_jazzy bash -lc "pkill -INT -f 'ros2 bag record' || true"
   sleep 1
   docker exec -u ubuntu table_sim_jazzy bash -lc \
     "pgrep -af 'sim\\.launch\\.py|strategy\\.launch\\.py|sim_node|strategy_node|rviz2' | awk '{print \$1}' | xargs -r kill -9 || true"
   ```
2. Save the text logs to the host, alongside a timestamped folder (so
   nothing gets overwritten by the next run - `/tmp/sim.log`/`/tmp/
   strategy.log` inside the container are overwritten every launch):
   ```bash
   RUN_DIR="/tmp/table_sim_runs/$(basename "$BAG_DIR")"
   mkdir -p "$RUN_DIR"
   docker cp table_sim_jazzy:/tmp/sim.log "$RUN_DIR/sim.log" 2>/dev/null
   docker cp table_sim_jazzy:/tmp/strategy.log "$RUN_DIR/strategy.log" 2>/dev/null
   ```
3. Convert the bag to a frames JSON (in-container, needs `rosbag2_py`),
   copy it next to the logs, then open the 2D replay viewer on the Mac
   (host side, no ROS/Docker needed for this part):
   ```bash
   docker exec -u ubuntu table_sim_jazzy bash -lc \
     "cd /ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
      ros2 run table_sim bag2frames $BAG_DIR -o /tmp/match.json"
   docker cp table_sim_jazzy:/tmp/match.json "$RUN_DIR/match.json"
   PYTHONPATH=/Users/max/Documents/holonomic_robot/src/table_sim python3 -m table_sim.replay_match \
     "$RUN_DIR/match.json" \
     --field /Users/max/Documents/holonomic_robot/src/table_sim/config/field_2027.yaml \
     --sim /Users/max/Documents/holonomic_robot/src/table_sim/config/sim.yaml
   ```

`$RUN_DIR` ends up with everything from that one run in one place
(`sim.log`, `strategy.log`, `match.json`) - check the logs first for any
`WARN`/`ERROR` before digging into the replay. The replay viewer is the
only way to check a pick/drop's actual heading against what was asked for
- live viewing shows it happened, not whether the alignment was within
tolerance.
