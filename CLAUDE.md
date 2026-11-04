# CLAUDE.md – holonomic_robot

ROS 2 workspace of our Eurobot / Coupe de France robot (season 2027,
"The legend of Camelot"). Read this first; it is the shared context between
Max and Claude.

## How to work here

- **ROS 2 is NOT installed on the Mac.** Everything ROS (build, run, `ros2 ...`)
  happens in the Docker container **`table_sim_jazzy`** (ROS 2 Jazzy, user
  `ubuntu`, VNC desktop where RViz and the sim panel are shown).
  - Run commands with: `docker exec -u ubuntu table_sim_jazzy bash -lc '<cmd>'`
  - Always source first: `source /opt/ros/jazzy/setup.bash && source install/setup.bash`
  - Find where this repo is mounted: `docker inspect table_sim_jazzy --format '{{json .Mounts}}'`
    and `cd` there before `colcon build`.
  - If the container is stopped: `docker start table_sim_jazzy`.
- Edit files on the Mac (the repo is mounted in the container); build in the container:
  ```bash
  colcon build --symlink-install --packages-select table_sim_interfaces table_sim holonomic_robot_description
  ```
- Long-running launches (sim, RViz): start them in the background inside the
  container (`nohup ... > /tmp/sim.log 2>&1 &`) and read the log, so your
  shell is not blocked. Kill with `pkill -f sim_node` etc.
- The real robot stack (`robot_application`, `can_interface`, `aruco_*`,
  `motion_controller`, …) needs hardware (CAN, serial, camera). Do not launch
  it in the container and do not change it unless asked.
- Git: work on branch **`feature/table-sim-2027`**. Commit only when Max asks.
  Don't commit `__pycache__/`, `build/`, `install/`, `log/`, `.claude/`.
- Max validates step by step: show results (what you ran, what you saw)
  before moving on to the next feature.

## Robot

- Holonomic, 3 omni wheels, **equilateral triangle, 340 mm per face**.
- Each face holds **3 boxes** (as a stack). Faces assumed at 0°, 120°, 240°
  from +x of `base_footprint` (face 0 = +x) – **to confirm** on the real robot.
- 2D lidar SL C1 **on a mast**: only sees the other robot (its mast, min 7 cm
  dia.), ~2.8 m (computed from angular resolution needing >=2 beam hits).
- Camera on face 0 (front), 160° HFOV, 720p global shutter, 15 fps: reads
  8×8 cm ArUco tags on all 4 box faces, ~2.5 m max range (computed for a
  reliable decode) - used to check pick-stack occupancy while driving by.
- Frames: `map -> odom -> base_footprint -> base_link -> base_scan`
  (`base_scan` yawed by π in `holonomic_robot_description/urdf/holonomic_robot.urdf`).
- Motion command: `/cmd_vel` (`geometry_msgs/Twist`, body frame: vx, vy, wz).

## Game (Eurobot 2027)

- Table 3000 × 2000 mm. Map frame: origin at the yellow-end corner,
  x along the 3 m length (yellow x≈0, blue x≈3), y along the 2 m width.
- Match **85 s**, two robots. Yellow starts in the square x 0–0.5, y 0.75–1.25;
  blue mirrored (x → 3 − x). **The robot must end inside its zone.**
- Boxes **320 × 110 × 110 mm**.
- **10 pick stacks of 3 boxes** (IDs **P1–P10**): 4 along each long edge, 2 in
  the centre at x = 1.5 m (y 0.5 and 1.5).
- **5 drop zones per team** (IDs **Y1–Y5**, **B1–B5**), **max 3 boxes per
  zone**; best is a stack of 3, as in the pick zones. Diagonal zones centred at
  x 0.55, y 0.70 and x 0.55, y 1.30 (mirrored for blue).
- Points in `src/table_sim/config/field_2027.yaml` are **placeholders**:
  official rules are "Règlement 2027" on coupederobotique.fr / eurobot.org.
- Strategy goal (later): choose where to pick depending on where the opponent
  is and what it takes, avoid it reactively, always get back home in time.

## Simulator: `src/table_sim` + `src/table_sim_interfaces`

Light 2D simulation displayed in RViz (no Gazebo, no box physics on purpose:
stacks and piles are single objects with a box count; only robots collide).

| file | role |
|---|---|
| `config/field_2027.yaml` | table layout: stacks, zones, start zone, box size, scoring |
| `config/sim.yaml` | robot size/limits, gripper timings, lidar, opponent |
| `table_sim/world.py` | world model, match clock, collisions, lidar raycast, camera FOV check, score (no ROS) |
| `table_sim/field.py` | layout loader, IDs, pick/drop robot poses (no ROS) |
| `table_sim/opponents.py` | scripted opponents: mirror, greedy, raider, center_first, random, blocker, idle, teleop |
| `table_sim/sim_node.py` | ROS node: `/cmd_vel` in; `/odom`, TF, `/scan`, `/camera/detections`, `/joint_states`, `/clock` out; action `/face_tool`; services `/sim/start`, `/sim/reset`, `/game/start_match`; markers `/sim/table`, `/sim/markers`; JSON state `/sim/score` |
| `table_sim/sim_panel.py` | Tk match panel window (score, clock, P/Y/B counts, events, start/reset, pick/drop buttons) |
| `table_sim/goal_driver.py` | click "2D Goal Pose" in RViz to drive our robot |
| `table_sim/headless.py` | many matches without ROS, `--video out.mp4` |
| `meshes/table_2027.dae/.png` | textured table floor for RViz |
| `launch/sim.launch.py` | args: `team`, `opponent`, `our_robot` (`cmd_vel` / `scripted:<behaviour>`), `goal_driver`, `panel`, `rviz`, `rtf`, `auto_start` |

Keep `world.py`, `field.py`, `geometry.py`, `gridplan.py` free of ROS imports
(they also run headless).

Run:
```bash
ros2 launch table_sim sim.launch.py                                   # drive with 2D Goal Pose
ros2 launch table_sim sim.launch.py our_robot:=scripted:mirror opponent:=raider
ros2 action send_goal /face_tool table_sim_interfaces/action/FaceTool "{face: 0, mode: 0}"   # 0 pick, 1 drop
ros2 topic echo /sim/score --once
```

## Strategy: `src/adaptive_strategy`

Sensor-realistic match strategy (not the old `robot_application/task_planner`,
which is a dead-code-riddled priority scheduler with no real opponent
awareness). Never reads ground truth (no `/opponent/odom`, no `/sim/score`) -
only what the real robot will eventually have: `/cmd_vel` out, `/odom`,
`/scan`, `/camera/detections`, `/face_tool` (sim-only for now - the real
robot has no equivalent action yet, see `robot_actuators`), `/game/time_remaining`,
`/game/match_active`.

| file | role |
|---|---|
| `adaptive_strategy/core.py` | `StrategyCore` (the brain): `Observation`/`Command` dataclasses, pick/drop/go-home state machine, utility-based target scoring. No ROS imports - runs under `table_sim/headless.py --brain adaptive_strategy.core:StrategyCore` for fast iteration (no Docker needed, even on the Mac host). |
| `adaptive_strategy/belief.py` | `StackBelief`: our own occupancy belief (camera sighting > our own pick result > time-decayed prior), decay rate higher for enemy-side stacks - this is what makes "steal enemy stacks early, play safe late" fall out of one utility score instead of a hardcoded match-phase rule. Drop-zone fill order = lowest-numbered not-yet-full zone (`Y1..Y5`/`B1..B5` are already sorted nearest-home-first). |
| `adaptive_strategy/opponent_tracker.py` | `OpponentTracker`: clusters lidar hits into one position estimate with decaying confidence between sightings - only ever reacts to the opponent within lidar range, never tracks it at range. |
| `adaptive_strategy/strategy_node.py` | ROS2 wrapper - current pose via `map -> base_footprint` TF (same pattern `goal_driver.py` uses), drives `/face_tool` via `ActionClient`. |
| `launch/strategy.launch.py`, `config/strategy.yaml` | run alongside `sim.launch.py` (just another `/cmd_vel` publisher), tunable weights. |

Out of scope for v1: the flip/door mechanic (vertical boxes, circle zones) -
a scoring optimization on a working baseline, natural next step.

Run: `ros2 launch table_sim sim.launch.py goal_driver:=false` then
`ros2 launch adaptive_strategy strategy.launch.py`. Fast iteration without
ROS: `python3 -m table_sim.headless --brain adaptive_strategy.core:StrategyCore --opponent all --runs 3`.

## Status / next steps

1. ✅ Sim runs in the container, RViz shows the table.
2. ✅ Validated with Max: text markers, match panel, diagonal zone position,
   door/flip mechanic, camera + lidar ranges computed from real specs.
3. ✅ Adaptive strategy package (`src/adaptive_strategy`) - picks, drops,
   reacts to the opponent within lidar range, parks home in time. Validated
   headless against all 7 scripted opponents (parks 14/14) and live in RViz.
4. Next: tune collision/stuck-recovery against aggressive opponents (`raider`
   scores low for us currently - see `StrategyCore.blocked_t`), then the
   flip/door mechanic as a v2 scoring layer on top.
