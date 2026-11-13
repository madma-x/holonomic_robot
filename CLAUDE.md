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
- The real robot stack (`robot_application`, `can_interface`, `aruco_*`, …)
  needs hardware (CAN, serial, camera). Do not launch it in the container and
  do not change it unless asked. Exception: **`motion_controller`** (the
  `PurePursuitController` Nav2 plugin) is shared by both the sim
  (`table_sim/config/nav2_params_sim.yaml`) and the real robot
  (`robot_application/config/nav2_params_odom_only.yaml`) - it's fine to
  build/test/edit via the sim.
- Git: work on branch **`feature/table-sim-2027`**. Commit only when Max asks.
  Don't commit `__pycache__/`, `build/`, `install/`, `log/`, `.claude/`.
- Max validates step by step: show results (what you ran, what you saw)
  before moving on to the next feature.

## Robot

- Holonomic, 3 omni wheels, **truncated-triangle (hexagon) footprint**: 3 long
  "face" edges (`face_long_m = 0.22`) alternating with 3 short corner edges
  (`face_short_m = 0.08`) - not a bare equilateral triangle. See
  `table_sim/geometry.py` (`polygon_from_truncated_triangle`,
  `truncated_triangle_apothem`/`_circumradius`).
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
- **Drop zones per team:** rectangles **Y1–Y5** / **B1–B5** and circles
  **YC1–YC4** (1 vertical box each, needs a flip; YC4 is in play).
- Carrying capacity **9 boxes** (3 faces × 3).
- **Scoring (Max):** one construction per zone. Wall (stack of 3 horizontal)
  = 3 + 5 = 8. Tower (1 vertical box in YC) = 1 + 5 = 6. Door (one per match,
  any of Y1–Y5, built with the flip face) = 23 total. 1 point per box. Extra boxes in an already-built zone
  = +1 each only. Implemented in `world.py score()` (`zone_kind` per zone,
  `field_2027.yaml scoring:` box 1 / wall_bonus 5 / tower_bonus 5 / door_bonus 20,
  `zone_max_boxes` 6). Tests: `src/table_sim/test/test_scoring.py`.
- **Drop constraints:** Y1 first → Y3 next with no pick in between (or Y2 → Y4).
  YC1 needs Y1 + Y3 dropped and P1 picked. YC4 needs Y2 + Y4 dropped and P2
  picked. Y5 is the last drop, made from inside the castle, then park.
- Strategy goal: a **search-based** planner. The rule-based brain was built for
  testing. See [docs/opponent_aware_planner.md](docs/opponent_aware_planner.md).

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

`motion_controller::PurePursuitController` (the Nav2 controller plugin both
sim and real use) has a terminal **docking stage**: inside `docking_distance`
(0.3 m) of the goal it steers in a fixed approach-axis frame (along-track +
cross-track against the line derived from the plan's last `docking_lookback_dist`,
0.4 m) instead of chasing the pure-pursuit lookahead point, so a robot handed
off with side velocity from a curved upstream segment gets corrected by
`k_lateral` directly instead of overshooting the docking line. Tunable per
params file (`docking_enabled`, `docking_distance`, `docking_lookback_dist`,
`k_lateral`); see `pure_pursuit_controller.cpp`'s `updateApproachAxis()`.

Run:
```bash
ros2 launch table_sim sim.launch.py                                   # drive with 2D Goal Pose
ros2 launch table_sim sim.launch.py our_robot:=scripted:mirror opponent:=raider
ros2 action send_goal /face_tool table_sim_interfaces/action/FaceTool "{face: 0, mode: 0}"   # 0 pick, 1 drop
ros2 topic echo /sim/score --once
```

See also the **`sim-match-launch`** skill for the three ways to run a full
scored match (headless no-ROS, full RViz session, or manual launch).

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
| `adaptive_strategy/core.py` | `StrategyCore` (the brain): `Observation`/`Command` dataclasses, pick/drop/go-home state machine, utility-based target scoring, flip/vertical-slot tracking for circle zones (own belief of `vertical_slots` per face, mirrors `world.py`'s ground truth without reading it). No ROS imports - runs under `table_sim/headless.py --brain adaptive_strategy.core:StrategyCore` for fast iteration (no Docker needed, even on the Mac host). |
| `adaptive_strategy/belief.py` | `StackBelief`: our own occupancy belief (camera sighting > our own pick result > time-decayed prior), decay rate higher for enemy-side stacks - this is what makes "steal enemy stacks early, play safe late" fall out of one utility score instead of a hardcoded match-phase rule. Pick order is Max's fixed `pick_sequence` (not reordered by distance), with a semi-modular exception: if the due stack is in `contested_hi_group` and the opponent looks headed there, swap to a `contested_lo_group` alternative (params not yet set in `strategy.yaml` - currently a no-op until configured). Drop-zone fill order = `drop_sequence`/lowest-numbered not-yet-full zone. |
| `adaptive_strategy/opponent_tracker.py` | `OpponentTracker`: clusters lidar hits into one position estimate with decaying confidence between sightings - only ever reacts to the opponent within lidar range, never tracks it at range. |
| `adaptive_strategy/strategy_node.py` | ROS2 wrapper - current pose via `map -> base_footprint` TF (same pattern `goal_driver.py` uses), drives `/face_tool` via `ActionClient`. |
| `launch/strategy.launch.py`, `config/strategy.yaml` | run alongside `sim.launch.py` (just another `/cmd_vel` publisher), tunable weights. |

The flip/door mechanic (vertical boxes into circle zones) is implemented:
`core.py` tracks which of the 3 fixed vertical slots are occupied per face
and re-resolves the live target slot every tick against the calibrated
circle-zone dock pose (`_dock_spot`/`_next_slot`) so the footprint/costmap
tracks the real box, not just the zone centre.

Run: `ros2 launch table_sim sim.launch.py goal_driver:=false` then
`ros2 launch adaptive_strategy strategy.launch.py`. Fast iteration without
ROS: `python3 -m table_sim.headless --brain adaptive_strategy.core:StrategyCore --opponent all --runs 3`.

## Status / next steps

1. ✅ Sim runs in the container, RViz shows the table.
2. ✅ Validated with Max: text markers, match panel, diagonal zone position,
   door/flip mechanic, camera + lidar ranges computed from real specs.
3. ✅ Adaptive strategy package (`src/adaptive_strategy`) - picks, drops,
   flips/drops into circle zones, reacts to the opponent within lidar range.
   It used to park 14/14 headless, but **the 2026-11-05 headless baseline parks
   0/21**: it stalls after Y1/Y3 with 9 boxes carried (see the doc's Baseline
   section).
4. ✅ Corrected robot footprint from a bare equilateral triangle to the real
   truncated-triangle hexagon (`face_long_m`/`face_short_m`), which every
   pick/drop standoff and opponent-clearance radius is derived from.
5. ✅ `motion_controller`'s pure-pursuit controller got a terminal docking
   stage (fixed approach-axis steering near the goal) to stop sim robots
   overshooting the docking line with carried-over side velocity.
6. Next: the search-based planner, phases 0–7 in
   [docs/opponent_aware_planner.md](docs/opponent_aware_planner.md). It starts
   with Phase 0: real scoring in `world.py`, YC4 back in, fix the baseline stall
   (the Y5 inside-castle dock pose is already set). Kickoff prompt:
   [docs/planner_kickoff_prompt.md](docs/planner_kickoff_prompt.md).
