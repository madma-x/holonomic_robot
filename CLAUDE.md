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
- 2D lidar SL C1 **on a mast**: only sees the other robot (its mast), ~1 m.
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
| `table_sim/world.py` | world model, match clock, collisions, lidar raycast, score (no ROS) |
| `table_sim/field.py` | layout loader, IDs, pick/drop robot poses (no ROS) |
| `table_sim/opponents.py` | scripted opponents: mirror, greedy, raider, center_first, random, blocker, idle, teleop |
| `table_sim/sim_node.py` | ROS node: `/cmd_vel` in; `/odom`, TF, `/scan`, `/joint_states`, `/clock` out; action `/face_tool`; services `/sim/start`, `/sim/reset`, `/game/start_match`; markers `/sim/table`, `/sim/markers`; JSON state `/sim/score` |
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

## Status / next steps

1. ✅ Sim runs in the container, RViz shows the table.
2. ⏳ Validate with Max: text markers (scale fix), match panel (`python3-tk`
   needed), diagonal zone position/size, approach side of each zone, face
   numbering, end zone = start square?
3. Then: write the new adaptive strategy as its own package (not the old
   `robot_application/task_planner`), driving the sim only through the real
   robot's interfaces (`/cmd_vel`, `/odom`, `/scan`, `/face_tool`, `/game/*`).
