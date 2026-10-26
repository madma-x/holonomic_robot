# holonomic_robot – context for Claude Code

ROS 2 workspace for our Eurobot / Coupe de France robot. Real robot runs ROS 2
Jazzy (fallback Humble). On the Mac, ROS 2 Jazzy is only available **inside
Docker images** (`docker images | grep jazzy`); build and run everything there,
with this repo mounted in the container.

## Robot
- Holonomic, 3 omni wheels, **equilateral triangle, 340 mm per face**.
- Each face can hold **3 boxes** (stacked). Face 0 assumed along +x of
  `base_footprint`, faces at 0°, 120°, 240° (to confirm on the real robot).
- 2D lidar (SL C1) **on a mast**: only sees the other robot, ~1 m range.
- Frames: `map -> odom -> base_footprint -> base_link -> base_scan`
  (`base_scan` is yawed by pi in the URDF). Motion via `/cmd_vel` (body frame).

## Game: Eurobot 2027 "The legend of Camelot"
- Table 3000 x 2000 mm, match **85 s**, two robots (yellow start square at
  x 0–0.5 m, blue mirrored at x 2.5–3 m). Robot must **end inside its zone**.
- Boxes **320 x 110 x 110 mm**. **10 pick stacks of 3 boxes** (P1–P10):
  4 on each long edge, 2 in the centre (x = 1.5 m).
- **5 drop zones per team** (Y1–Y5 / B1–B5), **max 3 boxes per zone**;
  best is a stack of 3 (same as in the pick zones). The diagonal zone is
  centred at (650, 650) mm (+ twin at y = 1350, mirrored for blue).
- Points in `src/table_sim/config/field_2027.yaml` are PLACEHOLDERS: check the
  official rules (coupederobotique.fr, "Règlement 2027") and update them.

## Current work: branch `feature/table-sim-2027`
Goal, in this order:
1. **Validate the simulation with Max** (current step), see below.
2. Then write the strategy: adapt where to pick depending on where the
   opponent is and what it takes (reactive avoidance, opponent tracking from
   the mast lidar, endgame return home). Do not reuse the old strategy code in
   `robot_application` (task_planner etc.); write a new one.

### Packages added
- `src/table_sim_interfaces`: action `FaceTool` (`face`, `mode` PICK=0/DROP=1).
- `src/table_sim`: light 2D simulator shown in RViz (no Gazebo).
  - `table_sim/world.py`: kinematic robots with accel limits, stacks and piles
    are single objects with a box count (no box physics on purpose), mast
    lidar raycast, scoring, 85 s match.
  - `table_sim/field.py` + `config/field_2027.yaml`: table layout and IDs.
  - `table_sim/opponents.py`: scripted opponents (mirror, greedy, raider,
    center_first, random, blocker, idle, teleop).
  - `table_sim/sim_node.py`: ROS node. Publishes `/odom`, TF, `/scan`,
    `/joint_states`, `/clock` (sim time), markers `/sim/table` (textured table
    `meshes/table_2027.dae`) and `/sim/markers`; action `/face_tool`;
    services `/sim/start`, `/sim/reset`, `/game/start_match`.
  - `table_sim/goal_driver.py`: click "2D Goal Pose" in RViz to drive.
  - `table_sim/headless.py`: many matches without ROS, `--video x.mp4`.
  - `launch/sim.launch.py`, `rviz/table_sim.rviz`, `README.md`.
- `scripts/sim_in_docker.sh`: build + launch inside the Jazzy container.

### Not yet tested in real ROS
The node was only smoke-tested with stub ROS modules (no ROS where it was
written). First thing to do:
```bash
colcon build --symlink-install --packages-select table_sim_interfaces table_sim holonomic_robot_description
source install/setup.bash
ros2 launch table_sim sim.launch.py            # then click 2D Goal Pose in RViz
ros2 launch table_sim sim.launch.py our_robot:=scripted:mirror opponent:=raider
```
Check: textured table visible on the ground (if the .dae texture fails, fall
back to a colored PointCloud2 like `robot_application/floor_image_publisher.py`),
IDs on pick/drop zones, robot moves, pick/drop via
`ros2 action send_goal /face_tool table_sim_interfaces/action/FaceTool "{face: 0, mode: 0}"`,
magenta lidar points when within 1 m of the opponent, scoreboard.

### Open questions for Max
- Size/orientation of the diagonal drop zones (assumed 400 x 200 at ±45°).
- Approach side of each pick/drop zone (centre stacks P5/P6 from both sides).
- End zone = start square?
- Real face numbering of the robot.

## Conventions
- Commit only when asked; keep work on `feature/table-sim-2027`.
- Python nodes are ament_python; keep the sim core (`world.py`, `field.py`,
  `geometry.py`) free of ROS imports so it also runs headless.
