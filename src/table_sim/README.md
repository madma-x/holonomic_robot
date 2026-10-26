# table_sim – Eurobot 2027 table simulator (ROS 2 Jazzy + RViz)

A light 2D simulator shown in 3D in RViz:

- the 2027 table drawn on the ground (`meshes/table_2027.png`),
- 10 pick stacks of 3 boxes (P1…P10) and 5 drop zones per team
  (Y1…Y5, B1…B5; max 3 boxes per zone),
- our holonomic triangle robot (340 mm faces, 3 boxes per face) driven by `/cmd_vel`,
- a scripted opponent (or a second teleop robot),
- a mast lidar on `/scan` that only sees the opponent's mast, within 1 m,
- an 85 s match with a score.

Only the robots are "physical" (stopped by borders, stacks, piles and each
other). Stacks and piles are single objects with a box count: nothing gets
pushed or knocked over.

## Build (inside your Jazzy container)

```bash
cd /path/to/holonomic_robot            # the repo, mounted in the container
colcon build --symlink-install \
  --packages-select table_sim_interfaces table_sim holonomic_robot_description
source install/setup.bash
```

## Run

```bash
ros2 launch table_sim sim.launch.py                      # you drive our robot
ros2 launch table_sim sim.launch.py our_robot:=scripted:mirror opponent:=raider
```

The 85 s match starts 3 s after launch (`auto_start:=false`, then
`ros2 service call /sim/start std_srvs/srv/Trigger`).
Restart: `ros2 service call /sim/reset std_srvs/srv/Trigger`.

| launch argument | values |
|---|---|
| `team` | `yellow` (default) or `blue` |
| `opponent` | `mirror`, `greedy`, `raider`, `center_first`, `random`, `blocker`, `idle`, `teleop` |
| `our_robot` | `cmd_vel` (default: driven by `/cmd_vel`) or `scripted:<behaviour>` to watch a full match |
| `goal_driver` | `true`: click **2D Goal Pose** in RViz to drive our robot |
| `rtf` | real-time factor, e.g. `2.0` |
| `rviz` | `true` / `false` |
| `panel` | `true`: match panel window (needs `python3-tk`) |

## Match panel

`sim_panel` opens a separate window (it shows in the VNC desktop next to RViz):
clock, score, both robots (pose, boxes per face, current action), every pick
stack P1–P10 and drop zone Y1–Y5 / B1–B5 with its box count, the last game
events, and buttons to start / reset the match and to pick / drop on each face.
If it does not open: `sudo apt install python3-tk`.

## Validate it by hand

1. Click **2D Goal Pose** in RViz: the robot drives there and turns to the arrow.
   The face numbers (0, 1, 2) are written on top of the robot; face 0 points along +x
   of `base_footprint` (the red axis).
2. Put a face flat against a stack (2–3 cm gap max), then pick:
   ```bash
   ros2 action send_goal --feedback /face_tool table_sim_interfaces/action/FaceTool "{face: 0, mode: 0}"
   ```
   Up to 3 boxes jump onto that face; the stack count goes down.
3. Drive so the held boxes sit over one of your drop zones and drop (`mode: 1`).
   The zone shows `Y3: 3/3`; the score goes up.
4. Drive within 1 m of the opponent: magenta points appear on `/scan`.
5. Opponent driven by hand: `opponent:=teleop`, then publish on `/opponent/cmd_vel`.

## Interfaces (same as the real robot where possible)

| | topic / service | type |
|---|---|---|
| in | `/cmd_vel` | `geometry_msgs/Twist` (body frame, vx, vy, wz) |
| out | `/odom`, TF `odom → base_footprint`, static `map → odom` | `nav_msgs/Odometry` |
| out | `/scan` (frame `base_scan`, 10 Hz, 500 pts) | `sensor_msgs/LaserScan` |
| out | `/joint_states` (wheel joints at 0) | for `robot_state_publisher` |
| action | `/face_tool` | `table_sim_interfaces/action/FaceTool` |
| out | `/game/match_active`, `/game/time_remaining`, `/game/score`, `/sim/score` (JSON) | |
| srv | `/sim/start`, `/game/start_match`, `/sim/reset` | `std_srvs/Trigger` |
| out | `/opponent/odom`, TF `map → opponent/base_footprint` | ground truth, debug only |
| out | `/clock` | simulated time: use `use_sim_time:=true` elsewhere |

## Configuration

- `config/field_2027.yaml`: table layout (stacks, zones, start zone, box size, scoring
  **placeholders**). Positions were measured from the rulebook figure.
- `config/sim.yaml`: robot size and limits, gripper timings, lidar, opponent.

## Headless (no ROS, many matches, video)

```bash
ros2 run table_sim headless_match --brain <module:Class> --opponent all --runs 3
ros2 run table_sim headless_match --brain <module:Class> --opponent raider --video raider.mp4
```
