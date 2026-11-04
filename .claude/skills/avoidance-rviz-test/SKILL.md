---
name: avoidance-rviz-test
description: Use this skill to test motion_controller's opponent-avoidance (mechanism 1 costmap/footprint clearance, mechanism 2 predicted-opponent disk, the adaptive speed limit) live in RViz, head-on against the opponent on an empty table. Triggers on requests to test/verify/compare avoidance, collision-avoidance, or "robot and opponent going into each other" behaviour in the simulator.
---

# Avoidance testing in RViz, empty table

Reproduces the exact setup validated in this project for watching/comparing
`motion_controller::PurePursuitController`'s two avoidance mechanisms against
a controlled opponent, with no pick stacks/drop zones cluttering the costmap
or the view. Everything here runs inside the `table_sim_jazzy` container
(see CLAUDE.md for the general Docker workflow).

## Why an empty table

`src/table_sim/config/field_avoidance_test.yaml` is `field_2027.yaml` with
`stacks: []` and `zones: []` (table/box/start_zone kept) - the costmap then
only ever shows the opponent, so avoidance behaviour isn't confounded by the
robot routing around a pick stack. Pass it to the launch file instead of the
default:

```bash
field_file=$(ros2 pkg prefix table_sim)/share/table_sim/config/field_avoidance_test.yaml
# or, since the repo is mounted read/write in the container and this is a
# --symlink-install build, the source path works too:
field_file=/ws/src/table_sim/config/field_avoidance_test.yaml
```

## Launch

Our robot must be driven by **Nav2**, not `goal_driver.py` - `goal_driver`
publishes straight to `/cmd_vel` and bypasses the controller plugin
entirely, so neither mechanism ever runs. The opponent must be `teleop` -
every other scripted behaviour routes strategically and won't give a clean,
repeatable closing speed.

```bash
docker exec -u ubuntu -d table_sim_jazzy bash -lc '
  cd /ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && DISPLAY=:1 \
  ros2 launch table_sim sim.launch.py \
    team:=yellow opponent:=teleop opponent_teleop:=true our_robot:=cmd_vel \
    nav2:=true goal_driver:=false rviz:=true panel:=true auto_start:=true rtf:=1.0 \
    field_file:=/ws/src/table_sim/config/field_avoidance_test.yaml \
  > /tmp/sim.log 2>&1'
```

`opponent_teleop:=true` spawns an xterm on the VNC desktop running
`teleop_twist_keyboard` remapped to `/opponent/cmd_vel` - lets you pilot the
opponent by keyboard instead of a scripted `ros2 topic pub`. Only does
anything when `opponent:=teleop`. Kill it between relaunches along with
everything else (`pkill -9 -f teleop_twist_keyboard; pkill -9 -f xterm`) -
it's not covered by the other nodes' pkill patterns below.

Start poses/headings (yellow team, unchanged by the empty field): us at
`(0.25, 1.0)` heading `0` (facing +x); opponent (blue) mirrored at
`(2.75, 1.0)` heading `pi` (facing -x, i.e. toward us). So a positive
`/opponent/cmd_vel` `linear.x` drives it straight at our start.

RViz already has persistent displays wired up for the markers below
(`src/table_sim/rviz/table_sim.rviz`: "Mechanism 2 (opponent prediction)" on
`/debug/opponent_prediction`, "Mechanism 1 (avoidance footprint)" on
`/debug/avoidance_footprint`), both enabled by default - no manual
"Add display" needed, just rebuild `table_sim` after editing that file.

### map_server startup race - check before sending any goal

`lifecycle_manager_localization` sometimes calls `map_server`'s configure
transition before `/clock` is ticking reliably; when that happens
`map_server` sticks on `unconfigured` forever, the global costmap's static
layer never gets a map, and every planner goal fails with `"Costmap timed
out waiting for update"` - silently, with no error that points at the real
cause. Check before relying on any goal:

```bash
docker exec -u ubuntu table_sim_jazzy bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
   ros2 service call /map_server/get_state lifecycle_msgs/srv/GetState "{}"'
```

If the label isn't `active`, drive it through by hand (both calls return
`success=True` once `/clock` is up, which it will be a few seconds after
launch):

```bash
ros2 service call /map_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"  # configure
ros2 service call /map_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"  # activate
```

### Stale processes from an earlier session

`pkill -f sim.launch.py` only kills the launch process, not every node it
spawned, and a `lifecycle_manager`/`sim_panel`/`odom_sim` from hours earlier
can still be running and fight the new launch. Before relaunching, check and
kill explicit PIDs rather than trusting a single pattern:

```bash
docker exec -u ubuntu table_sim_jazzy bash -lc \
  'ps aux | grep -E "sim.launch|nav2_sim|sim_node|sim_panel|rviz2|controller_server|planner_server|map_server|lifecycle_manager|bt_navigator|behavior_server|waypoint_follower|odom_sim|robot_state_publisher|topic pub|action send_goal|teleop_twist_keyboard|xterm" | grep -v grep'
# then: kill -9 <pids>
```

### Footprint shape - hexagon, not a triangle

The robot's real footprint (`table_sim/world.py`'s `RobotModel`,
`sim.yaml`'s `robot.face_long_m`/`face_short_m`, default 0.22/0.08) is a
hexagon - an equilateral triangle with each 60deg corner cut off, 3 long
(face) edges alternating with 3 short (corner) edges - not the old bare
triangle. Three real bugs surfaced getting this right, all now fixed but
worth knowing about if something looks wrong again after touching this:

- **`sim_node.py`'s `_robot_prism`** (the "game marker" mesh) was hardcoded
  for exactly 3 vertices (`range(3)`, `% 3`) - for 6 vertices this drew a
  bogus diagonal chord as one "edge" and only capped half the top face. Now
  generalized to `len(v)` vertices.
- **Nav2's footprint topic subscriber is VOLATILE/BEST_EFFORT** (confirmed
  via `ros2 topic info -v /local_costmap/footprint`), while `sim_node`
  published the real footprint only ONCE at startup on a TRANSIENT_LOCAL
  publisher. A volatile reader that connects after that one publish never
  receives it (no late-join replay) - silently, no error either side - so
  the costmap kept its constructor-time default (a circle from
  `robot_radius`). Fixed by republishing on a 1Hz timer
  (`sim_node._publish_footprint`) instead of once.
- **`footprint_padding` (Nav2's `padFootprint()`) distorts this shape
  badly** - it nudges each vertex's x/y independently by
  `padding * sign(x)`/`padding * sign(y)`, not a true polygon offset. Fine
  for the old symmetric triangle, but on the hexagon it produced wildly
  wrong edge lengths (confirmed via `/global_costmap/published_footprint`).
  Both costmaps now run `footprint_padding: 0.0` - the real hard-block
  margin comes entirely from mechanism 1's own `avoidance_min_clearance_m_`
  (a proper edge-normal offset, see `offsetPolygon` in
  `pure_pursuit_controller.cpp`).

To sanity-check the footprint is right after any related change, read
`/global_costmap/published_footprint` or `/local_costmap/published_footprint`
directly (PolygonStamped, world-frame) and check edge lengths alternate
0.22/0.08 with all 6 vertices at the same ~0.1553m circumradius - `ros2
topic echo --full-length /global_costmap/published_footprint --once`.

## Toggling the mechanisms (live, no relaunch/rebuild needed)

Mechanism 1 (footprint vs. costmap clearance) is always active while
`local_avoidance_enabled: true` (set in `nav2_params_sim.yaml`, normally
left on - don't disable it for these tests, it's the baseline). Mechanism 2
(the predicted-opponent disk, its opponent-speed-based margin scaling, and
the proximity-based speed limit) can be toggled per test case:

```bash
ros2 param set /controller_server FollowPath.opponent_prediction_enabled <true|false>
ros2 param set /controller_server FollowPath.opponent_speed_limit_enabled <true|false>
```

Other tunables worth sweeping live the same way:
`opponent_radius_m`, `opponent_safety_margin_m`, `opponent_speed_deadband_m_s`,
`opponent_speed_full_effect_m_s`, `avoidance_caution_margin_m`,
`opponent_speed_limit_range_m`, `opponent_speed_limit_min_scale`.

In RViz, add `/debug/opponent_prediction` (mechanism 2's hard-stop/caution
disks, current and predicted) and `/debug/avoidance_footprint` (mechanism
1's margins drawn around the real footprint) as MarkerArray displays to see
both mechanisms' actual extents, not just infer them from parameter values.

## Per-test-case recipe

Always reset both robots to their init pose between cases - don't reuse
wherever the last case left them:

```bash
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"
ros2 service call /sim/start std_srvs/srv/Trigger "{}"
```

Then, per case:

1. Set whatever params this case needs (see above).
2. Send our robot's goal in the background (it blocks on feedback otherwise):
   ```bash
   docker exec -u ubuntu -d table_sim_jazzy bash -lc \
     'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
      ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: <gx>, y: <gy>, z: 0.0}, orientation: {w: 1.0}}}}" \
      > /tmp/goal.log 2>&1'
   ```
3. Drive the opponent in the background at a constant closing speed:
   ```bash
   docker exec -u ubuntu -d table_sim_jazzy bash -lc \
     'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
      ros2 topic pub -r 10 /opponent/cmd_vel geometry_msgs/msg/Twist "{linear: {x: <vx>}}" \
      > /tmp/opp.log 2>&1'
   ```
   Omit this call entirely for a stationary-opponent case.
4. Let it run, then read the verdict. `ros2 topic echo` truncates long
   strings by default - always pass `--full-length`:
   ```bash
   ros2 topic echo --full-length /sim/score --once
   ```
   Check `robots.us.contacts` / `robots.opp.contacts` and the `events` list
   for `"robots collided (closing speed ... m/s)"`.
5. Before the next case: stop this case's background commands first, then
   reset:
   ```bash
   pkill -9 -f "action send_goal"; pkill -9 -f "topic pub"
   ```

## Scenario menu

- **Stationary opponent** (skip step 3), goal placed at/through the
  opponent's position: the robot should pass close by (mechanism 2 should
  be nearly inert at zero opponent speed, letting mechanism 1's much
  tighter margin govern) and not collide.
- **Fast-closing opponent with lateral room** (goal offset a bit off the
  opponent's line, opponent `cmd_vel` 0.3-0.6 m/s): with mechanism 2 on,
  expect 0 contacts. Rerun with `opponent_prediction_enabled:=false` for
  contrast - a bare costmap snapshot reacts too late to a fast-closing
  obstacle, so this case is expected to collide.
- **No-lateral-room head-on** (goal sits exactly on the opponent's start
  position, same corridor): contact is expected either way - there's no
  room to dodge. The pass/fail signal here is that the robot stops
  advancing and gets pushed back rather than plowing straight through.

## Braking distance / detection range sanity check

Worth recomputing whenever `max_lin_acc` (`sim.yaml`), `lidar.detect_range`,
or `local_costmap.update_frequency` (`nav2_params_sim.yaml`) change. For two
robots closing head-on at 1 m/s each (2 m/s combined) with the physics-limit
deceleration (`max_lin_acc: 1.5 m/s^2` for both, combined 3 m/s^2):

- Braking distance: `2^2 / (2*3) = 0.67m`.
- Reaction/staleness distance added by the costmap's own update period (the
  obstacle picture mechanism 1/2 react against is only as fresh as this):
  at `update_frequency: 10.0` (raised from 5.0, Max's request), worst-case
  staleness 0.1s * 2 m/s = 0.2m (was 0.4m at 5Hz).
- Total worst case: `~0.67 + 0.2 = ~0.87m`, plus margin
  (`opponent_radius_m + opponent_safety_margin_m + min_gap ~= 0.2m`) if you
  don't want to stop at bare skin contact -> want `lidar.detect_range`
  comfortably above ~1.1m.

Also check the controller's ACTUAL loop rate during a run, not just its
configured `controller_frequency: 20.0` - `controller_server` logs `Control
loop missed its desired rate of 20.0000 Hz. Current loop rate is X Hz` when
it falls behind. A real loop rate half the configured one roughly doubles
the reaction-latency term above.

This WAS happening steadily (~10Hz under load with RViz + 7+ nodes in the
container) - root-caused to two things, both now fixed:
- `motion_controller`'s `CMakeLists.txt` never set `CMAKE_BUILD_TYPE`
  (confirmed via `grep CMAKE_BUILD_TYPE build/motion_controller/
  CMakeCache.txt` - was empty, i.e. no `-O2`/`-O3` at all). Now defaults to
  Release.
- `selectSafeVelocity` was calling `footprintCollides` TWICE per
  forward-simulated step per candidate (hard margin, then hard+caution
  margin) - two full independent costmap bbox scans with a `sqrt`-heavy
  point-to-polygon distance per cell. Replaced with `footprintClearance`,
  one combined scan at the larger margin that tests both thresholds per
  cell. Candidate count was also halved (`avoidance_vx/vy_samples_` 5->3,
  29 candidates instead of 77).

If the warning reappears after some other change, rebuild with a clean
`build/motion_controller` dir first (`rm -rf build/motion_controller
install/motion_controller`) to make sure a stale cached `CMAKE_BUILD_TYPE`
isn't masking the fix, then re-check with the same load (goal + opponent
cmd_vel running, not idle) - `grep -c "missed its desired rate"
/tmp/sim.log` should come back 0 or just a single `inf Hz` startup blip,
not a steady stream.

## Cleanup

```bash
pkill -9 -f "action send_goal"; pkill -9 -f "topic pub"
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"
ros2 service call /sim/start std_srvs/srv/Trigger "{}"
ros2 param set /controller_server FollowPath.opponent_prediction_enabled true
ros2 param set /controller_server FollowPath.opponent_speed_limit_enabled true
```

Leave the sim reset to init pose with both mechanism-2 toggles back to
`true` (the default) so a later session doesn't silently inherit a disabled
safety mechanism. Stop the whole launch with `pkill -f sim.launch.py` plus
the explicit-PID cleanup above if anything lingers.
