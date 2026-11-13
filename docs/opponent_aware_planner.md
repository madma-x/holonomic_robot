# Opponent-Aware Match Strategy Planner

This is the spec for a **search-based** strategy planner, re-baselined against
what already exists in the repo. The current `adaptive_strategy` brain is
rule-based and was built for testing. It stays as the **benchmark baseline**
that the planner has to beat.

Legend: ✅ done · 🟡 partial / differs · ⬜ to do

---

## 1. Objective

Replace the rule-based `StrategyCore._decide` (fixed `pick_sequence` and
`drop_sequence`) with a search-based planner. The planner chooses the sequence
of picks, drops, constructions and the final go-home. It accounts for the
opponent, travel and handling time, the 9-box capacity, the drop constraints
and the remaining match time.

- The planner is ROS-free and Nav2-free, like `core.py` / `belief.py` today.
  It evaluates actions with estimated times, and the robot executes them through
  the existing navigation and tool stack.
- It reuses `StrategyCore`'s **execution layer**: `_goto`, `_dock_spot` /
  `_next_slot`, flip handling and backout. Only the *decision* is replaced.
- **Language: Python first**, inside `adaptive_strategy`, so it plugs into
  `headless.py --brain` and `strategy_node.py` unchanged. Port the search core to
  C++ (e.g. pybind11) only if profiling shows the 2–5 Hz target can't be met.
- **Acceptance:** it beats the baseline in
  `headless.py --opponent all --runs N` before it replaces it.

## 2. Game model

- Arena 3 × 2 m, match 85 s, robot must end parked in its zone (inside the castle).
- 10 pick stacks of 3 boxes, P1–P10 (`src/table_sim/config/field_2027.yaml`).
  Box 320 × 110 × 110 mm.
- **Carrying capacity: 9 boxes = 3 faces × 3.** One pick loads a whole stack
  onto one face. You can't pick onto a vertical face. Only the flip face can
  turn boxes vertical, and only inside the castle polygon (`field.in_castle`).
- **Drop zones per team:** Y1–Y5 (rectangles) and YC1–YC4 (circles, 1 vertical
  box each). **YC4 is back in play**: remove it from `excluded_drop_zones` in
  `strategy.yaml`.

### Scoring (official rules as given by Max)

Only **one construction per drop zone**:

| construction | where | boxes | points |
|---|---|---|---|
| wall (stack of 3 horizontal boxes) | Y1–Y5 | 3 | 3 × 1 + 5 = **8** |
| tower (1 vertical box) | YC1–YC4 | 1 | 1 + 5 = **6** |
| door | any **one** of Y1–Y5 | 3 | **23 total** (20 + 3 × 1) |

- 1 point per box in a drop zone.
- Extra boxes dropped in a zone whose construction is already built score
  **+1 each only**, with no second construction bonus.
- Parking points: still a placeholder (`robot_in_end_zone`).

### Door

The door is already modelled in `world.py`: `RobotState.door_used` /
`door_zone`, and the post/lid logic in `_update_action`. Sequence: on the flip
face, drop 2 vertical boxes (posts) → back out → flip the remaining box
horizontal (the "elevating" phase, see `request_action`) → move forward → drop
it as the lid. There is one door per robot per match. The door is built with
the robot's **flip face** (`robot.flip_face` in `sim.yaml`, currently 0), the
same face that builds towers.

### Opponents

`table_sim/opponents.py` provides scripted test fixtures: mirror, greedy, raider,
center_first, random, blocker, idle, teleop. Scripted opponents never flip.

## 3. Strategic objective

Lexicographic, evaluated over the opponent scenarios:

1. Maximize P(points ≥ threshold). The threshold is configurable.
2. Maximize E[points].
3. Minimize completion time.
4. Minimize risk (contacts, opponent interference).

Boxes still carried at the end score 0. Plans that can't park in time rank last.

✅ **Phase 0 done (2026-11-05): the sim scoring matches the rules above.** (Old text follows for history.)
`world.py` `World.score()` currently gives 3 per box + 6 per full stack, with no
tower or door points. `field_2027.yaml` `scoring:` must become box 1, wall +5,
tower +5, door +20. `zone_capacity: 3` must allow extra boxes at +1 each, and
only the first construction in a zone earns its bonus. Until then, neither the
planner nor the baseline is measured against the real rules.

## 4. State representation — `PlanState` ⬜

A compact, copyable state rebuilt each planning cycle from `StrategyCore`,
`StackBelief` and `OpponentTracker`:

- Robot pose. Per face: count, vertical flag, vertical slots (mirrors
  `StrategyCore.faces` / `vertical` / `vertical_slots` ✅).
- Remaining boxes per stack, taken from `StackBelief.believed_count` ✅
  (priority: camera sighting > own pick result > decayed prior).
- Per zone: construction type (none, wall, tower, door) and box count. Door
  progress (`none`, `posts`, `done`).
- Drop-rule state: zones dropped so far (in order), stacks picked so far, the
  pending "must drop next" zone (Y3 or Y4), and whether Y5 is done.
- t / time_left. Opponent scenario index and estimate (`OpponentEstimate`: x, y,
  vx, vy, sigma ✅).

## 5. Actions ⬜

Search-level actions:

- `PICK(stack, face)`
- `DROP_WALL(zone, face)` (or an extra +1 drop)
- `DROP_TOWER(circle, slot)`
- `DOOR_POSTS(zone)` → `DOOR_LID(zone)`
- `FLIP(face)`: castle only. It can also run concurrently while driving, as
  `_decide_flip` does today.
- `GO_HOME`
- `WAIT`

Each one maps onto the existing execution: FaceTool pick/drop/flip and the
`core.py` target kinds. The door has ⬜ **no execution in the brain yet**. The
sim supports it, but `StrategyCore` has no door sequence (`drop_sequence` type
`D` is a placeholder).

Legality mirrors `world.py`: face capacity, no pick onto a vertical face, only
a single box can un-flip, circle zones take vertical boxes only, one
construction per zone, one door per match.

## 6. Pickup

✅ Reuse:
- `belief.pick_score` / `best_pick`: believed boxes ÷ (travel + handling),
  discounted by the opponent's projected position (`contested_*` params).
- `belief.next_pick`: fixed order, used by the baseline.
- `field.pick_spots` and camera correction of the belief.

🟡 `next_pick(opp=…)` has a hi/lo swap (`contested_hi_group` /
`contested_lo_group`), but those params aren't set in `strategy.yaml`, so it
currently does nothing. The `strategy.yaml` comment describes a column-mirror
swap, which doesn't match the code.

⬜ For each stack, compute the opponent's arrival time and P(opponent takes it
first), per scenario. This must account for depletion while we travel.

## 7. Drop-off rules

### Hard constraints (Max)

1. **Start top:** if the first drop is **Y1**, the next drop must be **Y3**, with
   no pick in between.
2. **Start bottom:** if the first drop is **Y2**, the next drop must be **Y4**,
   with no pick in between.
3. **YC1** is allowed only once Y1 and Y3 are both dropped **and P1 has been
   picked** (P1 feeds it).
4. **YC4** is allowed only once Y2 and Y4 are both dropped **and P2 has been
   picked**.
5. **Y5 is the last drop of the match.** It must be made from inside the castle,
   because the parking zone is inside the castle. The plan ends with Y5 → park.
6. One door per match, on any of Y1–Y5. No ordering constraint is stated for
   YC2 or YC3.

### Rules are data, not code

Express the rules as a guard / precedence model in `strategy.yaml`. It replaces
today's fixed `drop_sequence`. Example:

```yaml
drop_rules:
  - {zone: Y3,  must_follow_immediately: Y1}
  - {zone: Y4,  must_follow_immediately: Y2}
  - {zone: YC1, requires_dropped: [Y1, Y3], requires_picked: [P1]}
  - {zone: YC4, requires_dropped: [Y2, Y4], requires_picked: [P2]}
  - {zone: Y5,  last: true, approach: inside}
```

The action generator evaluates the guards, and the search algorithm never
hardcodes them. Everything else is the planner's choice: top or bottom start,
when to interleave picks, which zone gets the door, extra +1 boxes, and which
face drops.

### Existing pieces

- ✅ `belief.due_order` / `first_interleave_order` already enforce rule 1 for a
  top start.
- ✅ `belief.required_pick` / `is_pinned` already pin P1 → YC1 and P2 → YC4 onto
  the flip face.
- ✅ Hand-calibrated dock poses are in `config/dock_poses.json` (with optional
  `approach_deg`). The circle-zone slot shift (`_dock_spot`) is reused as is.
  ✅ **Y5 dock pose from inside the castle** (Max) is in `dock_poses.json`:
  x 0.670, y 1.001, yaw 0°, approach 180° → face 0, pre-pose (0.47, 1.001)
  inside the castle. It hasn't been confirmed in a match yet, because the
  baseline stalls before Y5. ⬜ Check YC4 again now that it's back.
- The pose after each drop (including backout) is the start of the next leg,
  and it must be scored as part of the plan.

## 8. Travel-time model

🟡 Existing:
- `_eta` (straight line) in `core.py`.
- `gridplan.GridPlanner` / `path_length_estimate`: A* with stack, zone and
  opponent-disc obstacles.
- Tool timings in `sim.yaml`: `pick_engage_time`, `pick_time_per_box`,
  `drop_time`, `flip_time`.

⬜ A cached travel-time matrix between strategic poses (pick spots, dock poses,
door backout poses, home). Build it from `path_length_estimate` with a trapezoid
velocity profile (`max_lin_vel`, `max_lin_acc`) and a rotation term
(`max_ang_vel`). Calibrate it against leg times measured in headless runs.
Nav2 is never run inside the planner.

## 9. Forward simulators

✅ **Full-fidelity** `world.py`: collisions, lidar, camera, tool timings, door,
85 s clock, deterministic per seed. `headless.py run_match` runs any brain
against any scripted opponent.

⬜ **Fast abstract** `simulate(state, action, scenario) -> new_state` for the
search. It must be pure (returns a new state) with no physics, using the same
legality and scoring as the corrected `world.py`. Unit-test it against `World`
transitions.

## 10. Opponent prediction

✅ `OpponentTracker` does live tracking: lidar cluster, velocity smoothing, and a
sigma that grows while the opponent is unseen. Detect range is 1.1 m in sim
(2.8 m physical). Per-side decay priors (`decay_own/center/enemy_per_s`) act as a
crude "opponent took it" model.

⬜ Defensive, aggressive and adaptive scenarios with configurable probabilities
and an expected stack order and timing for each. Seed them from the
`opponents.py` orderings: mirror ≈ defensive, raider ≈ aggressive,
center_first / greedy ≈ adaptive. Later, update the probabilities from tracker
observations.

## 11. Search ⬜

- Depth-limited **beam search**. Start at beam 150, depth 15, and tune by
  profiling.
- **Receding horizon:** execute only the first action, then replan. Replan when
  a target completes or fails, the opponent moves significantly, the belief
  changes, Nav2 fails, or the plan becomes invalid.
- Implement it as a new brain class with the same `step(obs, dt) -> Command`
  interface as `StrategyCore`.

## 12. Evaluation ⬜

`EvaluationResult` {P(≥ threshold), E[points], completion time, risk}, averaged
over the scenarios and compared lexicographically. Reuse the go-home guard
(`eta_home + go_home_margin_s`) as a hard feasibility check. Carried boxes and
a failed park count as lost.

## 13. Search optimization ⬜

Generate legal actions only. Use dominance pruning on the *full* `PlanState`
key, never on pose alone. Cache the travel-time matrix.

## 14. ROS 2 integration

✅ `strategy_node.py` already wraps any brain:
- **Inputs:** pose from the `map → base_footprint` TF, `/scan` (mount yaw
  resolved through TF), `/camera/detections`, `/game/time_remaining`,
  `/game/match_active`, FaceTool results.
- **Outputs:** `/cmd_vel`, Nav2 goals, `/face_tool` pick/drop/flip, a
  loaded-aware footprint, `/strategy/status`, markers, and `brain.debug.say`
  forwarded to the ROS log.

✅ Execution: Nav2 for the coarse legs, and `motion_controller`
PurePursuitController (with the terminal docking stage) for the final approach.

🟡 `/face_tool` is sim-only. The real robot needs an equivalent action
(`robot_actuators`).

## 15. Debugging and visualization

✅ RViz markers, `sim_panel.py`, `headless.py --video/--png/--frames`,
`render.py`, `replay_match.py`, `bag2frames.py`, and the `sim-match-launch`
skill.

⬜ A per-cycle planner log (state → chosen plan → P / E / time / risk) and a
top-N candidate-plan overlay in `render.py`.

## 16. Phases

| # | Phase | Status |
|---|---|---|
| 0 | Rules in sim: new scoring (+1 extra boxes, one bonus per zone, door 23), YC4 back, Y5 inside dock pose | ✅ (2026-11-05) |
| 1 | Abstract fast simulator + `PlanState` + legality | ✅ (2026-11-06) `plan_state.py`, `travel.py`, `test/test_plan_state.py` |
| 2 | Drop guards as data (Y1→Y3 / Y2→Y4, YC1 / YC4 prerequisites, Y5 last from inside) + door execution in the brain | ✅ guards (`rules.py`, `drop_rules:`) + door in brain (2026-11-06); ⚠ default choreography stalls at the castle waist, see below |
| 3 | Beam search + evaluator + pruning, static opponent | 🟡 built (`planner.py`, `planner_brain.py`), runs end to end, does NOT yet beat the baseline |
| 4 | Opponent scenarios + arrival time / depletion | ✅ (2026-11-06) `scenarios.py`, `plan_robust` |
| 5 | Full lexicographic evaluation, complete cycles | ⬜ |
| 6 | ROS 2 integration | ✅ (new brain plugs into `strategy_node.py`) |
| 7 | Profiling; C++ port only if needed | ⬜ |

## 17. Design principles

1. Don't build the decision tree by hand. Generate it from legal actions plus
   simulated consequences.
2. Evaluate complete pick → drop cycles, including the pose after each drop.
3. Drop rules are hard constraints, kept as data in `strategy.yaml`.
4. Scoring is construction-based (wall, tower, door, plus +1 extra boxes), not a
   raw object count.
5. Model the opponent probabilistically, and never assume that every contested
   stack is lost.
6. Strategy stays independent of Nav2: the planner chooses *what*, and Nav2 /
   `motion_controller` choose *how*.
7. Replan continuously, one action at a time.
8. The rule-based brain is the baseline to beat, not the target design.

---

## Baseline (2026-11-05, rule-based `StrategyCore`, **old scoring**)

```
PYTHONPATH=src/table_sim:src/adaptive_strategy python3 -m table_sim.headless \
  --brain adaptive_strategy.core:StrategyCore \
  --params src/adaptive_strategy/config/strategy.yaml --opponent all --runs 3
```

`21 matches | avg us 28.6 vs 39.5 | wins 9/21 | parked 0/21 | contacts 435`

The baseline delivered 6 boxes / 2 stacks in every match except one, and
finished with 9 boxes still carried. **It never parked.** Against `random`
seed 2 it scored 0, with 171 contacts. Against `blocker` it logged 70 contacts
per match. In other words, the current fixed `drop_sequence` stalls after
Y1/Y3. Rerun this baseline once Phase 0 (scoring) is done.

## Baseline v2 (2026-11-05, **new scoring**, headless dock-pose path fixed)

Cause of the old 0/21 park: `headless.py` didn't resolve `dock_poses_file`
relative to the params file (as `strategy_node.py` does), so no calibrated dock
poses loaded and the robot wedged at a geometric YC1 spot. Fixed in
`headless.load_params`.

`21 matches | avg us 34.3 vs 24.2 | wins 13/21 | parked 17/21 | contacts 387`

Known remaining issues (not Phase 0): vs `raider` it scores 16 and doesn't park;
vs `random` seed 2 it scores 0 (171 contacts); in most matches it drops a flipped
face into Y5 by accident (two vertical "posts", then a failing drop loop) because
the brain has no door logic (Phase 2). No door or Y5-from-inside-castle score yet.

## Phase 2 results (2026-11-06)

- `rules.py` + `strategy.yaml drop_rules:` give the planner a `guard`; tested in `test_plan_state.py`.
- Brain door: posts -> backout -> un-flip -> lid verified (Y4 door = 23 pts) with a test order. Flip is now only
  requested when needed (tower pending / door due); a lone vertical box is un-flipped before a rect drop (fixes
  the accidental Y5 posts).
- `drop_sequence` is now rules-compliant (Y5 last, YC4 in, P1/P2 pins). Headless `--opponent all --runs 3`:
  `avg us 29.8 vs 24.2 | wins 13/21 | parked 0/21`.
- **Open problem (choreography):** after YC2 + YC3 are placed the castle waist is ~0.48 m wide; a robot loaded on
  all 3 faces needs ~0.56 m, so it cannot re-enter to build Y2/Y4 and stalls. The planner must avoid this
  (model the waist as a loaded-robot constraint), or the tower order must change.
- Also: a tower load picked outside the castle is never flipped (flip needs the robot centre in the castle).

## Phase 2 update (2026-11-06, later)

- Reference: a fully built castle is **79 points** (4 walls 32 + 4 towers 24 + door 23; 19 boxes), 89 with parking.
- Circles: YC2 y 1.315 -> 1.415, YC3 y 0.703 -> 0.683 (dock poses shifted to match); waist 0.48 -> 0.62 m.
  Final Y5 entry only needs ~36 cm (one loaded face, 32 cm box); the wider gap serves the loaded door trip.
- New rule `only_face_loaded` on Y5 (guard + brain: other faces dumped as +1 extras in built zones first).
- Brain fixes: flip detour through the start zone, door gate no longer hangs, trip-fits-in-time check before a
  drop, `pick_min_slack_s` 22, and the path planner inflates by held boxes when loaded.
- Baseline v3 (`--opponent all --runs 3`): `avg us 52.8 vs 24.2 | wins 20/21 | parked 14/21`.
  Idle opponent: 75 (misses Y5 wall + YC4 for time). Still bad: raider (16), random seed 2 (0).
- Flip-face rule (Max): a vertical batch must be emptied before the flip face can pick again; tower+tower+tower,
  door (3), or tower+tower+lone box un-flipped. A single tower strands 2 vertical boxes (only a lone box un-flips).

## Phase 3 status (2026-11-06)

- `planner.py`: beam search (150 x depth 15), value = points if it went home now (+ potential for carried boxes:
  2/horizontal, 4/vertical), dominance pruning on `PlanState.key()`, ~0.3 s per replan (0.7 s the first time).
- `planner_brain.py` (`PlannerBrain(StrategyCore)`): replans whenever a target completes, executes the first action
  through the existing execution layer. Run: `--brain adaptive_strategy.planner_brain:PlannerBrain`.
- Rules added: un-flip any count in the castle (YC4 leftovers become +1 extras), pick only onto an empty face, door lid
  only from the flip face, door in progress is atomic (flip, lid, or home).
- 1 seed, 7 opponents: planner avg 43.6 (parked 3/7) vs baseline 55.3 (parked 5/7). Best case 75 (center_first, 0 carried).
  Weak spots: it opens with the door and then often stalls in dock/pile collisions (dock poses to be debugged by Max),
  and the heuristic credits are untuned.

## Phase 4 results (2026-11-06)

- `scenarios.py`: defensive / aggressive / adaptive / idle (priors 0.45 / 0.20 / 0.30 / 0.05, in `strategy.yaml`),
  each an expected stack order turned into per-stack `take` times and a path by a simple timing model
  (0.7 m/s, 1 s per pick, 9 s drop trip per 3 picks). Live updates: lidar sightings reweight by predicted
  position, stacks seen emptied (not by us) reweight by predicted take time; weights are floored at 2 %.
- Search: a pick is illegal when P(taken before our arrival) >= 0.5. The 24 best complete plans are re-run
  open-loop under every scenario and ranked lexicographically: P(points >= 50), E[points], finish time, failed picks.
- Planner brain also gives up on a stuck target after 8 s and bans it for 15 s (the rule brain waits forever).
- Benchmark (`--opponent all --runs 3`), planner: `avg us 56.5 vs 28.0 | wins 17/21 | parked 11/21 | contacts 335`.
  Rule-based baseline v3: `52.8 vs 24.2 | 20/21 | 14/21`. Planner scores more (raider 70 vs 16, greedy 62, center_first 75,
  random 43-75) but loses to `mirror` (37 vs 50) and doesn't park as reliably. Not yet at the acceptance bar
  (beat baseline AND park every match).

## Drop fixes after the first replays (2026-11-06)

- Rule `first_drop_in: [Y1, Y2]` (the first drop of the match); 'home' stays legal while a Y1->Y3 / Y2->Y4 pair is
  pending (it is the abandon move, and the search values states by "go home now").
- Y4 dock pose: x 0.440, y 0.473, yaw -30, approach -90 (reference face 1).
- A dock pose fixes position + approach only; the strategy picks the face and the body yaw is the dock heading minus
  that face's angle (planner passes the face via `_drop_face`; no forced dock face). Door at Y4 works from face 0.
- World: a horizontal drop must be centred (sim.yaml `tool.drop_tol_along_m` 0.06, `drop_tol_across_m` 0.05,
  `drop_tol_angle_deg` 10), else the drop fails "not centred". Brain tolerances tightened (0.03 m / 0.06 rad).
- 7 opponents, seed 0, planner: 51 / 67 / 70 / 70 / 70 / 75 / 36 (mirror..idle), avg 62.7 (was 53.7).

## Rules fixed after replay review (2026-11-06)

- `requires_dropped`: Y3 needs Y1 dropped before, Y4 needs Y2 (a first-drop rule alone let Y3 through later).
- Door only at Y5 (`door_only_in: [Y5]`, `door_zone: Y5`): Y4's dock (y 0.473) is outside the castle, where the
  lid's un-flip is refused. The 'last drop' rule lets the Y5 door be finished (flip + lid) after its posts.
- `field_2027.yaml door_accepts_extras: false`: a finished door takes no 4th box (kickoff said door + extra = 24;
  flip the switch to restore).
- Self-calibrating docks: a "not centred (along a cm, across b cm)" refusal shifts that zone's dock by (-a, -b) in
  the zone frame and logs it (`core._note_off_centre`), so the calibrated poses stay usable (Y3 was 5.0 cm off).
- Flip detour now goes to the start zone and holds there until the flip completes (it used to leave as soon as the
  centre crossed the castle edge, so the flip never finished).
- 7 opponents, seed 0: planner avg 52.0 (7/7 wins, parked 6/7), rule brain 48.9 (parked 6/7). The planner builds
  the Y5 door rarely: it spends the flip face on towers and has no time left for a fresh 3-stack.

## Opponent-side picks, in-castle flips, extras off (2026-11-07)

- Denial value: picking a box an opponent scenario would have emptied later (before 50 s) is worth `deny_weight`
  (1.0) points per box in the search value and the scenario ranking (not in P(points >= threshold)).
- Flip legality in the planner now matches World: the held-box footprint must be inside the castle, which depends on
  the docked face (Y1 docked on face 0 or 2 can flip face 0, on face 1 it cannot). `PlanState.dock_f` tracks it.
- The brain flips early (`_flip_coming`): if the plan flips the flip face next without using it before, it flips
  while docked/driving in the castle instead of a later detour. Refused flips back off 1 s.
- `field_2027.yaml extra_boxes: false`: no +1 extras for now (they made the robot stick on built zones). Set `true`
  to restore (limit `zone_max_boxes`).
- 7 opponents, seed 0, planner: avg 61.4 vs 34.0, wins 6/7, parked 7/7. Picks on the opponent side in 6/7 matches
  (first at ~7.6 s); 0 s spent in flip detour/hold.

## Replay review: early home, 2-stack start, stopping at ~40 s (2026-11-07)

Causes found (the beam size/depth was NOT the limit: 600 x 25 found the same plan):
- Time barely counted in the final plan choice (0.02 pt/s), so slow loops and junk picks were free, and the
  denial bonus even paid for hoarding boxes. Hoarded boxes then made Y5 (only its face loaded) impossible, so the
  plan ended early. Fixes: denial is credited only when the boxes are delivered (`PlanState.fbonus` per face ->
  `bonus` on drop), and `plan_time_weight: 0.5` (pt/s).
- A flip outside the castle was illegal in the model although the brain flips on the way in. Now a flip outside is a
  `flip_debt`: the very next action must be a drop at a castle dock where the flip is legal.
- `plan_rank_time_weight` (beam ranking) exists but did not help; default 0.
- Result (7 opponents, seed 0): picks 3 stacks before the first drop in 7/7 (was 2 in some), carried at the end 0 in
  6/7, last action at 51-74 s (was ~41 s). avg 59.4, wins 7/7, parked 7/7. Results are noisy between settings
  (raider 16..49), so keep an eye on it. Still open: door rarely built, 4 of 7 runs stop around 52-57 s.

## Five fixes from the replay analysis (2026-11-07)

1. Re-ranking now sees plans for EVERY first action (top 3 each + global top 12), P(>=50) rounded to 0.1,
   so a first pick with a better P(>=50) is really compared (`planner.plan(per_first)`).
2. Scenarios: new `blocker` scenario (priors 0.40/0.15/0.27/0.10/0.08); sighting evidence tempered
   (`scenario_pos_power` 0.35, `scenario_taken_power` 0.7); `observe_present` (camera sees a stack still full
   discredits scenarios that emptied it). The blocker is now recognised (0.92) instead of read as a raider (0.94).
3. Hysteresis: the previous plan's remainder competes with +`plan_hysteresis` (2) points; it was kept in 40 of 94
   replans.
4. `pin_to_flip_face: [P1, P2]` rule: stacks that unlock towers go onto the flip face (the original design), so
   rule-driven picks are not junk.
5. Denial = `deny_weight` (2.0) * sum_scenarios prob * exp(-(take - arrival)/deny_tau 15 s): it separates stacks
   (was ~0.95 for almost all).
- P9/P10 flaws: the hard P(taken) >= 0.5 refusal is now 0.85 (`pick_block_p`; risk below that is priced by the
  scenario re-ranking); `observe_present` supplies the missing 'still there' evidence.
- `--opponent all --runs 3`: planner avg 62.8 vs 36.3, wins 21/21, parked 21/21 (acceptance bar met).
