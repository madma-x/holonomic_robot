# Kickoff prompt: search-based strategy planner

Paste everything below the line into a new Claude Code session at the repo root.

---

We're replacing the rule-based Eurobot 2027 strategy with a **search-based
planner**. Read these before writing code: `CLAUDE.md`, then
`docs/opponent_aware_planner.md` (the spec: what exists, what's left, and the
phases), then `src/table_sim/table_sim/world.py` (`score()`, `_update_action`
including the door logic, `request_action`) and
`src/adaptive_strategy/adaptive_strategy/{core,belief}.py`.

## How we work
- One phase at a time. **After each phase, stop and show me what you ran and what
  you saw** (headless numbers, test output). Don't start the next phase until I
  say so.
- Branch `feature/table-sim-2027`. Don't commit unless I ask.
- `world.py`, `field.py`, `geometry.py`, `gridplan.py`, `core.py` and `belief.py`
  stay free of ROS imports. The planner goes in `src/adaptive_strategy/` as
  plain Python.
- Fast loop without Docker, on the Mac host:
  `PYTHONPATH=src/table_sim:src/adaptive_strategy python3 -m table_sim.headless --brain adaptive_strategy.core:StrategyCore --params src/adaptive_strategy/config/strategy.yaml --opponent mirror --runs 1`
  Use `--opponent all --runs 3` (~150 s) for phase sign-off.

## Game rules (authoritative, from me, not yet in the code)
- Capacity: 9 boxes (3 faces × 3). One pick puts a whole stack on one face.
- One construction per drop zone:
  - **wall** = stack of 3 horizontal boxes on Y1–Y5: 3 + 5 = 8 points.
  - **tower** = 1 vertical box on YC1–YC4: 1 + 5 = 6 points.
  - **door** = one per match, on any of Y1–Y5, built with the robot's flip
    face (2 vertical posts, back out, flip the last box horizontal, move
    forward, drop the lid): **23 points total**.
- 1 point per box in a drop zone. Extra boxes in a zone that already has its
  construction score **+1 each, no second bonus**.
- Drop constraints:
  1. If the first drop is Y1, the next drop must be Y3, with no pick in between.
  2. If the first drop is Y2, the next drop must be Y4, with no pick in between.
  3. YC1 needs Y1 and Y3 dropped, and P1 picked.
  4. YC4 needs Y2 and Y4 dropped, and P2 picked.
  5. Y5 is the last drop of the match, made from inside the castle, then park.
- YC4 is in play again.

## Phase 0: make the sim match the real rules (do this now)
1. Rewrite `World.score()` and the `field_2027.yaml` `scoring:` block to the
   rules above. Track per zone which construction was completed first (wall,
   tower or door) so only that one earns its bonus. Let rect zones accept extra
   boxes past 3 (make the limit configurable) at +1 each. Keep the parking
   points as they are.
2. Update every consumer of the score dict: `sim_node.py` `/sim/score`,
   `sim_panel.py`, `headless.py` (table and summary), and `render.py` /
   `replay_match.py` if they read it. `grep -rn "full_stacks\|full_stack_bonus\|box_in_zone"` first.
3. Remove YC4 from `excluded_drop_zones` in `strategy.yaml`.
4. Y5 dock pose from inside the castle is **already set** in `dock_poses.json`
   (x 0.670, y 1.001, yaw 0°, approach 180°). It's checked: it resolves to
   face 0 and the pre-pose (0.47, 1.001) is inside the castle. Once the stall in
   step 6 is fixed, confirm in a match that the Y5 drop actually lands.
5. Add small unit tests for the scoring: wall = 8, wall + 1 extra = 9,
   tower = 6, door = 23, door zone + extra box = 24, second wall attempt in the
   same zone scores +1 per box only. Drive `World` directly (no ROS), next to
   any existing tests or in a new `src/table_sim/test/`.
6. Rerun the baseline (`--opponent all --runs 3`) under the new scoring. The last
   baseline (old scoring) **parked 0/21** and stalled after Y1/Y3 with 9 boxes
   still carried. Find out why using `-v` and the brain log. Report the cause.
   Fix it only if it's small and clearly a bug; otherwise tell me first. The
   baseline has to be a fair benchmark.

Then stop and show me: the test output, the before/after baseline table, and
the stall diagnosis.

## Next phases (after I sign off; details in the spec)
- **Phase 1:** `PlanState` + a fast abstract forward simulator (pure, no physics,
  same legality and scoring as `world.py`) + a cached travel-time matrix built on
  `gridplan.path_length_estimate`. Unit-test the simulator against `World`
  transitions.
- **Phase 2:** drop rules as data (`drop_rules:` guards in `strategy.yaml`,
  replacing the fixed `drop_sequence`), plus door execution in the brain
  (posts → backout → elevate flip → forward → lid) reusing `StrategyCore`'s
  execution layer.
- **Phase 3:** beam search (start at beam 150, depth 15) + lexicographic
  evaluator (P(points ≥ threshold), E[points], time, risk) + dominance pruning,
  with a static opponent. It's a new brain class with the same
  `step(obs, dt) -> Command` interface, runnable through `headless.py --brain`.
  Receding horizon: replan on events, execute only the first action.
- **Phases 4–7:** opponent scenarios, full evaluation, ROS check, profiling.

Acceptance for replacing the baseline: the planner beats it on
`--opponent all --runs 3` under the new scoring, parks in every match, and
never violates a drop constraint.
