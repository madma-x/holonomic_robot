# Simplified Robot Application Architecture

## Key Principle: Separation of Concerns

**Navigation & Obstacle Avoidance:** Let Nav2 handle it  
**Task Strategy & Switching:** TaskPlanner handles it

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                      TaskPlanner                             │
│  - Dynamic task prioritization (utility-based)              │
│  - Game phase management (EARLY/MID/LATE/ENDGAME)           │
│  - Task execution sequencing                                 │
│  - Monitor goal outcomes (SUCCEEDED, UNREACHABLE, TIMEOUT)   │
│  - Switch tasks on goal failure                              │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
                   ┌───────────────────────┐
                   │  Mission Executor     │
                   │  (PatrolMission,      │
                   │   PickPlaceMission)   │
                   └───────────┬───────────┘
                               │
                               ▼
                   ┌───────────────────────┐
                   │  Nav2 NavigateToPose  │
                   │  Action Server        │
                   └───────────┬───────────┘
                               │
        ┌──────────────────────┼──────────────────────┐
        │                      │                      │
        ▼                      ▼                      ▼
   MPPI Local       Costmap Dynamic         BehaviorTree
   Planner          Update                  (replanning)
   - Obstacle       - Sensor fusion         - Local avoidance
   avoidance        - Collision checks      - Goal sequencing
```

## What Changed: Removed Redundancy

### ❌ REMOVED: ObstacleMonitor
- **Why:** Nav2's costmap-based replanning already detects obstacles
- **Result:** Fewer topics, simpler architecture

### ❌ REMOVED: TaskRecovery Strategies  
- **Why:** Nav2 retries paths automatically; only high-level task switching needed
- **Result:** TaskPlanner logic simplified, fewer callbacks

### ✅ SIMPLIFIED: TaskPlanner
- Removed blockage_callback, recovery_suggestion_callback, handle_blockage_callback
- Removed recovery_manager, blockage tracking state
- Removed critical blockage handling
- Kept high-level task prioritization and switching

## Information Flow

### 1. Task Execution Path
```
TaskPlanner selects task
    ↓
Mission executes (dispatch to Nav2 goal)
    ↓
Nav2 NavigateToPose action processes goal
    ↓
MPPI Local Planner + Costmap handle obstacle avoidance
    ↓
[Goal succeeded]
    ↓
TaskPlanner marks task completed, moves to next task

[Goal failed: UNREACHABLE or TIMEOUT after Nav2 retries]
    ↓
TaskPlanner catches goal failure
    ↓
Move task to failed list, switch to next task
```

### 2. Obstacle Handling (Automatic)
```
Sensor detects obstacle
    ↓
Costmap updates (built into Nav2)
    ↓
MPPI Planner replans path locally
    ↓
Robot executes new path
    ↓
[If obstacle persists after N retries]
    ↓
Nav2 returns UNREACHABLE
    ↓
TaskPlanner switches tasks
```

## Topic List (Simplified)

### Input Topics
- `/game/time_remaining` (Float32) - Match timer
- `/game/score` (Int32) - Current score
- `/game/opponent_score` (Int32) - Opponent score
- `/game/phase` (String) - Game phase (EARLY/MID/LATE/ENDGAME)
- `/amcl_pose` (PoseStamped) - Robot localization

### Output Topics
- `/planner/current_task` (String) - Currently executing task
- `/planner/queue_size` (Int32) - Remaining tasks in queue

### Action Topics
- `/navigate_to_pose` (NavigateToPose) - Nav2 goal dispatch
- `/servo_controller/move_servo` (MoveServo) - Servo control
- `/pump_controller/operate_pump` (OperatePump) - Pump control

## Service List (Simplified)

- `/planner/start` (Trigger) - Begin strategic planning
- `/planner/stop` (Trigger) - Stop planning and executing
- `/planner/replan` (Trigger) - Force immediate re-evaluation

## Configuration Files

- `game_state.yaml` - Match duration, phase transition times
- `task_planner.yaml` - Replan interval, utility threshold, task library
- `mission_controller.yaml` - Nav2 goal timeout, retry parameters

## Task Prioritization

Each task has utility calculated based on:
1. **Base priority** - Task importance (pick_cube: 10, steal_cube: 25)
2. **Base points** - Scoring reward
3. **Time estimate** - Duration to complete
4. **Success probability** - Historical success rate
5. **Game phase** - Phase-specific weighting

**Utility function** (game-phase dependent):
```python
utility = base_priority + (base_points / time_remaining) * phase_weight
```

## Error Handling

### Task Level
- **Goal UNREACHABLE:** Remove from queue, switch to next task
- **Goal TIMEOUT:** Remove from queue after N Nav2 retries, switch task
- **Task execution error:** Mark failed, move to next task

### System Level
- **No tasks available:** Wait and replan
- **Low utility:** Keep waiting, update priorities every 5s
- **Match timer expired:** Stop planning, return to base

## Deployment Checklist

- [ ] Nav2 configured with MPPI controller
- [ ] Costmap parameters tuned for obstacle detection
- [ ] GameStateManager broadcasting match state
- [ ] Mission executors linked to NaviageToPose action
- [ ] Actuator action servers running (servo, pump)
- [ ] Task planner configuration loaded

## Next Steps

1. **Vision Integration:** Add `robot_vision` package for AprilTag detection
2. **Mission Implementation:** Link TaskPlanner → actual mission execution
3. **Hardware Deployment:** Test on Jetson Orin Nano + STM32 hardware
4. **Nav2 Tuning:** Adjust MPPI parameters for specific environment

## Rationale

This simplified architecture:
- ✅ Leverages Nav2's proven replanning algorithms
- ✅ Reduces code complexity and bugs
- ✅ Clearer separation: Nav2 = navigation, TaskPlanner = strategy
- ✅ Faster iteration on task logic without worrying about obstacle handling
- ✅ Scales better: Easy to add new tasks or game phases
