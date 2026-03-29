# Strategic Planning System - Quick Start

## Overview

The strategic planning system dynamically prioritizes tasks based on:
- **Time remaining** in match
- **Current score** vs opponent
- **Game phase** (early/mid/late/endgame)
- **Task characteristics** (points, time, success probability)

## Architecture

```
Game State Manager → Task Planner → Mission Executor
       ↓                  ↓              ↓
   Time/Score      Task Queue      Nav2/Actuators
```

## Usage

### 1. Launch Strategic Planner

```bash
# Start game state manager and task planner
ros2 launch robot_application strategic_planner.launch.py

# With custom match duration (120 seconds)
ros2 launch robot_application strategic_planner.launch.py match_duration_sec:=120.0

# Auto-start match on launch
ros2 launch robot_application strategic_planner.launch.py auto_start:=true
```

### 2. Control Match

```bash
# Start match timer
ros2 service call /game/start_match std_srvs/srv/Trigger

# Stop match
ros2 service call /game/stop_match std_srvs/srv/Trigger

# Add score points
ros2 service call /game/add_score std_srvs/srv/SetBool "{data: true}"  # 10 points
ros2 service call /game/add_score std_srvs/srv/SetBool "{data: false}" # 5 points
```

### 3. Control Task Planner

```bash
# Start task planning and execution
ros2 service call /planner/start std_srvs/srv/Trigger

# Stop task planning
ros2 service call /planner/stop std_srvs/srv/Trigger

# Force immediate replanning
ros2 service call /planner/replan std_srvs/srv/Trigger
```

### 4. Monitor Status

```bash
# Watch game state
ros2 topic echo /game/time_remaining
ros2 topic echo /game/score
ros2 topic echo /game/phase

# Watch task planner
ros2 topic echo /planner/current_task
ros2 topic echo /planner/queue_size
```

## Task Definitions

### Creating Custom Tasks

```python
from robot_application.task_definitions import Task, TaskType

# Define a custom task
custom_task = Task(
    task_id="my_task_1",
    task_type=TaskType.PICK_CUBE,
    name="Pick High Value Cube",
    base_points=25.0,           # Points awarded
    time_estimate=15.0,         # Expected time (seconds)
    success_probability=0.85,   # 85% success rate
    base_priority=7,            # Priority 1-10
    target_location={'x': 1.5, 'y': 0.5, 'theta': 0.0}
)
```

### Priority Functions by Game Phase

**Early Game (>60s):**
- Focus: High-value tasks, strategic positioning
- Strategy: Build score foundation, low risk

**Mid Game (30-60s):**
- Focus: Balanced scoring and defense
- Strategy: Adapt to score differential

**Late Game (10-30s):**
- Focus: Quick wins, time-efficient tasks
- Strategy: Maximize points/second

**Endgame (<10s):**
- Focus: Return to base, secure score
- Strategy: No new tasks unless <3s duration

## Utility Calculation

Task utility determines execution order:

```
Utility = (Points × Success_Prob × Time_Multiplier) / Time_Cost
```

**Time Multiplier:**
- Can't complete in time: 0.1× (avoid)
- Barely enough time: 0.7× (risky)
- Plenty of time: 1.0× (normal)

**Phase Multipliers:**
- Early: High-value bonus (1.5×), risk penalty (0.8×)
- Mid: Score differential adjustments
- Late: Quick task bonus, slow task penalty
- Endgame: Return to base = 1000× priority

## Task Interruption

If enabled, planner can interrupt current task when:
```
New_Utility > Current_Utility × 1.5
```

Example: Interrupt low-value defense for high-value steal opportunity.

## Example Tasks Included

1. **Pick Cubes** (10 pts, 12s, 85% success)
2. **Place Cubes** (15 pts, 10s, 90% success)
3. **Defend Area** (5 pts, 30s, 95% success)
4. **Steal Cube** (25 pts, 18s, 60% success) - High risk/reward
5. **Return Base** (10 pts, 8s, 98% success) - Endgame priority
6. **Move Object** (12 pts, 20s, 80% success)

## Configuration

### Game State (`config/game_state.yaml`)

```yaml
match_duration_sec: 180.0      # 3 minute match
early_phase_threshold: 60.0    # >60s = early game
mid_phase_threshold: 30.0      # 30-60s = mid game
late_phase_threshold: 10.0     # <10s = endgame
```

### Task Planner (`config/task_planner.yaml`)

```yaml
replan_interval_sec: 5.0           # Replan every 5s
enable_task_interruption: true     # Allow interruptions
min_utility_threshold: 0.5         # Ignore low-utility tasks
base_location_x: 0.0               # Home position
base_location_y: 0.0
```

## Integration with Existing Missions

Tasks can reference existing behavior trees or mission classes:

```python
task = Task(
    # ... task properties ...
    behavior_tree="pick_place.xml",      # Use existing BT
    mission_class="PickPlaceMission",    # Or mission class
    parameters={'method': 'gripper'}     # Custom parameters
)
```

## Topics Reference

### Published by Game State Manager
- `/game/time_remaining` (Float32) - Seconds left
- `/game/score` (Int32) - Current score
- `/game/opponent_score` (Int32) - Opponent score
- `/game/phase` (String) - EARLY/MID/LATE/ENDGAME/FINISHED
- `/game/match_active` (Bool) - Match running status

### Published by Task Planner
- `/planner/current_task` (String) - Currently executing task
- `/planner/queue_size` (Int32) - Number of queued tasks

## Services Reference

### Game State Manager
- `/game/start_match` (Trigger) - Start match timer
- `/game/stop_match` (Trigger) - Stop match
- `/game/add_score` (SetBool) - Add points (data=true: 10pts, false: 5pts)

### Task Planner
- `/planner/start` (Trigger) - Start task execution
- `/planner/stop` (Trigger) - Stop task execution
- `/planner/replan` (Trigger) - Force immediate replanning

## Advanced: Custom Priority Functions

Define phase-specific strategies:

```python
def aggressive_strategy(time_remaining, current_score, game_state, task):
    """High-risk, high-reward strategy."""
    if task.base_points > 20:
        return task.base_points * 2.0  # Prioritize high value
    return task.base_points * 0.5

def defensive_strategy(time_remaining, current_score, game_state, task):
    """Conservative, protect lead."""
    if task.task_type == TaskType.DEFEND_AREA:
        return 100.0  # Maximize defense
    return task.base_points * 0.8

# Assign to task
task.priority_function = aggressive_strategy
```

## Tips for Competition

1. **Early Game**: Focus on point accumulation, avoid risky tasks
2. **Mid Game**: Monitor opponent, adapt strategy
3. **Late Game**: Only quick, high-success tasks
4. **Endgame**: Always reserve time to return to base
5. **Score Differential**: If behind, take calculated risks
6. **Time Management**: Buffer 10-20% time for failures

## Debugging

```bash
# View task queue in real-time
ros2 topic echo /planner/current_task

# Check game phase transitions
ros2 topic echo /game/phase

# Monitor score progression
ros2 topic echo /game/score

# Check planner logs
ros2 run robot_application task_planner.py --ros-args --log-level debug
```
