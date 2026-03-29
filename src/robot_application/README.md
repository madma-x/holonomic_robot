# Robot Application Package

High-level mission control and behavior coordination for autonomous robot operations.

## Overview

This package provides:
- **Mission Controller**: Behavior tree executor for complex task sequencing
- **Pre-built Missions**: Patrol, pick-and-place, teleop modes
- **Action Clients**: Integration with Nav2 navigation and robot actuators
- **State Management**: Mission monitoring, recovery behaviors, and logging

## Architecture

```
┌─────────────────────────────────────────────────────┐
│            robot_application Package                │
├─────────────────────────────────────────────────────┤
│                                                     │
│  ┌──────────────────────────────────────────────┐  │
│  │         Mission Controller Node              │  │
│  │  - Behavior Tree Executor                    │  │
│  │  - State Machine Management                  │  │
│  │  - Recovery Behaviors                        │  │
│  └───────────────┬──────────────────────────────┘  │
│                  │                                  │
│                  ▼                                  │
│  ┌──────────────────────────────────────────────┐  │
│  │         Action Client Layer                  │  │
│  ├──────────────────────────────────────────────┤  │
│  │  - NavigateToPose (Nav2)                     │  │
│  │  - FollowPath (Nav2)                         │  │
│  │  - MoveServo (robot_actuators)               │  │
│  │  - ControlPump (robot_actuators)             │  │
│  │  - ControlGripper (robot_actuators)          │  │
│  └──────────────────────────────────────────────┘  │
│                                                     │
└─────────────────────────────────────────────────────┘
```

## Dependencies

### Python Packages
```bash
pip3 install py_trees py_trees_ros
```

### ROS2 Packages
- `nav2_msgs`: Navigation action interfaces
- `robot_actuators`: Actuator control actions
- `geometry_msgs`, `nav_msgs`: Standard messages

## Usage

### Launch Mission Controller

```bash
# Launch with patrol mission
ros2 launch robot_application patrol_mission.launch.py

# Launch with pick-place mission
ros2 launch robot_application pick_place_mission.launch.py

# Launch with teleop mode
ros2 launch robot_application teleop_mission.launch.py
```

### Run Missions Manually

```bash
# Start mission controller node
ros2 run robot_application mission_controller.py

# Load and execute patrol mission
ros2 service call /load_mission std_srvs/srv/SetBool "{data: true}"
ros2 service call /start_mission std_srvs/srv/Trigger
```

### Monitor Mission Status

```bash
# Check mission state
ros2 topic echo /mission_status

# View behavior tree status
ros2 topic echo /behavior_tree_status
```

## Missions

### 1. Patrol Mission (`patrol_mission.py`)

**Description**: Autonomous patrol of predefined waypoints with obstacle avoidance.

**Behavior:**
1. Load patrol waypoints from config
2. Navigate to each waypoint in sequence
3. Pause at each waypoint (configurable duration)
4. Repeat indefinitely or for N cycles
5. Return to home on completion

**Configuration**: `config/patrol_waypoints.yaml`

```yaml
patrol_waypoints:
  - {x: 1.0, y: 0.0, theta: 0.0}
  - {x: 1.0, y: 1.0, theta: 1.57}
  - {x: 0.0, y: 1.0, theta: 3.14}
  - {x: 0.0, y: 0.0, theta: 0.0}
  pause_duration_sec: 2.0
  num_cycles: -1  # -1 = infinite
```

### 2. Pick-and-Place Mission (`pick_place_mission.py`)

**Description**: Autonomous object manipulation with navigation and actuator control.

**Behavior:**
1. Navigate to object location
2. Align robot for pickup
3. Lower gripper/activate pump
4. Grasp object (servo or vacuum)
5. Navigate to target location
6. Release object
7. Return to home

**Configuration**: `config/pick_place_tasks.yaml`

### 3. Teleop Mission (`teleop_mission.py`)

**Description**: Manual control with augmented safety features.

**Behavior:**
1. Accept joystick/keyboard commands
2. Enforce velocity limits
3. Enable emergency stop on timeout
4. Allow manual actuator control

## Behavior Trees

### Patrol Behavior Tree (`behavior_trees/patrol.xml`)

```xml
<root>
  <BehaviorTree ID="PatrolMission">
    <Sequence>
      <Action ID="LoadWaypoints"/>
      <Repeat num_cycles="-1">
        <Sequence>
          <ForEach waypoints="patrol_waypoints">
            <Sequence>
              <Action ID="NavigateToWaypoint" waypoint="{waypoint}"/>
              <Action ID="PauseAtWaypoint" duration="2.0"/>
            </Sequence>
          </ForEach>
        </Sequence>
      </Repeat>
    </Sequence>
  </BehaviorTree>
</root>
```

### Pick-Place Behavior Tree (`behavior_trees/pick_place.xml`)

See XML file for complete definition.

## Custom Behavior Tree Nodes

### Navigation Nodes
- `NavigateToWaypoint`: Navigate to specific pose using Nav2
- `FollowPath`: Follow predefined path
- `CheckLocalization`: Verify localization quality

### Actuator Nodes
- `MoveServoAction`: Control servo position
- `ActivatePump`: Turn pump on/off
- `GripObject`: Gripper open/close/force control

### Condition Nodes
- `IsAtPosition`: Check if robot at target
- `IsBatteryLow`: Check battery level
- `IsObjectDetected`: Check vision system

### Decorator Nodes
- `RetryUntilSuccess`: Retry action with backoff
- `Timeout`: Abort action after duration
- `RateLimiter`: Limit action execution rate

## API

### Services

#### `/load_mission` (std_srvs/srv/SetBool)
Load mission configuration.

#### `/start_mission` (std_srvs/srv/Trigger)
Start mission execution.

#### `/pause_mission` (std_srvs/srv/Trigger)
Pause current mission.

#### `/stop_mission` (std_srvs/srv/Trigger)
Stop and reset mission.

### Topics

#### `/mission_status` (std_msgs/msg/String)
Current mission state (IDLE, RUNNING, PAUSED, COMPLETED, FAILED).

#### `/behavior_tree_status` (std_msgs/msg/String)
Behavior tree execution status.

#### `/mission_progress` (std_msgs/msg/Float32)
Mission completion percentage (0-100%).

## Configuration Files

### `config/mission_controller.yaml`

```yaml
mission_controller:
  ros__parameters:
    # Behavior tree settings
    bt_update_rate_hz: 10.0
    bt_default_timeout_sec: 30.0
    
    # Recovery behaviors
    enable_recovery: true
    max_recovery_attempts: 3
    
    # Navigation timeouts
    nav_timeout_sec: 120.0
    
    # Actuator timeouts
    servo_timeout_sec: 10.0
    pump_timeout_sec: 30.0
```

## Development

### Creating Custom Missions

1. Create new Python script in `scripts/`
2. Inherit from `MissionBase` class
3. Implement behavior tree structure
4. Add launch file in `launch/`

Example:
```python
from robot_application.mission_base import MissionBase

class CustomMission(MissionBase):
    def __init__(self):
        super().__init__('custom_mission')
        self.build_behavior_tree()
    
    def build_behavior_tree(self):
        # Define your behavior tree here
        pass
```

### Adding Custom Behavior Nodes

Create node in `robot_application/behaviors/`:

```python
import py_trees

class CustomAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
    
    def update(self):
        # Implement behavior logic
        return py_trees.common.Status.SUCCESS
```

## Safety Features

### Emergency Stop
All missions respond to emergency stop service:
```bash
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

### Watchdog Timer
Mission controller monitors:
- Navigation progress
- Action completion
- Communication timeouts

### Recovery Behaviors
Automatic recovery from:
- Navigation failures → Re-plan path
- Localization loss → Rotate and relocalize
- Actuator timeouts → Retry with safety checks

## Troubleshooting

### Mission Won't Start
- Check Nav2 stack is running: `ros2 topic list | grep navigate`
- Verify actuator nodes: `ros2 node list | grep controller`
- Check localization: `ros2 topic echo /amcl_pose`

### Navigation Failures
- Verify map loaded: `ros2 topic echo /map -n 1`
- Check costmaps: `rviz2` → Add → CostMap
- Increase timeout: Edit `config/mission_controller.yaml`

### Actuator Not Responding
- Test action server: `ros2 action list`
- Send manual goal: `ros2 action send_goal /move_servo ...`
- Check hardware connections and GPIO permissions

## Future Enhancements

- [ ] Vision-based object detection integration
- [ ] Multi-robot coordination
- [ ] Machine learning for adaptive behaviors
- [ ] Mission planning GUI
- [ ] Data logging and replay
- [ ] Simulation mode for testing

## License

Apache 2.0
