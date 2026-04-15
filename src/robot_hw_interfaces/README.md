# robot_hw_interfaces

Minimal ROS2 interfaces for hardware architecture:
- Safety state
- Pump commands and state
- PWM servo commands (command-only)
- Feetech servo commands and state

## Topics (intended)
- /safety_state: robot_hw_interfaces/msg/SafetyState
- /pump_cmd: robot_hw_interfaces/msg/PumpCommand
- /pump_state: robot_hw_interfaces/msg/PumpState
- /pwm_servo_cmd: robot_hw_interfaces/msg/PwmServoCommand
- /servo_cmd: robot_hw_interfaces/msg/ServoCommand
- /servo_state: robot_hw_interfaces/msg/ServoState
