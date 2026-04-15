# One step in an actuator sequence.
# Fill only the fields that apply to step_type.

uint8 MOVE_SERVO     = 0
uint8 CONTROL_PUMP   = 1
uint8 MOVE_PWM_SERVO = 2

uint8 step_type

# parallel_group: 0 = sequential (run alone).
# Steps sharing the same non-zero value in consecutive positions
# are dispatched simultaneously; the sequencer waits for all of
# them before advancing.
uint8 parallel_group

# --- MOVE_SERVO fields (Feetech USB serial) ---
uint8   servo_id
float32 target_deg
float32 speed_deg_s     # 0 → use servo default max speed

# --- CONTROL_PUMP fields ---
uint8   pump_id
bool    pump_enable
float32 pump_duration_sec   # 0.0 = indefinite (pump stays on until next command)

# --- MOVE_PWM_SERVO fields (PCA9685 channel, fire-and-forget) ---
uint8   pwm_channel
float32 pwm_target_deg
float32 pwm_settle_sec  # expected travel time; sequencer waits this long before proceeding
