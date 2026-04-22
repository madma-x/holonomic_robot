std_msgs/Header header

uint8 SET_POSITION=0
uint8 SET_TORQUE=1
uint8 REINIT=2

uint8 servo_id
uint8 command_type
float32 target_deg
float32 speed_deg_s
bool torque_enable
