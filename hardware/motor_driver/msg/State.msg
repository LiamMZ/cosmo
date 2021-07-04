float64[] horizontal_velocity
float64 yaw_rate
float64 height
float64 pitch
float64 roll
float64 yaw
int8 activation
int8 behavior_state

uint64 ticks
geometry_msgs/Point[] foot_locations

float64[] joint_angles # length 12 vector 4x {abduction_angle, hip_angle, knee_angle}

# enum equivalent for behavior states
int8 DEACTIVATED = -1
int8 REST = 0
int8 TROT = 1
int8 HOP = 2
int8 FINISHHOP = 3