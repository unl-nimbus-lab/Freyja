# Discrete waypoint for a waypoint handler to convert into ReferenceState.

std_msgs/Header header
float64 terminal_pn
float64 terminal_pe
float64 terminal_pd
float64 terminal_vn
float64 terminal_ve
float64 terminal_vd
float64 terminal_yaw

# time allocated to travel from 'here' to the target
float32 allocated_time
# use a constant speed instead of allocated_time(see flag below)
float32 translational_speed

# use allocated_time OR use translational_speed
uint8 MODE_TIME  = 0
uint8 MODE_SPEED = 1
uint8 waypoint_mode

# define manager's behaviour once the waypoint is done
uint8 HOVER = 0
uint8 HOLD = 1
uint8 HOVER_STOPPUB = 2
uint8 HOLD_STOPPUB = 3
uint8 after_waypoint_done
