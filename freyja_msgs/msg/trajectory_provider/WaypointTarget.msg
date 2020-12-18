# Discrete waypoint for a waypoint handler to convert into ReferenceState.

Header header
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
uint8 TIME  = 0
uint8 SPEED = 1
uint8 waypoint_mode