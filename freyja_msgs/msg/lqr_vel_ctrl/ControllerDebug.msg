# Debug message for controller.
# State can be invalid due to time delay, or a dead source.

Header header
float64[4] lqr_u
float64 thrust
float64 roll
float64 pitch
float64 yaw
uint8 state_valid
