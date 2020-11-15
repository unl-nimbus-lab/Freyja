# Control commands for the attitude controller.
# Need to translate differently for AscTec HLP/LLP, PX4, AC etc.
# Attitude in radians, thrust in *Newtons*. No scaling!
# ctrl_mode is:
#    8 bits (asctec): [ x x gps height thrust yaw roll pitch ]
#    8 bits (apm)   : [ x x x x x x yawrate ctrlvalid]
float64 roll
float64 pitch
float64 yaw
float64 thrust
uint8 ctrl_mode
