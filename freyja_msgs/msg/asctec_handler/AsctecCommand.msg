# asctec handler common message structure that contains everything commandable
## command_type =   1   :   this message is to turn motors on or off
##                  2   :   this message is to idle the motors
##                  3   :   this message is to command attitudes

uint8 command_type
float64 roll
float64 pitch
float64 yaw
float64 thrust
uint8 ctrl_byte
