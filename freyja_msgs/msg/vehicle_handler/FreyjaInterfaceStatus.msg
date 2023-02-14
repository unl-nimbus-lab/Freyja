# Custom status msg that keeps track of system status
## Autopilot handlers must fill these using their own
## communication interfaces. Not all information may
## be available to a handler. Fill -1 if so.

# No internal component of Freyja should depend on
# this msg, i.e., no state machine transitions are
# effected through this. The primary purpose of this
# msg is to ease log analysis. External tools/code
# are, of course, free to consume it however.

std_msgs/Header  header
bool    connected       # comms to vehicle
bool    armed           # motors on
bool    computer_ctrl   # mode == computer
# extra rc channels (as boolean (>1500))
bool    aux1
bool    aux2
bool    aux3
bool    aux4

