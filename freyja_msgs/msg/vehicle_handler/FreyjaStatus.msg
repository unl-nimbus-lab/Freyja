# Custom status msg that keeps track of system status
## Autopilot handlers must fill these using their own
## communication interfaces. Not all information may
## be available to a handler. Fill -1 if so.

# No internal component of Freyja should depend on
# this msg, i.e., no state machine transitions are
# effected through this. The primary purpose of this
# msg is to ease log analysis. External tools/code
# are, of course, free to consume it however.

Header  header
int8    is_connected    # comms to vehicle
int8    is_armed        # motors on
int8    is_compctrl     # mode == computer
