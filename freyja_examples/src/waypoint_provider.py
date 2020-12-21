#!/usr/bin/python

import rospy
import sys
import time

from enum import Enum
from freyja_msgs.msg import WaypointTarget

class mode_enum(Enum):
  TIME    = 0
  SPEED   = 1


class WaypointTester:
    def __init__(self):
        # Init the ROS node
        rospy.init_node('waypoint_testing_node')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the rate
        self.rate = 10.0
        self.dt = 1.0 / self.rate

        # Init the drone and program state
        self._quit = False

        # Get the mode param
        mode_param = rospy.get_param(rospy.get_name() + '/waypoint_mode', default=0)

        self.mode = None
        if mode_param == mode_enum.TIME.value:
          self.mode = mode_enum.TIME
        elif mode_param == mode_enum.SPEED.value:
          self.mode = mode_enum.SPEED
        else:
          self._log("Unknown waypoint mode.")
          self._quit = True

        # Init all the publishers and subscribers
        self.attitude_pub = rospy.Publisher("/waypoint_state", WaypointTarget, queue_size=10)
        
        time.sleep(2)
        self.start()

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _mainloop(self):

        previous_state = None
        r = rospy.Rate(self.rate)

        sent = False

        while not self._quit:
        
            if not sent:
                # Create message
                msg = WaypointTarget()
                msg.terminal_pn = 0.0
                msg.terminal_pe = 0.0
                msg.terminal_pd = -10.0
                msg.terminal_vn = 0.0
                msg.terminal_ve = 0.0
                msg.terminal_vd = 0.0
                msg.terminal_yaw = 0.0

                msg.allocated_time = 10.0
                msg.translational_speed = 1.0

                msg.waypoint_mode = self.mode.value

                self.attitude_pub.publish(msg)
                self._log("Waypoint sent")
                sent = True

            # Mantain the rate
            r.sleep()

if __name__ == "__main__":
    try:
        x = WaypointTester()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)
