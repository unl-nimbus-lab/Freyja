#!/usr/bin/python

import rospy
import sys
import time

from geometry_msgs.msg import PointStamped
from freyja_msgs.msg import ReferenceState

class ReferenceToPoint:
    def __init__(self):
        # Init the ROS node
        rospy.init_node('reference_to_point_node')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the rate
        self.rate = 10.0
        self.dt = 1.0 / self.rate

        # Init the drone and program state
        self._quit = False

        # Init all the publishers and subscribers
        self.ref_sub = rospy.Subscriber("/reference_state", ReferenceState, self.reference_callback)
        self.ref_pub = rospy.Publisher("/reference_point", PointStamped, queue_size=10)
        
        time.sleep(0.5)
        self.start()

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def reference_callback(self, msg):
        msg_out = PointStamped()

        msg_out.header.seq = 1
        msg_out.header.stamp = rospy.Time.now()
        msg_out.header.frame_id = "map"

        msg_out.point.x = msg.pn
        msg_out.point.y = msg.pe
        msg_out.point.z = -1 * msg.pd

        self.ref_pub.publish(msg_out)

    def _mainloop(self):

        previous_state = None
        r = rospy.Rate(self.rate)

        while not self._quit:

            # Mantain the rate
            r.sleep()

if __name__ == "__main__":
    try:
        x = ReferenceToPoint()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)
