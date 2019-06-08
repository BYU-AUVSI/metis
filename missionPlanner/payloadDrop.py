#!/usr/bin/python

import rospy
from rosplane_msgs.msg import Waypoint
import numpy as np

class PayloadDrop():
    def __init__(self):
        self.dropWaypoint = rospy.get_param('DROP_LOCATION')    # the drop waypoint
        self.nextWaypoint = False                               # true if the next waypoint is the drop waypoint
        self.threshold = 2.0

        rospy.Subscriber('/current_waypoint',Waypoint,self.check)
        """
        Testing:
        rostopic pub /current_waypoint rosplane_msgs/Waypoint "{w:[0.0, 0.0, 0.0]}"
        """


    def check(self,waypoint_msg):
        self.dropWaypoint = rospy.get_param('DROP_LOCATION')
        currentWaypoint = list(waypoint_msg.w);
        print("drop waypoint = ",self.dropWaypoint)
        print("current waypoint =",currentWaypoint)

        if self.nextWaypoint:
            if not (np.linalg.norm(np.subtract(currentWaypoint,self.dropWaypoint)) < self.threshold):
                ii = 0
                while ii < 10:
                    # drop the payload!
                    print("\n\n\n\n\n  WE HAVE ATTEMPTED PAYLOAD DROP!!!   \n\n\n\n\n")
                    rospy.ServiceProxy('arm payload',arm_bomb) and rospy.ServiceProxy('drop payload',actuate_drop_bomb)
                    ii += 1
        else:
            if np.linalg.norm(np.subtract(currentWaypoint,self.dropWaypoint)) < self.threshold:
                print("\n\n\n\n\n  PAYLOAD DROP IS UP NEXT!!! \n\n\n\n\n")
                self.nextWaypoint = True



#Run the main planner
if __name__ == "__main__":

    rospy.init_node('paylod_drop', anonymous=True)
    paylodDrop = PayloadDrop()
    while not rospy.is_shutdown():
        rospy.spin()
