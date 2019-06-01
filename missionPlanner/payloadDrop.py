#!/usr/bin/python

import rospy
from rosplane_msgs.msg import Waypoint

class PayloadDrop():
    def __init__(self):
        self.dropWaypoint = rospy.get_param('DROP_LOCATION')    # the drop waypoint
        self.nextWaypoint = False                               # true if the next waypoint is the drop waypoint

        rospy.Subscriber('/current_waypoint',Waypoint,self.check)



    def check(self,waypoint_msg):
        self.dropWaypoint = rospy.get_param('DROP_LOCATION')
        currentWaypoint = list(waypoint_msg.w);
        print("drop waypoint = ",self.dropWaypoint)
        print("current waypoint =",currentWaypoint)

        if self.nextWaypoint:
            print("\n\n\n\n\n  PAYLOAD DROP IS UP NEXT!!! \n\n\n\n\n")
            if not (all(currentWaypoint == dropWaypoint)):
                # drop the payload!
                print("\n\n\n\n\n  WE HAVE ATTEMPTED PAYLOAD DROP!!!   \n\n\n\n\n")
                rospy.ServiceProxy('arm payload',arm_bomb) and rospy.ServiceProxy('drop payload',actuate_drop_bomb)

        else:
            if currentWaypoint == self.dropWaypoint:
                self.nextWaypoint = True



#Run the main planner
if __name__ == "__main__":

    rospy.init_node('paylod_drop', anonymous=True)
    paylodDrop = PayloadDrop()
    while not rospy.is_shutdown():
        rospy.spin()
