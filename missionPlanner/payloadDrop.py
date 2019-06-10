#!/usr/bin/python

import rospy
from rosplane_msgs.msg import Waypoint
import numpy as np
import std_srvs.srv
import subprocess

class PayloadDrop():
    def __init__(self):
        self.dropWaypoint = rospy.get_param('DROP_LOCATION')    # the drop waypoint
        self.nextWaypoint = False                               # true if the next waypoint is the drop waypoint
        self.threshold = 2.0
        self.ARMED_AND_READY = False

        rospy.Subscriber('/current_waypoint',Waypoint,self.check)
        """
        Testing:
        rostopic pub /current_waypoint rosplane_msgs/Waypoint "{w:[0.0, 0.0, 0.0]}"
        """


    def check(self,waypoint_msg):
        self.dropWaypoint = rospy.get_param('DROP_LOCATION')
        try:
            print(np.array(self.dropWaypoint)[0]) # this fails if the drop wayopint is seen as a string
        except:
            try:
                print("dropWaypoint is a string")
                # if it's seen as a string than make is a list
                self.dropWaypoint = eval(rospy.get_param('DROP_LOCATION'))
            except:
                # if those didn't work just assume it's not there and make the drop far away
                print("using defualt drop location value!")
                self.dropWaypoint = [99999.9,9999.9,99999.9]

        currentWaypoint = list(waypoint_msg.w);
        print("drop waypoint = ",self.dropWaypoint)
        print("current waypoint =",currentWaypoint)
        print("distance to drop location = ", np.linalg.norm(np.subtract(currentWaypoint,self.dropWaypoint)));

        self.ARMED_AND_READY = rospy.get_param('ARMED_AND_READY')
        if self.ARMED_AND_READY:
            print("Watch out, I'm armed!!")
        else:
            print("I'm not armed!")

        if self.nextWaypoint:
            if not (np.linalg.norm(np.subtract(currentWaypoint,self.dropWaypoint)) < self.threshold):

                if self.ARMED_AND_READY:
                    ii = 0
                    while ii < 10:
                        # drop the payload!
                        print("\n\n\n\n\n  WE HAVE ATTEMPTED PAYLOAD DROP!!!   \n\n\n\n\n")

                        # 'cause we couldn't get a real service call to work
                        subprocess.call("rosservice call /arm_bomb && rosservice call /actuate_drop_bomb", shell=True)
                        # rospy.ServiceProxy('arm_bomb', std_srvs.srv.Trigger)
                        # rospy.ServiceProxy('actuate_drop_bomb', std_srvs.srv.Trigger)
                        ii += 1
                else:
                    print("\n\n\n\n\n  DID NOT DROP BECAUSE NOT ARMED AND READY!!!   \n\n\n\n\n")

                self.nextWaypoint = False # no longer attempting drop
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
