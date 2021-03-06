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
        self.threshold = 2.0                                    # threshold between drop location parameter and rosplane waypoint
        self.ARMED_AND_READY = False                            # rosparam must be changed in order for bomb to drop
        rospy.Subscriber('/current_waypoint',Waypoint,self.check)# subscribes to current waypoint from rosplane

    def check(self,waypoint_msg):
        self.dropWaypoint = rospy.get_param('DROP_LOCATION')    # get the current drop waypoint from the payload planner
        try:
            # if the payload planner hasn't been run yet, the 'DROP_LOCATION' is a string
            print(np.array(self.dropWaypoint)[0]) # this fails if the drop wayopint is seen as a string
        except:
            try:
                print("dropWaypoint is a string")
                # if it's seen as a string than make it a list
                self.dropWaypoint = eval(rospy.get_param('DROP_LOCATION'))
            except:
                # if those didn't work just assume it's not there and make the drop far away
                print("using defualt drop location value!")
                self.dropWaypoint = [99999.9,9999.9,99999.9]

        print("drop waypoint = ",self.dropWaypoint)             # payload drop waypoint calculated by payload planner
        currentWaypoint = list(waypoint_msg.w);                 # current waypoint the plane is headed to
        print("current waypoint =",currentWaypoint)             # current waypoint the plane is headed to
        print("distance to drop location = ", np.linalg.norm(np.subtract(currentWaypoint,self.dropWaypoint)));

        # this ros parameter must be changed manually to arm by running:
        # `rosparam set /ARMED_AND_READY "true"`
        # this parameter should be set to false to prevent random dropping:
        # `rosparam set /ARMED_AND_READY "false"`
        self.ARMED_AND_READY = rospy.get_param('ARMED_AND_READY') # get the current value of this rosparam


        if self.ARMED_AND_READY:
            print("Watch out, I'm armed!!")
        else:
            print("I'm not armed!")

        if self.nextWaypoint:
            # if we're headed to the drop location, check to see if we've moved on to the next waypoint
            if not (np.linalg.norm(np.subtract(currentWaypoint,self.dropWaypoint)) < self.threshold):

                # if we moved to the waypoint past the drop waypoint and we're armed, drop it!
                if self.ARMED_AND_READY:
                    ii = 0
                    # send the command 10 times just to be safe
                    while ii < 10:
                        # drop the payload!
                        print("\n\n\n\n\n  WE HAVE ATTEMPTED PAYLOAD DROP!!!   \n\n\n\n\n")

                        # 'cause we couldn't get a real service call to work
                        subprocess.call("rosservice call /gpio_0_pulse_actuate", shell=True)
                        # rospy.ServiceProxy('arm_bomb', std_srvs.srv.Trigger)
                        # rospy.ServiceProxy('actuate_drop_bomb', std_srvs.srv.Trigger)
                        ii += 1
                else:
                    print("\n\n\n\n\n  DID NOT DROP BECAUSE NOT ARMED AND READY!!!   \n\n\n\n\n")

                self.nextWaypoint = False # no longer attempting drop

        # check to see if the waypoint we're headed towards is the drop location waypoint
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
