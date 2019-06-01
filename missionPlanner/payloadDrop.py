import rospy
from rosplane_msgs.msg import Waypoint



class PayloadDrop():
    def __init__(self):
        self.dropWaypoint = rospy.get_param('DROP_LOCATION')    # the drop waypoint
        self.nextWaypoint = False                               # true if the next waypoint is the drop waypoint

        rospy.init_node('payload_drop')
        rospy.Subscriber('/current_waypoint',Waypoint,self.check)

    def check(self,waypoint_msg):
        self.dropWaypoint = rospy.get_param('DROP_LOCATION')
        currentWaypoint = waypoint_msg.w;

        if self.nextWaypoint:
            if not (all(currentWaypoint == dropWaypoint)):
                # drop the payload!
                rospy.ServiceProxy('arm payload',arm_bomb) && rospy.ServiceProxy('drop payload',actuate_drop_bomb)
        else:
            if all(currentWaypoint == dropWaypoint):
                self.nextWaypoint = True
