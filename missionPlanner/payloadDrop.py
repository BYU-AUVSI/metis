import rospy

class PayloadDrop():
    def __init__(self,dropWaypoint):
        self.dropWaypoint = dropWaypoint    # the drop waypoint
        self.nextWaypoint = False           # true if the next waypoint is the drop waypoint

    def check(currentWaypoint):
        if self.nextWaypoint:
            if not (all(currentWaypoint == dropWaypoint)):
                # drop the payload!
                rospy.ServiceProxy('arm payload',arm_bomb) && rospy.ServiceProxy('drop payload',actuate_drop_bomb)
        else:
            if all(currentWaypoint == dropWaypoint):
                self.nextWaypoint = True
