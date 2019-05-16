#!/usr/bin/python
import rospy
from uav_msgs.msg import JudgeMission, OrderedPoint, Point, StationaryObstacle
from uav_msgs.srv import GetMissionWithId

def mission_response(req):
    res = JudgeMission
    res.mission_type = req.mission_type
    res.now = False

    #Define the waypoints
    pt1 = OrderedPoint()
    pt1.point = Point(40.267052, -111.636455, 30, 0)
    pt1.ordinal = 1

    pt2 = OrderedPoint()
    pt2.point = Point(40.267625, -111.635216, 45, 0)
    pt2.ordinal = 2

    pt3 = OrderedPoint()
    pt3.point = Point(40.266511, -111.633746, 45, 0)
    pt3.ordinal = 3

    pt4 = OrderedPoint()
    pt4.point = Point(40.266340, -111.635935, 45, 0)
    pt4.ordinal = 4

    pt5 = OrderedPoint()
    pt5.point = Point(40.266815, -111.637083, 45, 0)
    pt5.ordinal = 5

    pt6 = OrderedPoint()
    pt6.point = Point(40.266946, -111.632552, 0, 0)
    pt6.ordinal = 1

    if res.mission_type == JudgeMission.MISSION_TYPE_DROP:
        res.waypoints = [pt1]
    elif res.mission_type == JudgeMission.MISSION_TYPE_SEARCH:
        res.waypoints = [pt1, pt2, pt3, pt4, pt5]
    elif res.mission_type == JudgeMission.MISSION_TYPE_OFFAXIS:
        res.waypoints = [pt6]
    else:
        res.waypoints = [pt1, pt2]

    #Define boundaries

    bnd1 = OrderedPoint()
    bnd1.point = Point(40.268526, -111.634529, 0, 0)
    bnd1.ordinal = 1

    bnd2 = OrderedPoint()
    bnd2.point = Point(40.268350, -111.637608, 0, 0)
    bnd2.ordinal = 2

    bnd3 = OrderedPoint()
    bnd3.point = Point(40.266139, -111.637710, 0, 0)
    bnd3.ordinal = 3

    bnd4 = OrderedPoint()
    bnd4.point = Point(40.266143, -111.632233, 0, 0)
    bnd4.ordinal = 4

    res.boundaries = [bnd1, bnd2, bnd3, bnd4]

    #Define obstacles

    obs1 = StationaryObstacle()
    obs1.point = Point(40.267670, -111.636782, 0, 0)
    obs1.cylinder_height = 60
    obs1.cylinder_radius = 20

    obs2 = StationaryObstacle()
    obs2.point = Point(40.267208, -111.635183, 0, 0)
    obs2.cylinder_height = 40
    obs2.cylinder_radius = 10

    res.stationary_obstacles = [obs1, obs2]

    return res

if __name__ == '__main__':
    rospy.init_node('fake_interop', anonymous=True)
    service = rospy.Service("get_mission_with_id", GetMissionWithId, mission_response)
    while not rospy.is_shutdown():
        #try:
        rospy.spin()
        #except KeyboardInterrupt:
            #print('Kill signal received')
