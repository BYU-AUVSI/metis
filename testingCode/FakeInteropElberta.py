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
    pt1.point = Point(39.983031, -111.991051, 30, 0)
    pt1.ordinal = 1

    pt2 = OrderedPoint()
    pt2.point = Point(39.983449, -111.992160, 45, 0)
    pt2.ordinal = 2

    pt3 = OrderedPoint()
    pt3.point = Point(39.982648, -111.993338, 45, 0)
    pt3.ordinal = 3

    pt4 = OrderedPoint()
    pt4.point = Point(39.981005, -111.992602, 45, 0)
    pt4.ordinal = 4

    pt5 = OrderedPoint()
    pt5.point = Point(39.981912, -111.989530, 45, 0)
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
    bnd1.point = Point(39.985229, -111.993796, 0, 0)
    bnd1.ordinal = 1

    bnd2 = OrderedPoint()
    bnd2.point = Point(39.981026, -111.986903, 0, 0)
    bnd2.ordinal = 2

    bnd3 = OrderedPoint()
    bnd3.point = Point(39.977468, -111.993858, 0, 0)
    bnd3.ordinal = 3

    bnd4 = OrderedPoint()
    bnd4.point = Point(39.983239, -112.000138, 0, 0)
    bnd4.ordinal = 4

    res.boundaries = [bnd1, bnd2, bnd3, bnd4]

    #Define obstacles

    obs1 = StationaryObstacle()
    obs1.point = Point(39.983258, -111.994228, 0, 0)
    obs1.cylinder_height = 60
    obs1.cylinder_radius = 20

    obs2 = StationaryObstacle()
    obs2.point = Point(39.981556, -111.993697, 0, 0)
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
