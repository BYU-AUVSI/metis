# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

# import rospy
from uav_msgs.msg import NED_pt, NED_list
# from metis.messages import NED_pt, NED_list
# from geographiclib.geodesic import Geodesic
# from shapely.geometry import Point
# from shapely.geometry.polygon import Polygon
from metis.messages import msg_ned
# import numpy as np
# import math
# from uav_msgs.srv import GetMissionWithId


def wypts2msg(waypoints, mission_type):
    """
    This function converts a list of lists of waypoints to a rosmsg.

    Parameters
    ----------
    waypoints : 
    mission_type : int

    Returns
    -------
    resp
        A ros message; a NED_list of waypoints.
    """
    resp = NED_list()

    for i in waypoints:
        tmp_wypt = NED_pt()
        tmp_wypt.N = i.n
        tmp_wypt.E = i.e
        tmp_wypt.D = i.d
        tmp_wypt.task = mission_type
        resp.waypoint_list.append(tmp_wypt)

    return resp


def msg2wypts(message):
    """
    Converts a ros message to a list of msg_ned classes
    """

    waypoints = []
    for point in message.waypoint_list:
        waypoints.append(msg_ned(point.N, point.E, point.D))

    return waypoints