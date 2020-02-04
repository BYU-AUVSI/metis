# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import rospy
from uav_msgs.msg import JudgeMission, NED_pt, NED_list
from uav_msgs.srv import GetMissionWithId

from metis.messages import msg_ned
from metis import Mission, GPSWaypoint
from metis import tools


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
    Converts a ros message to a list of msg_ned classes.
    """

    waypoints = []
    for point in message.waypoint_list:
        waypoints.append(msg_ned(point.N, point.E, point.D))

    return waypoints


def get_reference_position():
    """
    Returns the reference position as a GPSWaypoint object.

    The reference latitude and longitude is from the launch file
    (it's a <param> in the XML file). This function takes its values from 
    parameters with the labels `ref_lat`, `ref_lon`, and `ref_h`.

    Returns
    -------
    ref : metis.core.GPSWaypoint
        The reference position from the ROS global namespace as a GPSWaypoint 
        object.
    """
    ref = GPSWaypoint(
        rospy.get_param("ref_lat"),
        rospy.get_param("ref_lon"),
        rospy.get_param("ref_h"),
    )
    return ref

def get_mission():
    """
    A convenience function for initializing a Mission object with data from
    the interop server.

    Returns
    -------
    mission : metis.core.Mission
        A fully initialized mission object.
    """
    mission = Mission()
    mission.home = get_reference_position()

    # Get the obstacles, boundaries, and drop location in order to initialize the planner classes
    (
        _,
        mission.obstacles,
        mission.boundary_list,
        mission.boundary_poly,
        drop_location,
    ) = get_server_data(JudgeMission.MISSION_TYPE_DROP, mission.home)

    _, _, _, _, mission.waypoints = get_server_data(
        JudgeMission.MISSION_TYPE_WAYPOINT, mission.home
    )

    _, _, _, _, mission.search_area = get_server_data(
        JudgeMission.MISSION_TYPE_SEARCH, mission.home
    )

    _, _, _, _, offaxis_location = get_server_data(
        JudgeMission.MISSION_TYPE_OFFAXIS, mission.home
    )

    mission.drop_location = drop_location[0]
    mission.offaxis_location = offaxis_location[0]
    return mission


def get_server_data(mission_type, ref_pos):
    """
    Gets data from the interop server.

    Polls the interop server node via a service call to get the data 
    associated with a specific mission. Returns a tuple of all the information.

    Parameters
    ----------
    mission_type : int
        The mission type number for which data is to be recieved 
        (legal values defined in the JudgeMission message).
    ref_pos : metis.GPSWaypoint
        The home location, typically the groundstation. All positions will be
        converted to positions relative to this position.

    Returns
    -------
    mission_type : int
        The mission type number for which data was obtained (number defined in the JudgeMission message)
    obstacles : list of metis.messages.msg_ned
        A list of NED messages
    boundary_list : list of metis.messages.msg_ned
        A list of all boundary points as msg_ned objects.
    boundary_poly : shapely.geometry.polygon.Polygon
        A polygon object that defines the boundaries
    waypoints : list of metis.messages.msg_ned
        A list of NED messages
    """
    # TODO: Move this service proxy to `services.py`
    rospy.wait_for_service('get_mission_with_id')
    mission_data = rospy.ServiceProxy('get_mission_with_id', GetMissionWithId)
    resp = mission_data(mission_type)

    #Get boundaries, obstacles, flight waypoints
    obstacles = _convert_obstacles(resp.mission, ref_pos)
    boundary_list, boundary_poly = _convert_boundaries(resp.mission, ref_pos)
    waypoints = _convert_waypoints(resp.mission, ref_pos)

    return mission_type, obstacles, boundary_list, boundary_poly, waypoints


def ft2m(feet):
    """
    Convenience function for converting from feet to meters.

    Parameters
    ----------
    feet : float
        Length in feet.

    Returns
    -------
    float
        Length in meters.
    """
    return float(feet / 3.2808)

def _convert_obstacles(msg, ref_pos):
    """
    Converts obstacles from a rospy message to a list of NED objects.

    Parameters
    ----------
    msg : uav_msgs.msg.JudgeMission
        The message received from the interop server.
    ref_pos : metis.GPSWaypoint
        The reference position for relative position calculations.

    Returns
    -------
    obstacle_list : list of metis.messages.msg_ned
        A list containing the position and height for each obstacle.
    """
    obstacle_list = []

    for i in msg.stationary_obstacles:
        obs = GPSWaypoint(i.point.latitude, i.point.longitude, ft2m(i.point.altitude))
        ned = obs.ned_from(ref_pos)
        ned.d = i.cylinder_height
        ned.r = i.cylinder_radius
        obstacle_list.append(ned)

    return obstacle_list


def _convert_waypoints(msg, ref_pos):
    """
    Converts the waypoints obtained from the interop server to NED coordinate
    objects.

    This function doesn't care about what mission is being run; it converts 
    waypoints which can be for the drop location, flight location, or search 
    boundaries.

    Parameters
    ----------
    msg : uav_msgs.msg.JudgeMission
        The message received from the interop server.
    ref_pos : metis.GPSWaypoint
        The reference position for relative position calculations.

    Returns
    -------
    waypoint_list : list of metis.messages.msg_ned
        List containing the NED position of each waypoint.
    """
    waypoint_list = []

    for i in msg.waypoints:
        wpt = GPSWaypoint(i.point.latitude, i.point.longitude, ft2m(i.point.altitude))
        ned = wpt.ned_from(ref_pos)
        ned.d -= 22.0 # TODO: For some reason, ground level is -22.0
        waypoint_list.append(ned)

    return waypoint_list


def _convert_boundaries(msg, ref_pos):
    """
    Converts the boundary points obtained from the interop server to NED 
    coordinates.

    Parameters
    ----------
    msg : uav_msgs.msg.JudgeMission
        The message received from the interop server.
    ref_pos : metis.GPSWaypoint
        The reference position for relative position calculations.

    Returns
    -------
    list : list of metis.messages.msg_ned
        A list of all the boundary points as msg_ned objects.
    """
    boundary_list = []

    for i in msg.boundaries:
        bnd = GPSWaypoint(i.point.latitude, i.point.longitude, ft2m(i.point.altitude))
        boundary_list.append(bnd.ned_from(ref_pos))

    boundary_poly = tools.makeBoundaryPoly(boundary_list)
    return boundary_list, boundary_poly