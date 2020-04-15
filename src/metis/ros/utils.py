# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import rospy
from uav_msgs.msg import JudgeMission, NED_pt, NED_list
from uav_msgs.srv import GetMissionWithId

from metis import Mission
from metis.location import GPSWaypoint, Waypoint, BoundaryPoint, CircularObstacle, convert_point
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
    Converts a ros message to a list of Waypoint classes.
    """

    waypoints = []
    for point in message.waypoint_list:
        waypoints.append(Waypoint(point.N, point.E, point.D))

    return waypoints


def get_reference_position():
    """
    Returns the reference position as a GPSWaypoint object.

    The reference latitude and longitude is from the launch file
    (it's a <param> in the XML file). This function takes its values from 
    parameters with the labels `ref_lat`, `ref_lon`, and `ref_h`.

    Returns
    -------
    ref : metis.location.GPSWaypoint
        The reference position from the ROS global namespace as a GPSWaypoint 
        object.
    """
    ref = GPSWaypoint(
        rospy.get_param("/interop_client/init_lat"),
        rospy.get_param("/interop_client/init_lon"),
        rospy.get_param("/interop_client/init_h"),
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
        _, # mission.boundary_poly,
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
        The mission type number for which data was obtained (number defined 
        in the JudgeMission message)
    obstacles : list of metis.location.CircularObstacle
        A list of NED messages
    boundary_list : list of metis.location.BoundaryPoint
        A list of all boundary points as BoundaryPoint objects.
    boundary_poly : shapely.geometry.polygon.Polygon
        A polygon object that defines the boundaries
    waypoints : list of metis.location.Waypoint
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


def _convert_obstacles(msg, ref_pos):
    """
    Converts obstacles from a rospy message to a list of NED objects.

    Parameters
    ----------
    msg : uav_msgs.msg.JudgeMission
        The message received from the interop server.
        Latitude and longitude are expressed as floats. Altitude is expressed
        in meters.
    ref_pos : metis.GPSWaypoint
        The reference position for relative position calculations.

    Returns
    -------
    obstacle_list : list of metis.location.CircularObstacle
        A list containing the position and height for each obstacle.
    """
    obstacle_list = []

    for i in msg.stationary_obstacles:
        obs = GPSWaypoint(i.point.latitude, i.point.longitude, i.cylinder_height)
        ned = obs.ned_from(ref_pos)
        ned = convert_point(ned, CircularObstacle)
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
        Latitude and longitude are expressed as floats. Altitude is expressed
        in meters.
    ref_pos : metis.GPSWaypoint
        The reference position for relative position calculations.

    Returns
    -------
    waypoint_list : list of metis.location.Waypoint
        List containing the NED position of each waypoint.
    """
    waypoint_list = []

    for i in msg.waypoints:
        wpt = GPSWaypoint(i.point.latitude, i.point.longitude, i.point.altitude)
        ned = wpt.ned_from(ref_pos)
        # ned.d -= 22.0 # TODO: For some reason, ground level is -22.0
        waypoint_list.append(convert_point(ned, Waypoint))

    return waypoint_list


def _convert_boundaries(msg, ref_pos):
    """
    Converts the boundary points obtained from the interop server to NED 
    coordinates.

    Parameters
    ----------
    msg : uav_msgs.msg.JudgeMission
        The message received from the interop server.
        Latitude and longitude are expressed as floats. Altitude is expressed
        in meters.
    ref_pos : metis.GPSWaypoint
        The reference position for relative position calculations.

    Returns
    -------
    list : list of metis.location.BoundaryPoint
        A list of all the boundary points as BoundaryPoint objects.
    """
    boundary_list = []

    for i in msg.boundaries:
        bnd = GPSWaypoint(i.point.latitude, i.point.longitude, i.point.altitude)
        pnt = bnd.ned_from(ref_pos)
        boundary_list.append(convert_point(pnt, BoundaryPoint))

    boundary_poly = tools.bounds2poly(boundary_list)
    return boundary_list, boundary_poly
