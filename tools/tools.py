

import rospy
from uav_msgs.msg import NED_pt, NED_list
from geographiclib.geodesic import Geodesic
# from uav_msgs.msg import NED_pt, NED_list
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from messages.ned import msg_ned
import numpy as np
import math
from uav_msgs.srv import GetMissionWithId

def convert(lat1, lon1, h1, lat2, lon2, h2):
    """
    This function gives the relative N E D coordinates of gps2 relative to gps1
    """
    diction = Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2)
    solution = [diction['s12']*math.cos(math.radians(diction['azi1'])), diction['s12']*math.sin(math.radians(diction['azi1'])), float(-(h2-h1))]
    return solution

def wypts2msg(waypoints, mission_type):
    """
    This function converts a list of lists of waypoints to a rosmsg
    """


    rsp = NED_list()

    for i in waypoints:
        tmp_wypt = NED_pt()
        tmp_wypt.N = i.n
        tmp_wypt.E = i.e
        tmp_wypt.D = i.d
        tmp_wypt.task = mission_type
        rsp.waypoint_list.append(tmp_wypt)

    return rsp

def msg2wypts(message):
    """
    Converts a ros message to a list of msg_ned classes
    """

    waypoints = []
    for point in message.planned_waypoints.waypoint_list:
        waypoints.append(msg_ned(point.N, point.E, point.D))

    return waypoints

def collisionCheck(obstaclesList, boundaryPoly, N, E, D, clearance):
	"""Checks points for collisions with obstacles and boundaries

	Parameters
	----------
	obstaclesList : msg_ned
		List of obstacles

	boundaryPoly : Polygon
		A Polygon object of the boundaries

	N : np.array
		Arrays of the north position of points

	E : np.array
		Arrays of the east position of points

	D : np.array
		Arrays of the down position of points

	clearance : float
		The amount of clearance desired from obstacles and boundaries

	Returns
	----------
	boolean
		Returns true if a safe path, false if not
	"""
	# First check for collision with obstacles
	for obstacle in obstaclesList:
		# first check if path is above obstacle
		if (all(D < -obstacle.d - clearance)):
			continue
		# then check if runs into obstacle
		else:
			distToPoint = np.sqrt((N - obstacle.n) ** 2 + (E - obstacle.e) ** 2)
			if (any(distToPoint < obstacle.r + clearance)):
				return False

	# Check for out of boundaries
	for i in range(0, len(N)):
		if not boundaryPoly.contains(Point(N[i], E[i])):
			return False
	return True


def makeBoundaryPoly(boundariesList):
    """Makes a list of boundary points into a Polygon object.

    Parameters
    ----------
    boundariesList : msg_ned
        List of boundary points

    Returns
    ----------
    boundaries : Polygon
        Returns the Polygon object of boundaries
    """
    pointList = []
    for point in boundariesList:
        pointList.append(Point(point.n, point.e))
    return Polygon([[p.x, p.y] for p in pointList])  # Boundaries now contained in a Polygon object

def get_server_data(mission_type, ref_pos):
    """Gets data from the interop server

    Polls the interop server node via a service call to get the data associated with a specific mission
    Returns a tuple of all the information

    Parameters
    ----------
    mission_type : int
        The mission type number for which data is to be recieved (number defined in the JudgeMission message)

    Returns
    -------
    mission_type : int
        The mission type number for which data was obtained (number defined in the JudgeMission message)

    obstacles : list of NED messages
        A list of NED messages

    boundaries : polygon
        A polygon object that defines the boundaries

    waypoints : list of NED messages
        A list of NED messages
    """

    #Wait for the interop client service call to initiate
    rospy.wait_for_service('get_mission_with_id')

    #Set up a service call to poll the interop server
    mission_data = rospy.ServiceProxy('get_mission_with_id', GetMissionWithId)

    #Send the service call with the desired mission type number
    resp = mission_data(mission_type)

    #Get boundaries, obstacles, flight waypoints
    obstacles = convert_obstacles(ref_pos, resp.mission)
    boundary_list, boundary_poly = convert_boundaries(ref_pos, resp.mission)
    waypoints =  convert_waypoints(ref_pos, resp.mission)

    return mission_type, obstacles, boundary_list, boundary_poly, waypoints

def convert_obstacles(ref_pos, msg):
        """
        Converts the obstacles from the rospy message to a list of NED classes

        Parameters
        ----------
        msg : JudgeMission message
            The message received from the interop server
        ref_pos : list
            The reference latitude, longitude, and height (in m)

        Returns
        -------
        obstacle_list : list of NED classes
            Describes the position and height of each obstacle

        """
        ref_lat = ref_pos[0]
        ref_lon = ref_pos[1]
        ref_h = ref_pos[2]

        obstacle_list = []

        for i in msg.stationary_obstacles:
            obs_NED = convert(ref_lat, ref_lon, ref_h, i.point.latitude, i.point.longitude, i.point.altitude)
            obstacle_list.append(msg_ned(obs_NED[0], obs_NED[1], i.cylinder_height, i.cylinder_radius))

        return obstacle_list

def convert_waypoints(ref_pos, msg):
    """
    Converts the waypoints obtained from the interop server to NED coordinates
    This function doesn't care about what mission is being run, it just gets the waypoints
    which can be for the drop location, flight location, or search boundaries, and converts them


    Parameters
    ----------
    msg : JudgeMission message
        The message received from the interop server
    ref_pos : list
        The reference latitude, longitude, and height (in m)

    Returns
    -------
    waypoint_list : list of NED classes
        List describing the NED position of each waypoint

    """
    ref_lat = ref_pos[0]
    ref_lon = ref_pos[1]
    ref_h = ref_pos[2]

    waypoint_list = []

    for i in msg.waypoints:
        wpt_NED = convert(ref_lat, ref_lon, ref_h, i.point.latitude, i.point.longitude, i.point.altitude)
        waypoint_list.append(msg_ned(wpt_NED[0], wpt_NED[1], wpt_NED[2]))

    return waypoint_list

def convert_boundaries(ref_pos, msg):
    """
    Converts the boundary points obtained from the interop server to NED coordinates

    Parameters
    ----------
    msg : JudgeMission message
        The message received from the interop server
    ref_pos : list
        The reference latitude, longitude, and height (in m)

    Returns
    -------
    list
        a list of lists describing all the boundary points
        each inner list has the North location, East location, and Down location of a single boundary point
        all measurements are in meters

    """
    ref_lat = ref_pos[0]
    ref_lon = ref_pos[1]
    ref_h = ref_pos[2]

    boundary_list = []

    for i in msg.boundaries:
        bnd_NED = convert(ref_lat, ref_lon, ref_h, i.point.latitude, i.point.longitude, i.point.altitude)
        boundary_list.append(msg_ned(bnd_NED[0], bnd_NED[1]))

    boundary_poly = makeBoundaryPoly(boundary_list)
    return boundary_list, boundary_poly
