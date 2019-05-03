ROS_Flag = True #Flag to determine if file is being run in a ros node or testing enviornment
import socket

from sys import platform
if platform == "win32": #If running on windows, don't bother checking for ROS
    ROS_Flag = False
if ROS_Flag:
    try:
        import rospy
        rospy.get_master().getPid() #Check to see if ros is running
        from uav_msgs.msg import NED_pt, NED_list
    except socket.error:
        print("tools.py: File not being run through ROS")
        import sys
        sys.path.append("..")

from geographiclib.geodesic import Geodesic
# from uav_msgs.msg import NED_pt, NED_list
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from messages.ned import msg_ned
import numpy as np
import math

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

    if ROS_Flag:
        rsp = NED_list()

        for i in waypoints:
            tmp_wypt = NED_pt()
            tmp_wypt.N = i[0]
            tmp_wypt.E = i[1]
            tmp_wypt.D = i[2]
            tmp_wypt.task = mission_type
            rsp.waypoint_list.append(tmp_wypt)

        return rsp
    else: #Run testing code if ROS is not running
        #Currently, just pass the waypoint list through
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
