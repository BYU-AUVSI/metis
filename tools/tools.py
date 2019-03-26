from geographiclib.geodesic import Geodesic
from uav_msgs.msg import NED_pt, NED_list
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
        solution = [diction['s12']*math.cos(math.radians(diction['azi1'])), diction['s12']*math.sin(math.radians(diction['azi1'])), -(h2-h1)]
        return solution

def wypts2msg(waypoints, mission_type):
	"""
	This function converts a list of lists of waypoints to a rosmsg
	"""

	rsp = NED_list()

	for i in waypoints:
		tmp_wypt = NED_pt()
		tmp_wypt.N = i[0]
		tmp_wypt.E = i[1]
		tmp_wypt.D = i[2]
		tmp_wypt.task = mission_type
		rsp.waypoint_list.append(tmp_wypt)

	return rsp

def collisionCheck(obstaclesList, boundaryPoly, N, E, D, clearance):
	"""
	Checks points for collisions with obstacles and boundaries

	@type  obstaclesList: msg_ned
	@param obstaclesList: List of obstacles

	@type  boundaryPoly: Polygon
	@param boundaryPoly: A Polygon object of the boundaries

	@type  N: np.array
	@param N: Arrays of the north position of points

	@type  E: np.array
	@param E: Arrays of the east position of points

	@type  D: np.array
	@param D: Arrays of the down position of points

	@type  clearance: float
	@param clearance: The amount of clearance desired from obstacles and boundaries

	@rtype:  boolean
	@return: Returns true if a safe path, false if not
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
	"""
	Makes a list of boundary points into a Polygon object.

	@type  boundariesList: msg_ned
	@param boundariesList: List of boundary points

	@rtype:  Polygon
	@return: Returns the Polygon object of boundaries
	"""
	pointList = []
	for point in boundariesList:
		pointList.append(Point(point.n, point.e))
	return Polygon([[p.x, p.y] for p in pointList])  # Boundaries now contained in a Polygon object
