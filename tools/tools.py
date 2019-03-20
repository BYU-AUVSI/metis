from geographiclib.geodesic import Geodesic
from uav_msgs.msg import NED_pt, NED_list
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
