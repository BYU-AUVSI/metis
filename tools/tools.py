from geographiclib.geodesic import Geodesic
import math

def convert(lat1, lon1, h1, lat2, lon2, h2):
	"""
	This function gives the relative N E D coordinates of gps2 relative to gps1
	"""
	
	diction = Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2)
        solution = [diction['s12']*math.cos(math.radians(diction['azi1'])), diction['s12']*math.sin(math.radians(diction['azi1'])), -(h2-h1)]
        return solution