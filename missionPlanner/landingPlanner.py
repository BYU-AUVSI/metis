import sys
sys.path.append('..')

import numpy as np
# from messages.ned import msg_ned
from tools.tools import makeBoundaryPoly
# from shapely.geometry import Point
# import matplotlib.pyplot as plt

class LandingPlanner():

    def __init__(self, boundary_list, obstacles, alt_threshold = 10, alt_default = 30):
        self.boundary_list = boundary_list
        self.boundary_poly = makeBoundaryPoly(self.boundary_list)
        self.obstalces = obstacles
        self.altitude_threshold = alt_threshold
        self.altitude_default = alt_default

    def plan(self, landing_waypoint_list, curr_altitude):
        landing_wpt = landing_waypoint_list[0]
        approach_wpt = landing_waypoint_list[1]

        # set landing waypoint to 1 meter above the ground
        landing_wpt.d = -1
        
        # use current altitude if above 10m, otherwise approach from 30m
        if (curr_altitude > self.altitude_threshold):
            approach_wpt.d = curr_altitude
        else:
            approach_wpt.d = self.altitude_default

        return [approach_wpt, landing_wpt]
    
