import sys
sys.path.append('..')

import numpy as np
from messages.ned import msg_ned
from tools.tools import makeBoundaryPoly
from shapely.geometry import Point
import matplotlib.pyplot as plt

class LandingPlanner():

    def __init__(self, boundary_list, obstacles):
        self.boundary_list = boundary_list
        self.boundary_poly = makeBoundaryPoly(self.boundary_list)
        self.obstalces = obstacles

    def plan(self, landing_waypoint_list):
        return [msg_ned(0, 0, 0)]
    
