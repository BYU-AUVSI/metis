# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import numpy as np
from metis.planners import Planner

class LandingPlanner(Planner):
    """The Landing Planner mixin plans the landing waypoints."""

    def __init__(self, boundary_list, obstacles, boundary_poly=None, alt_threshold=10, alt_default=30):
        super(LandingPlanner, self).__init__(boundary_list, obstacles, boundary_poly=None)
        self.altitude_threshold = alt_threshold
        self.altitude_default = alt_default

    def plan(self, landing_waypoint_list, curr_altitude):
        """Plans the landing waypoints.

        Parameters
        ----------
        landing_waypoint_list : list
        curr_altitude : float
        altitude_threshold : float (optional)
        altitude_default : float (optional)

        Returns
        -------
        waypoints : list
            A list of waypoints
        """
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
    
