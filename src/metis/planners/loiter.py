# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

from shapely.geometry import Point
import numpy as np

from metis.messages import msg_ned
from metis.planners import Planner

class LoiterPlanner(Planner):
    """The Loiter Planner plans the loiter mission."""

    def __init__(self, mission, radius=25., clearance=2.5):
        """
        Parameters
        ----------
        boundary_list
        obstacles : list
        bound_poly
        radius : float (optional)
            I assume this is the orbital radius (default 25.0)?
        clearance : float (optional)
            Not sure what clearance is (default 2.5).
        """
        super(LoiterPlanner, self).__init__(mission)
        self.radius = radius
        self.clearance = clearance
        

    def plan(self, waypoints):
        """
        We're not sure what it does yet. Will brb...

        Parameters
        ----------
        waypoints : list[msg_ned]

        Returns
        -------
        waypoints : list[msg_ned]
            A list containing a single value.
        """
        point = Point(waypoints.n, waypoints.e).buffer(self.radius+self.clearance)

        safe_point = True
        if point.within(self.boundary_poly):
            for obs in self.obstacles:
                if point.intersects(Point(obs.n, obs.e).buffer(obs.r)):
                    safe_point = False
            if safe_point:
                if waypoints.d > -45.0:
                    waypoints.d = -45.0
                return([waypoints])

        north = waypoints.n
        east = waypoints.e

        std = 50
        count = 0
        safe_point = False
        while safe_point == False:
            count = count + 1
            n = np.random.normal(loc=north, scale=std)
            e = np.random.normal(loc=east, scale=std)
            print(n,e)

            point = Point(n, e).buffer(self.radius+self.clearance)
            
            safe_point = True
            if point.within(self.boundary_poly):
                for obs in self.obstacles:
                    if point.intersects(Point(obs.n, obs.e).buffer(obs.r)):
                        safe_point = False
            
            if np.mod(count,5) == 0:
                std = std + 25

        

        return([msg_ned(n,e,-45.0)])
