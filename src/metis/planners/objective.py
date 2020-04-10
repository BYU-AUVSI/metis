# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

from metis.planners import Planner
from metis.rrt import heading

class ObjectivePointsPlanner(Planner):
    """The Objective Planner class."""

    def __init__(self, mission):
        super(ObjectivePointsPlanner, self).__init__(mission)
    
    def plan(self):
        """ Plans the path to hit the designated flight points

        Since our architecture is such that all mission tasks (payload, search) take in parameters and then determine points they want to fly through, this just passes the mission waypoints straight through.
        A flyable path is constructed later using a path planning algorithm.

        Parameters
        ----------
        waypoints : list of NED class
            The objective points that interop wants us to fly through
        
        Returns
        -------
        waypoints : list of NED class
            The objective points that we want to fly through.

        """
        waypoints = self.mission.waypoints

        for way1, way2 in zip(waypoints[:-1], waypoints[1:]):
            way1.chi = heading(way1, way2)

        return self.mission.waypoints
