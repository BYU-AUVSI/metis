# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

from . import Planner, PlannerData

class ObjectivePointsPlanner(PlannerData, Planner):
    """The Objective Planner class."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    
    def plan(self, waypoints):
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
        return(waypoints)
