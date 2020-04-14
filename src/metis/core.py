# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

"""
Useful core classes that don't belong in any specific package.
"""

import logging

from metis.plotter import MissionPlotter
from metis.tools import bounds2poly


_module_logger = logging.getLogger(__name__)


class Mission(object):
    """
    A class containing all the data that comprises a mission.
    """
    _logger = _module_logger.getChild('Mission')

    def __init__(
        self,
        home=None,
        obstacles=None,
        boundary_list=None,
        waypoints=None,
        drop_location=None,
        search_area=None,
        offaxis_location=None
    ):
        """Initializes the Mission object.

        An empty Mission object can be initialized. However, for a Mission
        object to be useful, its attributes should be set prior to using it as
        a dataclass.

        Parameters
        ----------
        home : metis.GPSWaypoint, optional
            The location of the GPS groundstation, including the 
            groundstation's ground elevation.
        obstacles : list of metis.location.CircularObstacle, optional
        boundary_list : list of metis.location.BoundaryPoint, optional
            Boundaries that, in the order they are provided, trace out the 
            perimeter of the flight boundaries.
        waypoints : list of metis.location.Waypoint, optional
            List of waypoints in the order they should be traversed.
        drop_location : metis.location.Waypoint, optional
        search_area : list of metis.location.BoundaryPoint, optional
        """
        # private:
        self._boundary_poly = None
        self._boundary_list = None

        # public:
        self.home = home
        self.obstacles = obstacles
        self.boundary_list = boundary_list
        self.waypoints = waypoints
        self.drop_location = drop_location
        self.search_area = search_area
        self.offaxis_location = offaxis_location

    def __str__(self):
        """
        Returns a string representation of the mission.

        Includes a list of waypoints for obstacles, boundaries, and the drop
        location.

        Returns
        -------
        val : str
            The string representation of some Mission attributes.
        """
        val = ""
        val += "Obstacles\n"
        for obstacle in self.obstacles:
            val += "{} {} {} {}\n".format(obstacle.n, obstacle.e, obstacle.d, obstacle.r)
        val += "Boundaries\n"
        for boundary in self.boundary_list:
            val += "{} {}\n".format(boundary.n, boundary.e)
        val += "Drop Location\n"
        val += "{} {} {}\n".format(self.drop_location.n, self.drop_location.e, self.drop_location.d)
        return val

    @property
    def boundary_poly(self):
        # self._logger.debug("boundary_poly property called")
        return self._boundary_poly

    @property
    def boundary_list(self):
        # self._logger.debug("boundary_list property called")
        return self._boundary_list

    @boundary_list.setter
    def boundary_list(self, boundaries):
        if boundaries is not None:
            self._boundary_list = boundaries
            self._boundary_poly = bounds2poly(boundaries)
            # self._logger.debug("boundary_list property set")
        else:
            self._boundary_poly = None

class Plan:
    def __init__(self, mission, clear_previous=False, waypoints=None, callback=None):
        """
        Represents a planned path.

        Parameters
        ----------
        mission : metis.core.Mission
            The mission that corresponds to the certain plan.
        clear_previous : bool, optional
            Whether previous plans should be cleared and discarded.
        waypoints : list of metis.core.Waypoint
            The set of waypoints corresponding to the specified plan.
        """
        self.mission = mission
        self.clear_previous = clear_previous
        self.waypoints = waypoints
        self.callback = callback
        self.params = {}

    @property
    def last_waypoint(self):
        return self.waypoints[-1]

    def plot(self, block=True):
        mp = MissionPlotter(self.mission)
        mp.add_pathway(self.waypoints, "Planned pathway")
        mp.show(block=block)
        self.waypoints = mp.getNEDlist()

    def accept(self):
        if self.callback:
            self.callback(self)
        else:
            raise AttributeError("No callback configured for function 'accept'.")
