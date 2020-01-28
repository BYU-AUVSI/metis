# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

"""
Useful core classes that don't belong in any specific package.
"""

import math

from geographiclib.geodesic import Geodesic

from metis.errors import InvalidCallbackError
from metis.messages import msg_ned
from metis.plotter import MissionPlotter

class Mission:
    """
    A class containing all the data that comprises a mission.
    """

    def __init__(
        self,
        home=None,
        obstacles=None,
        boundary_list=None,
        boundary_poly=None,
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
        obstacles : list of metis.messages.msg_ned, optional
        boundary_list : list of metis.messages.msg_ned, optional
        boundary_poly : shapely.geometry.polygon.Polygon, optional
        waypoints : list of metis.messages.msg_ned, optional
        drop_location : metis.messages.msg_ned, optional
        search_area : list of metis.messages.msg_ned, optional
        """
        self.home = home
        self.obstacles = obstacles
        self.boundary_list = boundary_list
        self.boundary_poly = boundary_poly
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
        waypoints : list of metis.messages.msg_ned
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

    def accept(self):
        if self.callback:
            self.callback(self)
        else:
            raise InvalidCallbackError("No callback configured for function 'accept'.")

class GPSWaypoint:
    def __init__(self, lat, lon, elev):
        """`
        A data structure representing a GPS point.

        Parameters
        ----------
        lat : float
            The latitude of the point, expressed in decimal degrees (DD).
        lon : float
            The longitude of the point, expressed in decimal degrees (DD).
        elev : float
            The elevation of the point, in meters.
        """
        self.lat = lat
        self.lon = lon
        self.elev = elev

    def ned_from(self, gps2):
        """
        Returns the NED position of the this GPS point relative to another 
        GPS point.

        Parameters
        ----------
        gps2 : metis.GPSWaypoint
            The point to serve as a reference point. NED position is calculated
            relative to this point.

        Returns
        -------
        res : metis.messages.msg_ned
            A message where the result is the NED position of this point
            relative to gps2 (the radius property is always 0).

        >>> ref = GPSWaypoint(39.98304881111, -111.9903776, 0)
        >>> obstacle = GPSWaypoint(39.983258, -111.994228, 0)
        >>> obstacle.ned_from(ref).to_array()
        [23.234244610397234, -328.88079281265095, 0.0, 0]
        """
        inv = Geodesic.WGS84.Inverse(gps2.lat, gps2.lon, self.lat, self.lon)
        res = msg_ned(
            north = inv["s12"] * math.cos(math.radians(inv["azi1"])),
            east = inv["s12"] * math.sin(math.radians(inv["azi1"])),
            down = float(-(self.elev - gps2.elev)),
            radius = 0
        )
        return res