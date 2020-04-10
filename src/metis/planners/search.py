# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import numpy as np
from shapely.geometry import Point
import matplotlib.pyplot as plt

from metis.location import Waypoint, BoundaryPoint
from metis.tools import bounds2poly
from metis.planners import Planner
from metis.rrt import wrap2pi

class SearchPlanner(Planner):
    """
    Class that plans the search mission. 
    This class makes a lawn mower path by determining the box that bounds the search area and then populating that box with points a set distance apart. The points are narrowed by removing any points that are too close to the flight boundaries or too far from the search area.
    """

    def __init__(self, mission, waypoint_distance=50.0, height=30.0):
        """
        Initialize the variables for the planning algorithm.
        The flight boundaries are converted from a list to shapely object to allow checking if points are within the boundaries

        Parameters
        ----------
        mission : metis.core.Mission
            A mission description object.
        waypoint_distance : float
            The distance each point in the lawn mower path should be from its neighbors, in meters
        height : float
            
        """
        super(SearchPlanner, self).__init__(mission)
        self.flight_boundaries = mission.boundary_list
        self.flight_poly = mission.boundary_poly
        self.waypoint_distance = waypoint_distance
        self.height = height

    def plan(self, current_pos=Waypoint(0.,0.,-100.), clearance=10, visualize=True):
        """
        Creates a lawn mower path over the search area

        Plans a lawnmower path that covers the search area and does not get within the clearance value of the fly zone.

        Parameters
        ----------
        current_pos : NED class
            The current position of the airplane
        clearance : float
            The closest distance a waypoint is permitted to be to the fly zone boundaries
        visualize : boolean
            For debugging only. If true, the planned points and boundaries will be displayed
        
        Returns
        -------
        final_waypoints : list of NED class
            The points that define the planned search path
        
        Warnings
        --------
        The altitude of the search path is set to the attitude of the current position.

        Obstacles are not currently accounted for in planning the path.

        If visualize is true, the code looks like it stops and does not continue executing until the plot is closed.

        """
        self.search_boundaries = bounds2poly(self.mission.search_area)

        # Find the box that fits the search area        
        min_e, min_n, max_e, max_n = self.search_boundaries.bounds
        w_bound = min_e - self.waypoint_distance
        e_bound = max_e + self.waypoint_distance
        n_bound = max_n + self.waypoint_distance
        s_bound = min_n - self.waypoint_distance

        search_box = [BoundaryPoint(n_bound, e_bound), BoundaryPoint(s_bound, e_bound), BoundaryPoint(s_bound, w_bound), BoundaryPoint(n_bound, w_bound)]
        search_box_p = bounds2poly(search_box)

        long_axis = 'NS' if (max_n - min_n) > (max_e - min_e) else 'EW'

        x = np.arange(min_e, max_e + self.waypoint_distance, self.waypoint_distance)[::-1]
        y = np.arange(min_n - self.waypoint_distance, max_n, self.waypoint_distance)[::-1]

        if long_axis == 'NS':
            print('NS')
            all_points = self.zamboni(y, x, 'NS')
        else:
            print('EW')
            all_points = self.zamboni(x, y, 'EW')

        # all_points = [Waypoint(point[0], point[1], -self.height) for point in all_points]

        # Eliminate points that are too close to the flight boundary or too far from the search area
        final_waypoints = []
        for waypoint in all_points:
            waypoint_circle_large = Point(waypoint.e, waypoint.n).buffer(clearance) # Creates a point with a radius of clearance
            waypoint_circle_small = Point(waypoint.e, waypoint.n).buffer(self.waypoint_distance) # Point with a radius of waypoint_distance
            if waypoint_circle_large.within(self.flight_poly) and waypoint_circle_small.intersects(self.search_boundaries): # Check if the point is too close or far from boundaries
                final_waypoints.append(waypoint)

        # Plot the planned points and all boundaries
        if visualize:
            fig, ax = plt.subplots()
            counter = 1
            for point in final_waypoints:
                ax.scatter(point.e, point.n, c='k')
                # ax.text(point.e, point.n, str(counter))
                # ax.text(point.e, point.n, "    " + str(point.chi))
                counter += 1
            for point in self.flight_boundaries:
                ax.scatter(point.e, point.n, c='r')
            e, n = self.search_boundaries.exterior.xy
            ax.plot(e, n)
            plt.axis('equal')
            plt.show()
        
        return final_waypoints


    def zamboni(self, long_axis, short_axis, orientation):
        """
        Suppose we have a search as follows:

        \ A B C D E F G H I J K L M N O P
        1 . . . . . . . . . . . . . .
        2 . . . . . . . . . . . . . . .
        3 . . . . . .     . . . . . . . .
        4   . . . . . . . . . . . . . . .
        5     . . . . . . . . . . . . .

        The long axis is the alphabetical axis, and the short axis is the numerical
        axis.

        Paths are planned beginning at the north east corner.

        Parameters
        ----------
        long_axis : np.ndarry, dtype=float
            An array containing the coordinates along the longer axis.
        short_axis : np.ndarry, dtype=float
            An array containing the coordinates along the shorter axis.
        orientation : str, 'EW' or 'NS'
            Whether the search area should be traversed east-west or 
            north-south. 
        """
        turns = np.shape(short_axis)[0] - 1

        if orientation == 'EW':
            chi = np.radians(-90)
        else:
            chi = np.radians(180)

        waypoints = []
        direction = 1
        for i in reorder(short_axis):
            for j in long_axis[::direction]:
                waypoints.append(Waypoint(i, j, -self.height, chi))
            direction = -direction
            chi = -chi

        return waypoints

def reorder(short_axis):
    start, mid = 0, int(np.ceil(len(short_axis)/2.))
    reordered = np.array([])

    passes = 0
    middle = False
    while passes < len(short_axis):
        if not middle:
            reordered = np.append(reordered, short_axis[start])
            start += 1
        else:
            reordered = np.append(reordered, short_axis[mid])
            mid += 1
        middle = not middle
        passes += 1

    return reordered
