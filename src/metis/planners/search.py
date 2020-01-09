# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import numpy as np
from shapely.geometry import Point
import matplotlib.pyplot as plt

from metis.messages import msg_ned
from metis.tools import makeBoundaryPoly
from metis.planners import Planner

class SearchPlanner(Planner):
    """
    Class that plans the search mission. 
    This class makes a lawn mower path by determining the box that bounds the search area and then populating that box with points a set distance apart. The points are narrowed by removing any points that are too close to the flight boundaries or too far from the search area.
    """

    def __init__(self, boundary_list, obstacles, boundary_poly=None, waypoint_distance=50.0, height=30.0):
        """
        Initialize the variables for the planning algorithm.
        The flight boundaries are converted from a list to shapely object to allow checking if points are within the boundaries

        Parameters
        ----------
        flight_boundaries : list of NED class
            The points that define the fly zone
        obstacles : list of NED class
            The location and description of the obstacles
        waypoint_distance : float
            The distance each point in the lawn mower path should be from its neighbors
        """
        super(SearchPlanner, self).__init__(boundary_list, obstacles, boundary_poly=None)
        self.flight_boundaries = self.boundary_list
        self.flight_poly = self.boundary_poly
        self.waypoint_distance = waypoint_distance
        self.height = height

    def plan(self, search_boundaries, current_pos=msg_ned(0.,0.,-100.), clearance=10, visualize=True):
        """
        Creates a lawn mower path over the search area

        Plans a lawnmower path that covers the search area and does not get within the clearance value of the fly zone.

        Parameters
        ----------
        search_boundaries : list of NED class
            The boundaries that define the search area
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
        
        self.search_boundaries = makeBoundaryPoly(search_boundaries)

        # Find the box that fits the search area
        num_points = len(search_boundaries)
        n_pos = np.zeros(num_points)
        e_pos = np.zeros(num_points)
        for i in np.arange(num_points):
            n_pos[i] = search_boundaries[i].n
            e_pos[i] = search_boundaries[i].e
        
        w_bound = np.min(e_pos) - self.waypoint_distance
        e_bound = np.max(e_pos) + self.waypoint_distance
        n_bound = np.max(n_pos) + self.waypoint_distance
        s_bound = np.min(n_pos) - self.waypoint_distance

        search_box = [msg_ned(n_bound, e_bound), msg_ned(s_bound, e_bound), msg_ned(s_bound, w_bound), msg_ned(n_bound, w_bound)]
        search_box_p = makeBoundaryPoly(search_box)

        # Initialize variables for creating the search path
        all_points = []
        cur_pos = msg_ned(n_bound, w_bound, current_pos.d, -33.0)
        direction = 1 # Positive advances the path to the right, negative moves the path to the left. Changes each time it needs to turn around to stay in the search area

        # Create the lawn mower path
        all_points.append(msg_ned(cur_pos.n, cur_pos.e, current_pos.d)) #Start path at the North-East corner of the bounding box
        while cur_pos.n >= s_bound: #Stop adding points once we are below the bounding box
            while cur_pos.e >= w_bound and cur_pos.e <= e_bound: #Add points on an East-West or West-East line until we leave the bounding box
                cur_pos.e = cur_pos.e + direction*self.waypoint_distance 
                all_points.append(msg_ned(cur_pos.n, cur_pos.e, -self.height))
            direction = -direction #When we leave the bounding box, turn around
            cur_pos.n = cur_pos.n - self.waypoint_distance #Advance path one step in a South direction
            all_points.append(msg_ned(cur_pos.n, cur_pos.e, -self.height))
            while cur_pos.e <= w_bound or cur_pos.e >= e_bound: #Bring the path back inside the bounding box area
                cur_pos.e = cur_pos.e + direction*self.waypoint_distance
                all_points.append(msg_ned(cur_pos.n, cur_pos.e, -self.height))
        
        # Eliminate points that are too close to the flight boundary or too far from the search area
        final_waypoints = []
        for waypoint in all_points:
            waypoint_circle_large = Point(waypoint.n, waypoint.e).buffer(clearance) # Creates a point with a radius of clearance
            waypoint_circle_small = Point(waypoint.n, waypoint.e).buffer(self.waypoint_distance) # Point with a radius of waypoint_distance
            if waypoint_circle_large.within(self.flight_poly) and waypoint_circle_small.intersects(self.search_boundaries): # Check if the point is too close or far from boundaries
                final_waypoints.append(msg_ned(waypoint.n, waypoint.e,-self.height))

        # Plot the planned points and all boundaries
        if visualize:
            fig, ax = plt.subplots()
            for point in final_waypoints:
                ax.scatter(point.e, point.n, c='k')
            for point in self.flight_boundaries:
                ax.scatter(point.e, point.n, c='r')
            for point in search_boundaries:
                ax.scatter(point.e, point.n, c='g')
            plt.show()
        
        return final_waypoints
