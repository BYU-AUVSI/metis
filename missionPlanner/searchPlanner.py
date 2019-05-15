import sys
sys.path.append('..')

import numpy as np
from messages.ned import msg_ned
from tools.tools import makeBoundaryPoly
from shapely.geometry import Point
import matplotlib.pyplot as plt

class SearchPlanner():

    def __init__(self, flight_boundaries, obstacles, waypoint_distance=50.):
        self.flight_boundaries = flight_boundaries
        self.flight_poly = makeBoundaryPoly(self.flight_boundaries)
        self.obstalces = obstacles
        self.waypoint_distance = waypoint_distance

    def plan(self, search_boundaries, current_pos=msg_ned(0.,0.,-100.), clearance=10, visualize=False):
        self.search_boundaries = makeBoundaryPoly(search_boundaries)

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

        all_points = []
        cur_pos = msg_ned(n_bound, w_bound)
        direction = 1

        all_points.append(msg_ned(cur_pos.n, cur_pos.e))
        while cur_pos.n >= s_bound:
            while cur_pos.e >= w_bound and cur_pos.e <= e_bound:
                cur_pos.e = cur_pos.e + direction*self.waypoint_distance 
                all_points.append(msg_ned(cur_pos.n, cur_pos.e))
            direction = -direction
            cur_pos.n = cur_pos.n - self.waypoint_distance
            all_points.append(msg_ned(cur_pos.n, cur_pos.e))
            print(cur_pos.e, cur_pos.n)
            while cur_pos.e <= w_bound or cur_pos.e >= e_bound:
                cur_pos.e = cur_pos.e + direction*self.waypoint_distance
                all_points.append(msg_ned(cur_pos.n, cur_pos.e))
        
        final_waypoints = []
        for waypoint in all_points:
            waypoint_circle_large = Point(waypoint.n, waypoint.e).buffer(clearance)
            waypoint_circle_small = Point(waypoint.n, waypoint.e).buffer(self.waypoint_distance)
            if waypoint_circle_large.within(self.flight_poly) and waypoint_circle_small.intersects(self.search_boundaries):
                final_waypoints.append(msg_ned(waypoint.n, waypoint.e))

        if visualize:
            fig, ax = plt.subplots()
            for point in final_waypoints:
                ax.scatter(point.e, point.n, c='k')
            for point in self.flight_boundaries:
                ax.scatter(point.e, point.n, c='r')
            for point in search_boundaries:
                ax.scatter(point.e, point.n, c='g')
            ax.plot([self.flight_boundaries[0].e, self.flight_boundaries[1].e], [self.flight_boundaries[0].n, self.flight_boundaries[1].n],'r')
            ax.plot([self.flight_boundaries[1].e, self.flight_boundaries[2].e], [self.flight_boundaries[1].n, self.flight_boundaries[2].n],'r')
            ax.plot([self.flight_boundaries[2].e, self.flight_boundaries[3].e], [self.flight_boundaries[2].n, self.flight_boundaries[3].n],'r')
            ax.plot([self.flight_boundaries[3].e, self.flight_boundaries[0].e], [self.flight_boundaries[3].n, self.flight_boundaries[0].n],'r')

            ax.plot([search_boundaries[0].e, search_boundaries[1].e], [search_boundaries[0].n, search_boundaries[1].n],'g')
            ax.plot([search_boundaries[1].e, search_boundaries[2].e], [search_boundaries[1].n, search_boundaries[2].n],'g')
            ax.plot([search_boundaries[2].e, search_boundaries[3].e], [search_boundaries[2].n, search_boundaries[3].n],'g')
            ax.plot([search_boundaries[3].e, search_boundaries[4].e], [search_boundaries[3].n, search_boundaries[4].n],'g')
            ax.plot([search_boundaries[4].e, search_boundaries[0].e], [search_boundaries[4].n, search_boundaries[0].n],'g')
            plt.show()
        
        return final_waypoints



        

        

        

