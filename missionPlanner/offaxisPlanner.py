import sys
sys.path.append('..')

import numpy as np
from messages.ned import msg_ned
from tools.tools import makeBoundaryPoly
from shapely.geometry import Point
import matplotlib.pyplot as plt

class OffaxisPlanner():

    def __init__(self, boundary_list, obstacles):
        self.boundary_list = boundary_list
        self.boundary_poly = makeBoundaryPoly(self.boundary_list)
        self.obstalces = obstacles
    
    def plan(self, object_location_list, altitude=70, clearance=10, waypoint_distance=100):
        object_location = object_location_list[0]
        location_point = Point(object_location.n, object_location.e).buffer(clearance)

        if location_point.within(self.boundary_poly):
            waypoint = msg_ned(object_location.n, object_location.e, -altitude)
            final_waypoints = [waypoint]
        
        else:
            dist = np.zeros(len(self.boundary_list))
            norm = np.zeros((len(self.boundary_list),2))
            slope = np.zeros((len(self.boundary_list),2))
            for i in np.arange(len(self.boundary_list)):
                j = i + 1
                if i == len(self.boundary_list) - 1:
                    j = 0
                
                dn = self.boundary_list[i].n - self.boundary_list[j].n
                de = self.boundary_list[i].e - self.boundary_list[j].e 

                pn = object_location.n - self.boundary_list[j].n
                pe = object_location.e - self.boundary_list[j].e

                slope[i,:] = np.array([[dn, de]])/np.linalg.norm(np.array([[dn, de]]))
                norm[i,:] = np.array([[de, -dn]])/np.linalg.norm(np.array([[de, -dn]]))
                point = np.array([[pn, pe]])

                dist[i] = np.abs(np.dot(norm[i,:],point[0]))
            
            line_idx = np.argmin(dist)
            waypoint_n = object_location.n + norm[line_idx,0]*(dist[line_idx] + clearance)
            waypoint_e = object_location.e + norm[line_idx,1]*(dist[line_idx] + clearance)

            direction = 1

            if Point(waypoint_n, waypoint_e).within(self.boundary_poly):
                waypoint = msg_ned(waypoint_n, waypoint_e, -altitude)
            else:
                direction = -1
                waypoint_n = object_location.n - norm[line_idx,0]*(dist[line_idx] + clearance)
                waypoint_e = object_location.e - norm[line_idx,1]*(dist[line_idx] + clearance)

                waypoint = msg_ned(waypoint_n, waypoint_e, -altitude)
            
            parallel_point = msg_ned(waypoint.n + waypoint_distance*slope[line_idx,0], waypoint.e + waypoint_distance*slope[line_idx,1], altitude)

            if Point(parallel_point.n, parallel_point.e).buffer(clearance*.99).within(self.boundary_poly):
                pass
            else:
                parallel_point.n = waypoint.n - waypoint_distance*slope[line_idx,0]
                parallel_point.e = waypoint.e - waypoint_distance*slope[line_idx,1]
            
            end_point = msg_ned(waypoint.n + direction*waypoint_distance*norm[line_idx,0], waypoint.e +  direction*waypoint_distance*norm[line_idx,1], altitude)
            scale = 1.
            while(not Point(end_point.n, end_point.e).buffer(clearance).within(self.boundary_poly)):
                end_point.n = waypoint.n + direction*waypoint_distance*norm[line_idx,0]*scale
                end_point.e = waypoint.e + direction*waypoint_distance*norm[line_idx,1]*scale
                scale = scale - .05

            final_waypoints = [parallel_point, waypoint, end_point]


            
        if True: # This is for debugging/visualization purposes only. I don't think we need/want this during the competition but it is nice to use when debugging
            fig, ax = plt.subplots()
            for point in final_waypoints:
                ax.scatter(point.e, point.n, c='k')
            ax.scatter(object_location.e, object_location.n)
            for point in self.boundary_list:
                ax.scatter(point.e, point.n, c='r')
            ax.plot([self.boundary_list[0].e, self.boundary_list[1].e], [self.boundary_list[0].n, self.boundary_list[1].n],'r')
            ax.plot([self.boundary_list[1].e, self.boundary_list[2].e], [self.boundary_list[1].n, self.boundary_list[2].n],'r')
            ax.plot([self.boundary_list[2].e, self.boundary_list[3].e], [self.boundary_list[2].n, self.boundary_list[3].n],'r')
            ax.plot([self.boundary_list[3].e, self.boundary_list[0].e], [self.boundary_list[3].n, self.boundary_list[0].n],'r')

            ax.axis('equal')
            plt.show()

        return final_waypoints


        


