from shapely.geometry import Point
import numpy as np
from messages.ned import msg_ned

class LoiterPlanner():

    def __init__(self, obstacles, bound_poly, radius=25., clearance=2.5):
        self.obstacles = obstacles
        self.radius = radius
        self.bound_poly = bound_poly
        self.clearance = clearance
        

    def plan(self, waypoints):

        point = Point(waypoints.n, waypoints.e).buffer(self.radius+self.clearance)

        safe_point = True
        if point.within(self.bound_poly):
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
            if point.within(self.bound_poly):
                for obs in self.obstacles:
                    if point.intersects(Point(obs.n, obs.e).buffer(obs.r)):
                        safe_point = False
            
            if np.mod(count,5) == 0:
                std = std + 25

        

        return([msg_ned(n,e,-45.0)])
