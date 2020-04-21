# -*- coding: utf-8 -*-
# Copyright 2019-2020 Sequoia Ploeg

import matplotlib.pyplot as plt
import numpy as np
from numpy.random import rand

from metis import Mission
from metis.manager import MissionManager
from metis.location import Waypoint, BoundaryPoint, CircularObstacle
from metis.rrt import get_rrt, Animation2D

# np.random.seed(1111)
np.random.seed(150)
# np.random.seed(230)
# np.random.seed(5280) # Very fast and simple
# np.random.seed(99124) # Also very easy
# np.random.seed(521)
# np.random.seed(1527)
# np.random.seed(178926053)

# obstacle_count = 30 # For straight line RRT
obstacle_count = 60 # For Dubins paths RRT

clearance = 15.0
obstacles = [
    CircularObstacle(n=1000*rand(), e=1000*rand(), d=-200*rand()-100, r=50*rand()+10)
    # CircularObstacle(n=1000*rand(), e=1000*rand(), d=-500, r=50*rand()+10)
    for i in range(obstacle_count)
]
bounds = [
    BoundaryPoint(0.0, 0.0),
    BoundaryPoint(0.0, 1000.0),
    BoundaryPoint(1000.0, 1000.0),
    BoundaryPoint(1000.0, 0.0),
]
waypoints = [
    Waypoint(100, 750, -300),
    Waypoint(860, 700, -100),
    Waypoint(920, 150, -200),
    Waypoint(100, 130, -50),
]
obstacles = [obstacle for obstacle in obstacles if all(waypoint.distance(obstacle, d2=True) > obstacle.r + clearance for waypoint in waypoints)]

fig, ax = plt.subplots()
bound_e, bound_n = [bound.e for bound in bounds] + [bounds[0].e], [bound.n for bound in bounds] + [bounds[0].n]
ax.plot(bound_e, bound_n, 'b-')
for obstacle in obstacles:
    circ = plt.Circle((obstacle.e, obstacle.n), obstacle.r, fill=False, color='red')
    ax.add_artist(circ)
    ax.text(obstacle.e, obstacle.n, "{:.0f}".format(-obstacle.d))
for point in waypoints:
    ax.plot(point.e, point.n, 'rx')
ax.axis('equal')
plt.show()

mission = Mission(
    home = None,
    obstacles = obstacles,
    boundary_list = bounds,
    waypoints = waypoints,
    drop_location = None,
    search_area = None,
    offaxis_location = None
)

starting_pos = Waypoint(10., 10., -35.0, 0.0)
# RRT = get_rrt('straightstar')
RRT = get_rrt('dubinsstar')
rrt = RRT(mission)
waypoints.insert(0, starting_pos)
final_path = rrt.find_full_path(waypoints, connect=True)

ani = Animation2D(mission)
for p1, p2 in zip(final_path[:-1], final_path[1:]):
    ani.add_path(rrt.points_along_path(p1, p2, 30.0, 0.5))
for way in final_path:
    ani.add_waypoint(way.n, way.e, way.d)