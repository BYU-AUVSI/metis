import pytest
import sys
sys.path.append('../src')

import numpy as np

from metis import rrt
from metis.location import Waypoint, BoundaryPoint, CircularObstacle
from metis.tools import bounds2poly

def test_stepped_range_linear():
    pass

def test_generate_random_node():
    nmax, nmin = 0.0, -15.3
    emax, emin = 90.66, 0.0
    wpt = rrt.generate_random_node(nmax, nmin, emax, emin)
    print(wpt.n)
    assert wpt.n < nmax and wpt.n > nmin
    assert wpt.e < emax and wpt.e > emin

    nmax, nmin = 1804.0, -15.3
    emax, emin = 0.66, -550.0
    wpt = rrt.generate_random_node(nmax, nmin, emax, emin)
    assert wpt.n < nmax and wpt.n > nmin
    assert wpt.e < emax and wpt.e > emin

def test_heading():
    assert np.degrees(rrt.heading(Waypoint(1,1,0), Waypoint(2,2,0))) == 45.0
    assert np.degrees(rrt.heading(Waypoint(1,1,0), Waypoint(0,0,0))) == -135.0
    assert np.degrees(rrt.heading(Waypoint(1,1,0), Waypoint(2,0,0))) == -45.0
    assert np.degrees(rrt.heading(Waypoint(1,1,0), Waypoint(2,1,0))) == 0.0

def test_wrap():
    assert rrt.wrap(np.radians(170), np.radians(-170)) == -3.316125578789226
    assert rrt.wrap(np.radians(185), np.radians(180)) == 3.2288591161895095
    assert rrt.wrap(np.radians(90), np.radians(160)) == 1.5707963267948966
    assert rrt.wrap(np.radians(100), np.radians(260)) == 1.7453292519943295
    assert rrt.wrap(np.radians(100), np.radians(-100)) == -4.537856055185257

def test_wrap2pi():
    assert rrt.wrap2pi(np.radians(-180)) == 3.141592653589793
    assert rrt.wrap2pi(np.radians(180)) == 3.141592653589793
    assert rrt.wrap2pi(np.radians(-179)) == -3.12413936106985
    assert rrt.wrap2pi(np.radians(380)) == 0.3490658503988664

def test_collision():
    # The reference position
    ref = Waypoint(0, 0, 0)

    # Waypoints
    pts1 = [Waypoint(np.cos(n), n, 0) for n in np.linspace(-10, 10)]
    pts2 = [Waypoint(0, e) for e in np.linspace(6,10)]
    pts3 = [Waypoint(9, e) for e in np.linspace(-8, 0)]
    pts4 = [Waypoint(np.cos(n), n, 0) for n in np.linspace(-6, -3)]

    # Boundaries
    bnd1 = BoundaryPoint(-10, -10)
    bnd2 = BoundaryPoint(-10, 5)
    bnd3 = BoundaryPoint(10, 5)
    bnd4 = BoundaryPoint(10, -10)
    bnds = [bnd1, bnd2, bnd3, bnd4]
    boundaries = bounds2poly(bnds)

    # The obstacles
    ob1 = CircularObstacle(0, 0, 0, 5)

    assert rrt.collision(rrt.waypoints2ned(pts1), boundaries, [ob1], clearance=1) == True
    assert rrt.collision(rrt.waypoints2ned(pts2), boundaries, [ob1], clearance=1) == True
    assert rrt.collision(rrt.waypoints2ned(pts3), boundaries, [ob1], clearance=1) == False
    assert rrt.collision(rrt.waypoints2ned(pts4), boundaries, [ob1], clearance=1) == True

# def test_rrt():
#     show_animation = True


#     #List of obastacles and boundaries
#     obstaclesList = []
#     obstaclesList.append(msg_ned(25.,-25.,150.,20.))
#     obstaclesList.append(msg_ned(60.,60.,150.,20.))
#     # obstaclesList.append(msg_ned(50.,50.,75.,5.))
#     boundariesList = []
#     boundariesList.append(msg_ned(-100,100))
#     boundariesList.append(msg_ned(-100,50))
#     boundariesList.append(msg_ned(-75,50))
#     boundariesList.append(msg_ned(-75,0))
#     boundariesList.append(msg_ned(-100,0))
#     boundariesList.append(msg_ned(-100,-100))
#     boundariesList.append(msg_ned(100,-100))
#     boundariesList.append(msg_ned(100,100))

#     clearance = 5.  # The minimum distance between the path and all obstacles
#     maxDistance = 10.  # Max distance between each added leaf
#     maxIncline = .5  # Max slope the vehicle can climb or decend at
#     maxRelChi = np.pi / 2  # Max relative chi between leaves
#     iterations = 50  # How many sets of random points it will add each time until solution is found
#     resolution = 1.1  # The segment lengths checked for collisions
#     scaleHeight = 1.5  # Scales the height in the cost function for assigning random points to leaves
#     animating = True


#     #List of waypoints
#     waypoints = []
#     # waypoints.append(msg_ned(10.,10., -100.))
#     # waypoints.append(msg_ned(90.,90., -100.))
#     # waypoints.append(msg_ned(80.,10., -100.))
#     # waypoints.append(msg_ned(-40.,-50., -100.))
#     # waypoints.append(msg_ned(30.,30., -100.))
#     # waypoints.append(msg_ned(30.,30., -150.))
#     # waypoints.append(msg_ned(20.,20., -120.))
#     # waypoints.append(msg_ned(25.,25., -100.))
#     # waypoints.append(msg_ned(40.,25., -100.))
#     # waypoints.append(msg_ned(40.,0., -100.))
#     waypoints.append(msg_ned(25., -25., -140.))
#     waypoints.append(msg_ned(20.,20., -100.))
#     waypoints.append(msg_ned(80.,20., -140.))
#     waypoints.append(msg_ned(60., 60., -140.))
#     waypoints.append(msg_ned(-60., 60., -140.))
#     waypoints.append(msg_ned(-80., -60., -140.))
#     waypoints.append(msg_ned(50., 60., -140.))
#     waypoints.append(msg_ned(25., -25., -140.))
#     waypoints.append(msg_ned(25., 20., -100.))

#     # Create rrt object
#     rrt = RRT(obstaclesList, boundariesList, clearance, maxDistance, maxIncline, maxRelChi, iterations, resolution, scaleHeight, animating)

#     # Solve the path
#     solvedPath = rrt.findFullPath(waypoints)
#     # Make sure shows plot even when not debugging??

