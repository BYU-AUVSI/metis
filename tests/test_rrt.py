import pytest
import sys
sys.path.append('../src')

from metis.rrt import RRT
import numpy as np
# import time
from metis.messages import msg_ned


def test_rrt():
    show_animation = True


    #List of obastacles and boundaries
    obstaclesList = []
    obstaclesList.append(msg_ned(25.,-25.,150.,20.))
    obstaclesList.append(msg_ned(60.,60.,150.,20.))
    # obstaclesList.append(msg_ned(50.,50.,75.,5.))
    boundariesList = []
    boundariesList.append(msg_ned(-100,100))
    boundariesList.append(msg_ned(-100,50))
    boundariesList.append(msg_ned(-75,50))
    boundariesList.append(msg_ned(-75,0))
    boundariesList.append(msg_ned(-100,0))
    boundariesList.append(msg_ned(-100,-100))
    boundariesList.append(msg_ned(100,-100))
    boundariesList.append(msg_ned(100,100))

    clearance = 5.  # The minimum distance between the path and all obstacles
    maxDistance = 10.  # Max distance between each added leaf
    maxIncline = .5  # Max slope the vehicle can climb or decend at
    maxRelChi = np.pi / 2  # Max relative chi between leaves
    iterations = 50  # How many sets of random points it will add each time until solution is found
    resolution = 1.1  # The segment lengths checked for collisions
    scaleHeight = 1.5  # Scales the height in the cost function for assigning random points to leaves
    animating = True


    #List of waypoints
    waypoints = []
    # waypoints.append(msg_ned(10.,10., -100.))
    # waypoints.append(msg_ned(90.,90., -100.))
    # waypoints.append(msg_ned(80.,10., -100.))
    # waypoints.append(msg_ned(-40.,-50., -100.))
    # waypoints.append(msg_ned(30.,30., -100.))
    # waypoints.append(msg_ned(30.,30., -150.))
    # waypoints.append(msg_ned(20.,20., -120.))
    # waypoints.append(msg_ned(25.,25., -100.))
    # waypoints.append(msg_ned(40.,25., -100.))
    # waypoints.append(msg_ned(40.,0., -100.))
    waypoints.append(msg_ned(25., -25., -140.))
    waypoints.append(msg_ned(20.,20., -100.))
    waypoints.append(msg_ned(80.,20., -140.))
    waypoints.append(msg_ned(60., 60., -140.))
    waypoints.append(msg_ned(-60., 60., -140.))
    waypoints.append(msg_ned(-80., -60., -140.))
    waypoints.append(msg_ned(50., 60., -140.))
    waypoints.append(msg_ned(25., -25., -140.))
    waypoints.append(msg_ned(25., 20., -100.))

    # Create rrt object
    rrt = RRT(obstaclesList, boundariesList, clearance, maxDistance, maxIncline, maxRelChi, iterations, resolution, scaleHeight, animating)

    # Solve the path
    solvedPath = rrt.findFullPath(waypoints)
    # Make sure shows plot even when not debugging??

