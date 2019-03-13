import rrt
import numpy as np
from messages.ned import msg_ned

show_animation = True

#List of obastacles and boundaries
obstaclesList = []
obstaclesList.append(msg_ned(0.,0.,50.,20.))
boundariesList = []
boundariesList.append(msg_ned(-100,100))
boundariesList.append(msg_ned(-100,50))
boundariesList.append(msg_ned(-75,50))
boundariesList.append(msg_ned(-75,0))
boundariesList.append(msg_ned(-100,0))
boundariesList.append(msg_ned(-100,-100))
boundariesList.append(msg_ned(100,-100))
boundariesList.append(msg_ned(100,100))

animating = True

#List of waypoints
waypoints = []
waypoints.append(msg_ned(25.,25., -100.))
waypoints.append(msg_ned(40.,25., -100.))
waypoints.append(msg_ned(40.,0., -100.))
waypoints.append(msg_ned(20.,20., -100.))

#Create rrt object
rrt = RRT(obstaclesList, boundariesList, animating)
rrt.findPath(waypoints)
