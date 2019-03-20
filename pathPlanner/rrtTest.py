from rrt import RRT
import numpy as np
import time
from messages.ned import msg_ned

show_animation = True

#List of obastacles and boundaries
obstaclesList = []
obstaclesList.append(msg_ned(25.,-25.,100.,20.))
obstaclesList.append(msg_ned(60.,60.,110.,20.))
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

animating = True

#List of waypoints
waypoints = []
waypoints.append(msg_ned(10.,10., -100.))
waypoints.append(msg_ned(90.,90., -100.))
waypoints.append(msg_ned(80.,10., -100.))
waypoints.append(msg_ned(-40.,-50., -100.))
waypoints.append(msg_ned(30.,30., -100.))
# waypoints.append(msg_ned(30.,30., -150.))
# waypoints.append(msg_ned(20.,20., -120.))
# waypoints.append(msg_ned(25.,25., -100.))
# waypoints.append(msg_ned(40.,25., -100.))
# waypoints.append(msg_ned(40.,0., -100.))
# waypoints.append(msg_ned(20.,20., -100.))
# waypoints.append(msg_ned(80.,20., -140.))

#Create rrt object
rrt = RRT(obstaclesList, boundariesList, animating)
rrt.findFullPath(waypoints)
# time.sleep(5.5)
