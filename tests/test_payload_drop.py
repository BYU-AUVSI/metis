import pytest
import sys
sys.path.append('../src')

from metis.tools import makeBoundaryPoly, convert#, collisionCheck
from metis.messages import msg_ned
from metis.planners import PayloadPlanner
import numpy as np

def test_payload_drop():
    #List of obastacles and boundaries
    obstaclesList = []
    boundariesList = []
    boundariesList.append(msg_ned(-1000,1000))
    boundariesList.append(msg_ned(-1000,-1000))
    boundariesList.append(msg_ned(1000,-1000))
    boundariesList.append(msg_ned(1000,1000))
    boundaryPoly = makeBoundaryPoly(boundariesList)


    dropLocation = np.array([0.0,0.0,0.0])
    wind_vel = 5.36
    wind = np.array([wind_vel*np.cos(np.radians(30)),-wind_vel*np.sin(np.radians(30)),0.0])
    test = PayloadPlanner(dropLocation,obstaclesList,boundariesList,boundaryPoly,wind)
    test.drop_altitude = 41.611332
    test.chi_offset = np.pi/6.
    test.time_delay = 0.0
    test.time_to_open_parachute = 0.5
    result = test.plan(wind)
    test.plot()
    print(test.NED_release_location)

    drop2 = convert(40.36350202, -111.9019713,0.0,40.36419521, -111.9023111, 41.611332)
    print("Drop 2:",drop2)

    difference = (-test.NED_release_location.item(0)-drop2[0],-test.NED_release_location.item(1)-drop2[1])
    print(difference)

    dropLocation = np.array([0.0,0.0,0.0])
    wind = 2.68
    wind = np.array([0.0,2.68,0.0])
    test = PayloadPlanner(dropLocation,obstaclesList,boundariesList,boundaryPoly,wind)
    test.drop_altitude = 21.141054
    test.chi_offset = 3*np.pi/2
    test.time_delay = 0.0
    test.time_to_open_parachute = 0.5
    result = test.plan(wind)
    test.plot()
    print(test.NED_release_location)

    drop1 = convert(40.36326917, -111.901681,0.0, 40.3633268, -111.9014352, 21.141054)
    print("Drop 1:",drop1)

    difference = (-test.NED_release_location.item(0)-drop1[0],-test.NED_release_location.item(1)-drop1[1])
    print(difference)
