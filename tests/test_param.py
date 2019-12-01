import pytest
import sys
sys.path.append('../src')

from metis.messages import msg_ned
from metis.tools import convert, makeBoundaryPoly

def test_param():
    # Testing locations
    # All lists are in North, East, Down, Radius
    drop_location_list = [100, -100, 0, 0]

    obstacle_list = [[50, 50, -100, 20],
                    [150, -200, -75, 40],
                    [-200, 35, -80, 25],
                    [150, -75, -100, 30]]

    boundariesList = []

    home = [38.146269,-76.428164, 0.0]
    bd0 = [38.146269,-76.428164, 0.0]
    bd1 = [38.151625,-76.428683, 0.0]
    bd2 = [38.151889, -76.431467, 0.0]
    bd3 = [38.150594, -76.435361, 0.0]
    bd4 = [38.147567, -76.432342, 0.0]
    bd5 = [38.144667, -76.432947, 0.0]
    bd6 = [38.143256, -76.434767, 0.0]
    bd7 = [38.140464, -76.432636, 0.0]
    bd8 = [38.140719, -76.426014, 0.0]
    bd9 = [38.143761, -76.421206, 0.0]
    bd10 = [38.147347, -76.423211, 0.0]
    bd11 = [38.146131, -76.426653, 0.0]

    bd0_m = convert(home[0],home[1],home[2],bd0[0],bd0[1],bd0[2])
    bd1_m = convert(home[0],home[1],home[2],bd1[0],bd1[1],bd1[2])
    bd2_m = convert(home[0],home[1],home[2],bd2[0],bd2[1],bd2[2])
    bd3_m = convert(home[0],home[1],home[2],bd3[0],bd3[1],bd3[2])
    bd4_m = convert(home[0],home[1],home[2],bd4[0],bd4[1],bd4[2])
    bd5_m = convert(home[0],home[1],home[2],bd5[0],bd5[1],bd5[2])
    bd6_m = convert(home[0],home[1],home[2],bd6[0],bd6[1],bd6[2])
    bd7_m = convert(home[0],home[1],home[2],bd7[0],bd7[1],bd7[2])
    bd8_m = convert(home[0],home[1],home[2],bd8[0],bd8[1],bd8[2])
    bd9_m = convert(home[0],home[1],home[2],bd9[0],bd9[1],bd9[2])
    bd10_m = convert(home[0],home[1],home[2],bd10[0],bd10[1],bd10[2])
    bd11_m = convert(home[0],home[1],home[2],bd11[0],bd11[1],bd11[2])

    boundariesList.append(msg_ned(bd0_m[0],bd0_m[1]))
    boundariesList.append(msg_ned(bd1_m[0],bd1_m[1]))
    boundariesList.append(msg_ned(bd2_m[0],bd2_m[1]))
    boundariesList.append(msg_ned(bd3_m[0],bd3_m[1]))
    boundariesList.append(msg_ned(bd4_m[0],bd4_m[1]))
    boundariesList.append(msg_ned(bd5_m[0],bd5_m[1]))
    boundariesList.append(msg_ned(bd6_m[0],bd6_m[1]))
    boundariesList.append(msg_ned(bd7_m[0],bd7_m[1]))
    boundariesList.append(msg_ned(bd8_m[0],bd8_m[1]))
    boundariesList.append(msg_ned(bd9_m[0],bd9_m[1]))
    boundariesList.append(msg_ned(bd10_m[0],bd10_m[1]))
    boundariesList.append(msg_ned(bd11_m[0],bd11_m[1]))
    boundaryPoly = makeBoundaryPoly(boundariesList)

    drop_location = msg_ned(north=100., east=-100.)

    obstacles = []
    for ind_obstacle in obstacle_list:
        obstacles.append(msg_ned(ind_obstacle[0], ind_obstacle[1], ind_obstacle[3], ind_obstacle[3]))

    boundaries = boundaryPoly