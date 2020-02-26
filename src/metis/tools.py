# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from metis.messages import msg_ned
import numpy as np


def collisionCheck(obstaclesList, boundaryPoly, N, E, D, clearance):
    """Checks points for collisions with obstacles and boundaries

	Parameters
	----------
	obstaclesList : msg_ned
		List of obstacles
	boundaryPoly : Polygon
		A Polygon object of the boundaries
	N : np.array
		Arrays of the north position of points
	E : np.array
		Arrays of the east position of points
	D : np.array
		Arrays of the down position of points
	clearance : float
		The amount of clearance desired from obstacles and boundaries

	Returns
	----------
	boolean
		Returns true if a safe path, false if not
	"""
    # First check for collision with obstacles
    for obstacle in obstaclesList:
        # first check if path is above obstacle
        if all(D < -obstacle.d - clearance):
            continue
        # then check if runs into obstacle
        else:
            distToPoint = np.sqrt((N - obstacle.n) ** 2 + (E - obstacle.e) ** 2)
            if any(distToPoint < obstacle.r + clearance):
                return False

    # Check for out of boundaries
    for i in range(0, len(N)):
        if not boundaryPoly.contains(Point(N[i], E[i])):
            return False
    return True


def makeBoundaryPoly(boundariesList):
    """Makes a list of boundary points into a Polygon object.

    Parameters
    ----------
    boundariesList : msg_ned
        List of boundary points

    Returns
    ----------
    boundaries : Polygon
        Returns the Polygon object of boundaries
    """
    pointList = []
    for point in boundariesList:
        pointList.append(Point(point.n, point.e))
    return Polygon([[p.x, p.y] for p in pointList])

