# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from metis.messages import msg_ned


def will_collide(obstacles, boundaryPoly, N, E, D, clearance):
    """
    Checks points for collisions with obstacles and boundaries.

	Parameters
	----------
	obstacles : msg_ned
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
		Returns True if waypoints will collide, False if safe.
	"""
    # First check for collision with obstacles
    for obstacle in obstacles:
        # first check if path is above obstacle
        if all(D < -obstacle.d - clearance):
            continue
        # then check if runs into obstacle
        else:
            distToPoint = np.sqrt((N - obstacle.n) ** 2 + (E - obstacle.e) ** 2)
            if any(distToPoint < obstacle.r + clearance):
                return True

    # Check for out of boundaries
    for i in range(len(N)):
        if not boundaryPoly.contains(Point(N[i], E[i])):
            return True
    return False


def down_at_ne(n, e):
    pass

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


def bounds2poly(boundaries):
    """
    Makes a Polygon object from a list of boundary points.

    Parameters
    ----------
    boundaries : list of metis.messages.msg_ned
        List of boundary points.

    Returns
    -------
    boundaries : shapely.geometry.polygon.Polygon
        Returns the Polygon representing the boundaries.
    """
    return Polygon([[bound.n, bound.e] for bound in boundaries])


def ft2m(feet):
    """
    Convenience function for converting from feet to meters.

    Parameters
    ----------
    feet : float
        Length in feet.

    Returns
    -------
    float
        Length in meters.
    """
    return float(feet / 3.2808)