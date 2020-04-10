# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


def will_collide(obstacles, boundaryPoly, N, E, D, clearance):
    """
    Checks points for collisions with obstacles and boundaries.

	Parameters
	----------
	obstacles : list of metis.core.CircularObstacle
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
            # dist = np.linalg.norm(ned - obstacle.ned)
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


def bounds2poly(boundaries):
    """
    Makes a Polygon object from a list of boundary points.

    Parameters
    ----------
    boundaries : list of metis.core.BoundaryPoint
        List of boundary points.

    Returns
    -------
    boundaries : shapely.geometry.polygon.Polygon
        Returns the Polygon representing the boundaries.
    """
    return Polygon([[bound.e, bound.n] for bound in boundaries])


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
    
def m2ft(meters):
    """
    Convenience function for converting from meters to feet.

    Parameters
    ----------
    meters : float
        Length in meters.

    Returns
    -------
    float
        Length in feet.
    """
    return float(meters * 3.2808)
