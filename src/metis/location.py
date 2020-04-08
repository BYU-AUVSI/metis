# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

"""
Classes useful for specifying locations and objects in 3-space.
"""

import math

from geographiclib.geodesic import Geodesic
import numpy as np

class GPSWaypoint:
    def __init__(self, lat, lon, elev):
        """`
        A data structure representing a GPS point.

        Parameters
        ----------
        lat : float
            The latitude of the point, expressed in decimal degrees (DD).
        lon : float
            The longitude of the point, expressed in decimal degrees (DD).
        elev : float
            The elevation of the point, in meters.
        """
        self.lat = lat
        self.lon = lon
        self.elev = elev

    def ned_from(self, gps2):
        """
        Returns the NED position of the this GPS point relative to another 
        GPS point.

        Parameters
        ----------
        gps2 : metis.GPSWaypoint
            The point to serve as a reference point. NED position is calculated
            relative to this point.

        Returns
        -------
        res : metis.core.NEDPoint
            A message where the result is the NED position of this point
            relative to gps2 (the radius property is always 0).

        >>> ref = GPSWaypoint(39.98304881111, -111.9903776, 0)
        >>> obstacle = GPSWaypoint(39.983258, -111.994228, 0)
        >>> obstacle.ned_from(ref).to_array()
        [23.234244610397234, -328.88079281265095, 0.0, 0]
        """
        inv = Geodesic.WGS84.Inverse(gps2.lat, gps2.lon, self.lat, self.lon)
        res = NEDPoint(
            n = inv["s12"] * math.cos(math.radians(inv["azi1"])),
            e = inv["s12"] * math.sin(math.radians(inv["azi1"])),
            d = float(-(self.elev - gps2.elev)),
        )
        return res

class NEDPoint(object):
    """Represents a NED point.

    Parameters
    ----------
    n : float
        North coordinate relative to home (in meters).
    e : float
        East coordinate relative to home (in meters).
    d : float
        Down coordinate relative to home (in meters).
    """
    def __init__(self, n=0.0, e=0.0, d=0.0):
        self.ned = np.array([[float(n), float(e), float(d)]])

    @property
    def n(self):
        return self.ned.item(0)
    
    @n.setter
    def n(self, value):
        self.ned[0,0] = float(value)

    @property
    def e(self):
        return self.ned.item(1)
    
    @e.setter
    def e(self, value):
        self.ned[0,1] = float(value)

    @property
    def d(self):
        return self.ned.item(2)
    
    @d.setter
    def d(self, value):
        self.ned[0,2] = float(value)

    def __eq__(self, other):
        return isinstance(other, self.__class__) and \
            np.allclose(self.ned, other.ned)

    def __ne__(self, other):
        return not self.__eq__(other)

    def distance(self, other):
        """Represents a boundary point.

        Parameters
        ----------
        other : metis.core.NEDPoint
            Another NED point to find the Euclidean distance from.

        Returns
        -------
        dist : float
            The distance from one NED point to another.
        """
        return np.linalg.norm(other.ned - self.ned)

class BoundaryPoint(NEDPoint):
    """Represents a boundary point.

    Parameters
    ----------
    n : float
        North coordinate relative to home (in meters).
    e : float
        East coordinate relative to home (in meters).
    d : float
        Down coordinate relative to home (in meters).
    """
    def __init__(self, n=0.0, e=0.0):
        super(BoundaryPoint, self).__init__(n=n, e=e)

class Waypoint(NEDPoint):
    """Represents a point or a position.

    Parameters
    ----------
    n : float, optional
        North coordinate relative to home (in meters), default 0.
    e : float, optional
        East coordinate relative to home (in meters), default 0.
    d : float, optional
        Down coordinate relative to home (in meters), default 0.
    chi : float, optional
        Desired heading at this waypoint (in radians, (-pi, pi]), default 0.
    """
    def __init__(self, n=0.0, e=0.0, d=0.0, chi=0.0):
        super(Waypoint, self).__init__(n=n, e=e, d=d)
        self.chi = chi

class CircularObstacle(NEDPoint):
    """Represents a circular obstacle, defined by a coordinate, height, and 
    radius.

    Parameters
    ----------
    n : float
        North coordinate relative to home (in meters).
    e : float
        East coordinate relative to home (in meters).
    d : float
        Down coordinate relative to home (in meters).
    r : float
        Radius of obstacle (in meters).
    """
    def __init__(self, n=0.0, e=0.0, d=0.0, r=0.0):
        super(CircularObstacle, self).__init__(n=n, e=e, d=d)
        self._r = float(r)

    def __eq__(self, other):
        return super(CircularObstacle, self).__eq__(other) and \
            self.r == other.r

    def __ne__(self, other):
        return super(CircularObstacle, self).__ne__(other) or \
            self.r != other.r

    @property
    def r(self):
        return self._r

    @r.setter
    def r(self, value):
        self._r = float(value)

    @property
    def h(self):
        return -self.d

    @h.setter
    def h(self, value):
        self.d = -value

def convert_point(point, T):
    """
    Parameters
    ----------
    point : NEDPoint object (or one of its subclasses)
        The point to be converted.
    T : NEDPoint type (or one of its subclasses)
        The type to convert `point` to.
    """
    if T not in NEDPoint.__subclasses__():
        raise ValueError('Conversion to type {} is not supported.'.format(T))
    new = T()
    for key in new.__dict__.viewkeys() & point.__dict__.viewkeys():
        new.__dict__[key] = point.__dict__[key]
    return new
