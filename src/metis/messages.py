# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

"""
Python representations of ROS messages required for the metis package.
"""

import numpy as np


class msg_ned(object):
    """
    A North-East-Down (NED) representation of an object or position.

    All position are assumed relative to some known home location and are 
    represented in meters.

    Parameters
    ----------
    north : float, optional
        The north position from home in meters (default 0).
    east : float, optional
        The east position from home in meters (default 0).
    down : float, optional
        The down position from home in meters (default 0). Altitudes above ground level
        are therefore represented as negative values. For obstacles, the height
        is stored in this attribute.
    radius : float, optional
        Radius of object in meters (default 0). Used for obstacles.
    """
    def __init__(self, north=0., east=0., down=0., radius=0.):
        self.n = north
        self.e = east
        self.d = down
        self.r = radius

    def __repr__(self):
        return "[{} {} {} {}]".format(self.n, self.e, self.d, self.r)

    def __add__(self, other):
        """
        Adds two NED waypoints together and retains the maximum
        radius, if any.
        """
        if type(other) == type(msg_ned):
            return msg_ned(self.n + other.n, self.e + other.e, self.d + other.d, max(self.r, other.r))
        elif isinstance(other, np.ndarray) and (other.shape[0] == 3):
            return msg_ned(self.n + other.item(0), self.e + other.item(1), self.d + other.item(2), self.r)
        elif isinstance(other, np.ndarray) and (other.shape[0] == 4):
            return msg_ned(self.n + other.item(0), self.e + other.item(1), self.d + other.item(2), max(self.r, other.item(3)))
        else:
            raise TypeError("Addition is not supported for type {}".format(type(other)))

    def __sub__(self, other):
        """
        Subtracts two NED waypoints from each other and retains the maximum
        radius, if any.
        """
        return msg_ned(self.n-other.n, self.e-other.e, self.d-other.d, max(self.r, other.r))

    def __mul__(self, other):
        if type(other) == float or type(other) == int:
            val = float(other)
            return msg_ned(val*self.n, val*self.e, val*self.d, val*self.r)
        else:
            raise TypeError("Multiplication of type {} with msg_ned is undefined.".format(type(other)))

    def __eq__(self, other):
        return isinstance(other, msg_ned) and \
            float(self.n) == float(other.n) and \
            float(self.e) == float(other.e) and \
            float(self.d) == float(other.d) and \
            float(self.r) == float(other.r)

    def __ne__(self, other):
        return not self.__eq__(other)

    def to_array(self, radius=True):
        """
        Returns the msg_ned object as a list.

        Parameters
        ----------
        radius : bool, optional
            Determines whether the radius should be included in the array.
            (default is True).
        
        Returns
        -------
        list
            The msg_ned as a list of length 4, if all fields are included, or
            length 3, if `radius = False`.
            Format: [n, e, d, (r)]
        """
        if radius:
            return [float(self.n), float(self.e), float(self.d), float(self.r)]
        else:
            return [float(self.n), float(self.e), float(self.d)]

    def to_nparray(self, radius=False):
        """
        A function for easily converting the waypoint to a numpy array (ordered
        n, e, d[, r]), optionally including object radius.

        Parameters
        ----------
        radius : bool
            Whether to include radius or not. Default false.
        """
        if radius:
            return np.array([float(self.n), float(self.e), float(self.d), float(self.r)])
        else:
            return np.array([float(self.n), float(self.e), float(self.d)])
    
    @property
    def nparray(self):
        """
        A property for easily accessing the waypoint as a numpy array (ordered
        n, e, d) without including object radius.
        """
        return self.to_nparray()

class NED_pt(object):
    def __init__(self):
        self.N = None
        self.E = None
        self.D = None
        self.task = 0

class NED_list(object):
    def __init__(self):
        self.waypoint_list = []