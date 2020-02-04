# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

"""
Python representations of ROS messages required for the metis package.
"""

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

    def __repr__(self):
        return "[{} {} {} {}]".format(self.n, self.e, self.d, self.r)

class NED_pt(object):
    def __init__(self):
        self.N = None
        self.E = None
        self.D = None
        self.task = 0

class NED_list(object):
    def __init__(self):
        self.waypoint_list = []