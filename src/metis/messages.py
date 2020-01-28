# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

"""
Python representations of ROS messages required for the metis package.
"""

class msg_ned(object):
    def __init__(self, north=0., east=0., down=0., radius=0.):
        self.n = north  # North position
        self.e = east  # East position
        self.d = down  # Down position. Height for obstacles
        self.r = radius  # Radius for obstacles. Not sure if there is another message type for this

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