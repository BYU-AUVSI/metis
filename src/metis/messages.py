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

class NED_pt(object):
    def __init__(self):
        self.N = None
        self.E = None
        self.D = None
        self.task = 0

class NED_list(object):
    def __init__(self):
        self.waypoint_list = []