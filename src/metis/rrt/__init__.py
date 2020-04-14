# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

"""
RRT Module
----------

Someday, it would be nice to implement a version that also takes into account
speeds. For example, on longer straight runs, speeds could be increased, and
on tight turns, speeds could go around the minimum aircraft speed.
"""

from . import *
from .rrt_base import *
from .rrt_straight import StraightRRT
from .rrt_fillet import FilletRRT
from .rrt_dubins import DubinsRRT
from .rrtstar_straight import StraightRRTStar
from .rrtstar_dubins import DubinsRRTStar

mode = 'straight'

def get_mode():
    global mode
    return mode

modes = {
    'straight': StraightRRT,
    'straightstar': StraightRRTStar,
    'fillet': FilletRRT,
    'dubins': DubinsRRT,
    'dubinsstar': DubinsRRTStar,
}

def get_rrt(arg=mode):
    global mode
    print(arg.lower())
    if arg.lower() in modes.keys():
        mode = arg.lower()
        return modes[arg.lower()]
    else:
        raise ValueError("Nonexistent RRT mode '{}' requested.".format(mode))
