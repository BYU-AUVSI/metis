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

mode = 'straight'

def get_mode():
    global mode
    return mode

def get_rrt(arg=mode):
    global mode
    print(arg.lower())
    if arg.lower() == 'straight':
        mode = 'straight'
        return StraightRRT
    elif arg.lower() == 'fillet':
        mode = 'fillet'
        return FilletRRT
    elif arg.lower() == 'dubins':
        mode = 'dubins'
        return DubinsRRT
    else:
        raise ValueError("Nonexistent RRT mode '{}' requested.".format(mode))
