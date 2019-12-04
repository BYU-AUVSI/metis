# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

"""
The `planners` submodule. It has no dependencies on ros.
"""

__all__ = [
    'landing',
    'loiter',
    'objective',
    'offaxis',
    'payload',
    'search',
]

class Planner(object):
    def __init__(self, boundary_list, obstacles, boundary_poly=None):
        self.boundary_list = boundary_list
        self.obstacles = obstacles
        self.boundary_poly = makeBoundaryPoly(self.boundary_list) if boundary_poly is None else boundary_poly
    

from . import *

from .landing import *
from .loiter import *
from .objective import *
from .offaxis import *
from .payload import *
from .search import *
