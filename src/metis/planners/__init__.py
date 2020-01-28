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
    def __init__(self, mission):
        self.mission = mission
    
    @property
    def boundary_list(self):
        return self.mission.boundary_list

    @property
    def obstacles(self):
        return self.mission.obstacles

    @property
    def boundary_poly(self):
        return self.mission.boundary_poly

from . import *

from .landing import *
from .loiter import *
from .objective import *
from .offaxis import *
from .payload import *
from .search import *
