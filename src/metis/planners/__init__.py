__all__ = [
    'landing',
    'loiter',
    'objective',
    'offaxis',
    'payload',
    'search',
]

class Planner(object):
    pass


class PlannerData(object):
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
