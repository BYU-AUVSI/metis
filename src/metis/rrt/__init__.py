# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg


from . import *
from .rrt_straight import *
from .rrt_fillet import *
from .rrt_dubins import *

from .rrt import *

def get_rrt(arg):
    if arg is 'straight':
        return StraightRRT
    elif arg is 'fillet':
        return FilletRRT
    elif arg is 'dubins':
        return DubinsRRT
    else:
        return StraightRRT
