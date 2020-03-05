import os, pickle

import numpy as np

from metis.core import Mission, Plan
from metis.manager import MissionManager
from metis import rrt

with open(os.path.join('/home/sequoia/ros/uas_ws/src/metis/scripts', 'mis.pkl')) as f:
    mission = pickle.load(f)

manager = MissionManager(mission)
plan = manager.plan("objective")
# plan = manager.plan("search")
plan.plot()
