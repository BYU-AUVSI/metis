import os, pickle

from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import numpy as np

from metis.manager import MissionManager

with open('./mission.pkl', 'r') as f:
    mission = pickle.load(f)

print(mission)

manager = MissionManager(mission)

plan = manager.plan_objective()
plan.plot()
