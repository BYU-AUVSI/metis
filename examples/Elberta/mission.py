from .map import *

# Create a mission model in metis

mission = Mission()
mission.home = home

# Create a mission manager, instantiated with a mission
mm = MissionManager(mission)

# Mission manager handles all the subplanners. This means we can simply
# plan paths for the various missions, right from here.
plan = mm.plan("search")
plan = mm.plan("waypoint")
plan = mm.plan("payload")

# This will return path objects that contain:
#   .plot(): of potential paths.
#   .accept(): function for executing