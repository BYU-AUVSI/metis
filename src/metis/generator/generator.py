from metis import Mission, MissionPlotter
from metis.messages import msg_ned
from metis.tools import makeBoundaryPoly, collisionCheck
from metis.plotter import MissionPlotter

import numpy as np

class MissionGenerator(object):
    def __init__(self):
        self.mission = Mission()
        self.mission.boundary_list = self.generate_boundaries()
        self.mission.boundary_poly = makeBoundaryPoly(self.mission.boundary_list)
        # Obstacles must be created before waypoints and payload drop
        self.mission.obstacles = self.generate_obstacles()
        self.mission.waypoints = self.generate_waypoints()
        self.mission.drop_location = self.generate_payload()
        self.mission.search_area = self.generate_search()
        self.mission.offaxis_location = []

    def generate_home(self):
        home = msg_ned(0., 0.)
        return home

    def generate_boundaries(self):
        bnd = []
        bnd.append(msg_ned(500., 500.))
        bnd.append(msg_ned(500., -500.))
        bnd.append(msg_ned(-500., -500.))
        bnd.append(msg_ned(-500., 500.))
        return bnd

    def generate_obstacles(self, num=10):
        obs = []
        n = 1000 * np.random.random_sample((num,)) - 500
        e = 1000 * np.random.random_sample((num,)) - 500
        d = 50 * np.random.random_sample((num,))
        r = np.abs(50 * np.random.randn(num) + 10)
        for i in range(num):
            msg = msg_ned(n[i], e[i], d[i], r[i])
            obs.append(msg)
        return obs

    def generate_waypoints(self, num=10):
        wpt = []
        for i in range(num):
            north, east, down = self._generate_ned()
            while not collisionCheck(self.mission.obstacles, self.mission.boundary_poly, north, east, down, 5.):
                north, east, down = self._generate_ned()
            msg = msg_ned(north[0], east[0], down[0])
            wpt.append(msg)
        return wpt

    def generate_payload(self):
        north, east, down = self._generate_ned()
        while not collisionCheck(self.mission.obstacles, self.mission.boundary_poly, north, east, down, 5.):
            north, east, down = self._generate_ned()
        return msg_ned(north[0], east[0], down[0])

    def _generate_ned(self):
        n = np.array([1000 * np.random.random_sample() - 500])
        e = np.array([1000 * np.random.random_sample() - 500])
        d = np.array([100 * np.random.random_sample() + 150])
        return n, e, d

    def generate_search(self):
        bnd = []
        bnd.append(msg_ned(100., 100.))
        bnd.append(msg_ned(100., -100.))
        bnd.append(msg_ned(-100., -100.))
        bnd.append(msg_ned(-100., 100.))
        return bnd

    def get_mission(self):
        return self.mission


if __name__ == "__main__":
    mission = MissionGenerator().get_mission()
    mp = MissionPlotter(mission)
    mp.show()
