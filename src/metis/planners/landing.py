# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import numpy as np

from shapely.geometry import Polygon, LineString, Point

from metis.messages import msg_ned
from metis.planners import Planner
from metis.rrt import heading
from metis.tools import ft2m


class LandingPlanner(Planner):
    """The Landing Planner mixin plans the landing waypoints."""

    # def __init__(self, mission, alt_threshold=10, alt_default=30):
    def __init__(self, mission, approach_Va=14., descent_angle=np.radians(5)):
        super(LandingPlanner, self).__init__(mission)
        # self.altitude_threshold = alt_threshold
        # self.altitude_default = alt_default
        self.approach_Va = approach_Va
        self.descent_angle = descent_angle

    def plan(self, landing_waypoint_list, curr_altitude):
        """Plans the landing waypoints.

        Parameters
        ----------
        landing_waypoint_list : list of metis.messages.msg_ned
        curr_altitude : float
        # altitude_threshold : float (optional)
        # altitude_default : float (optional)

        Returns
        -------
        waypoints : list of metis.messages.msg_ned
            A list of waypoints
        """
        # landing_wpt = landing_waypoint_list[0]
        # approach_wpt = landing_waypoint_list[1]

        # # set landing waypoint to 1 meter above the ground
        # landing_wpt.d = -1

        # # use current altitude if above 10m, otherwise approach from 30m
        # if curr_altitude > self.altitude_threshold:
        #     approach_wpt.d = curr_altitude
        # else:
        #     approach_wpt.d = self.altitude_default

        # return [approach_wpt, landing_wpt]

        debug = True

        touchdown_point = landing_waypoint_list[0]
        touchdown_point.d = 0
        heading_point = landing_waypoint_list[1]
        chi = heading(touchdown_point, heading_point)

        # if debug:
        #     import matplotlib.pyplot as plt
        #     n = np.array([item.n for item in landing_waypoint_list])
        #     e = np.array([item.e for item in landing_waypoint_list])
        #     plt.plot(e, n, 'rx-')
        #     for i in range(len(n)):
        #         plt.text(e[i], n[i], str(i))
        #     plt.axis('equal')
        #     plt.show()

        # obstacles is a list of msg_ned
        obstacles = self.mission.obstacles
        shapes = []
        resolution = 360
        for o in obstacles:
            pts = [(o.r*np.cos(x) + o.e, o.r*np.sin(x) + o.n) for x in np.linspace(0, np.pi*2, resolution)]
            shapes.append(Polygon(pts))

        if debug:
            import matplotlib.pyplot as plt
            n = np.array([item.n for item in landing_waypoint_list])
            e = np.array([item.e for item in landing_waypoint_list])
            plt.plot(e, n, 'rx-')
            for i in range(len(n)):
                plt.text(e[i], n[i], str(i))
            for o in shapes:
                plt.plot(*o.exterior.xy)
            plt.axis('equal')
            plt.show()

        td_pnt_shapely = Point(touchdown_point.e, touchdown_point.n, touchdown_point.d)
        entry_altitude = ft2m(110)
        entry_distance = entry_altitude / np.tan(self.descent_angle)
        entry_pnt_shapely = Point(touchdown_point.e - entry_distance * np.sin(chi), touchdown_point.n - entry_distance * np.cos(chi), -entry_altitude)

        print(td_pnt_shapely)
        print(entry_pnt_shapely)
        line = LineString([td_pnt_shapely, entry_pnt_shapely])

        
        delta_n = entry_distance * np.cos(chi)
        delta_e = entry_distance * np.sin(chi)

        pattern_entry_point = msg_ned(
            north=touchdown_point.n - delta_n,
            east=touchdown_point.e - delta_e,
            down=-entry_altitude
        )

        return [pattern_entry_point, touchdown_point]
