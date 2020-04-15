# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import logging

import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D


_module_logger = logging.getLogger(__name__)

class Animation(object):
    """
    Animation class provides easy utility for visualizing planner paths in 
    real-time.

    This class is intended to be subclassed and not used directly.

    Parameters
    ----------
    mission : metis.core.Mission
        A mission object, from which all the details of what should be included
        on the map are taken.
    """
    _logger = _module_logger.getChild('Animation')

    def __init__(self, mission, name):
        # mpl.rcParams['legend.fontsize'] = 10
        plt.ion()
        self.obstacles = mission.obstacles
        self.boundaries = mission.boundary_list
        self.bound_poly = mission.boundary_poly
        self.waypoints = mission.waypoints
        self.fig, self.ax = plt.subplots(num=name)
        plt.pause(0.000001)


class Animation2D(Animation):
    """
    A 2D animation utility for visualizing planner plots in real-time.

    Parameters
    ----------
    mission : metis.core.Mission
        A mission object, from which all the details of what should be included
        on the map are taken.
    """
    def __init__(self, mission):
        # Backends:
        # https://stackoverflow.com/questions/30809316/how-to-create-a-plot-in-matplotlib-without-using-pyplot
        super(Animation2D, self).__init__(mission, '2D Animation')
        # nodes is a dictionary from Node to matplotlib Line objects.
        self.nodes = {}
        # ax = self.ax
        for obstacle in self.obstacles:
            self.ax.add_artist(plt.Circle((obstacle.e, obstacle.n), obstacle.r, color='r'))
        for point in self.waypoints:
            self.ax.plot(point.e, point.n, 'rx')
        self.ax.plot(*self.bound_poly.exterior.xy)
        minE, minN, maxE, maxN = self.bound_poly.bounds
        self.ax.set_xlim((int(1.1*minE), int(1.1*maxE)))
        self.ax.set_ylim((int(1.1*minN), int(1.1*maxN)))
        self.ax.set_aspect('equal')  
        plt.pause(0.000001)

    @staticmethod
    def ne2xy(ne):
        pass

    def add_path(self, ned, valid=False):
        """
        Adds colored paths between the list of waypoints received. Or, adds 
        gray paths between the list of nodes received.

        Parameters
        ----------
        waypoints : list of metis.messages.msg_ned
            A list of waypoint messages.
        valid : bool
            Paths are final, not random potential paths (default False). If
            True, paths are overlaid in black.
        
        Notes
        -----
        This function calls `plt.pause` after each invocation. This pauses
        execution of Python code to update the GUI loop. Performance of 
        functions that call this routine is therefore impacted as they are
        interrupted on each invocation to update the plot. Planners that don't
        include visualization will run much faster than those that do. 
        However, these visualizations are very useful for debugging. It is
        therefore suggested to implement planners in such a way that animations
        can be optionally set to on or off.
        """
        if valid:
            self.ax.plot(ned[:,1], ned[:,0], 'k-')
        else:
            self.ax.plot(ned[:,1], ned[:,0])
        plt.pause(0.000001)

    # def add_waypoint(self, n, e, d):
    #     self.ax.plot(e, n, 'rx')
    #     self.ax.text(e, n, '{:.1f}'.format(-d))
    #     plt.pause(0.000001)

    def remove_node(self, node):
        pass

# class Animation3D(Animation):
#     def __init__(self, obstacles, boundaries):
#         super(Animation3D, self).__init__(obstacles, boundaries)

#     def update(self):
#         plt.cla()
#         for obstacle in self.obstacles:
#             # Cylinder
#             x = np.linspace((obstacle.n - obstacle.r), (obstacle.n + obstacle.r), 100)
#             z = np.linspace(0, obstacle.d, 100)
#             # x = np.linspace(-1, 1, 25)
#             # z = np.linspace(-2, 2, 25)
#             Xc, Zc = np.meshgrid(x, z)
#             Yc = np.sqrt(obstacle.r**2 - (Xc - obstacle.n)**2) + obstacle.e

#             # Draw parameters
#             self.ax.plot_surface(Xc, Yc, Zc, alpha=0.9, color='b')
#             self.ax.plot_surface(Xc, (2.*obstacle.e-Yc), Zc, alpha=0.9, color='b')
#         plt.axis("equal")

        # for obstacle in obstacles:
        #     # Cylinder
        #     x = np.linspace((obstacle.n - obstacle.r), (obstacle.n + obstacle.r), 100)
        #     z = np.linspace(0, obstacle.d, 100)
        #     # x = np.linspace(-1, 1, 25)
        #     # z = np.linspace(-2, 2, 25)
        #     Xc, Zc = np.meshgrid(x, z)
        #     Yc = np.sqrt(obstacle.r**2 - (Xc - obstacle.n)**2) + obstacle.e

        #     # Draw parameters
        #     self.ax.plot_surface(Xc, Yc, Zc, alpha=0.9, color='b')
        #     self.ax.plot_surface(Xc, (2.*obstacle.e-Yc), Zc, alpha=0.9, color='b')
        # first = True
        # boundaries = []
        # last = []
        # for bounds in boundaries:
        #     if first:
        #         boundaries = np.array([[bounds.n, bounds.e, 0.]])
        #         last = np.array([[bounds.n, bounds.e, 0.]])
        #         first = False
        #         continue
        #     boundaries = np.append(boundaries, [[bounds.n, bounds.e, 0.]], axis=0)
        # boundaries = np.append(boundaries, last, axis=0)
        # self.ax.plot(boundaries[:, 0], boundaries[:, 1], boundaries[:, 2], label='Boundaries')
        # self.ax.set_xlabel('X axis')
        # self.ax.set_ylabel('Y axis')
        # self.ax.set_zlabel('Z axis')
        # plt.xlim(self.maxN*1.1, self.minN*1.1)
        # plt.ylim(self.minE * 1.1, self.maxE * 1.1)
        # self.ax.elev = 90 #55
        # self.ax.azim = 0 #80
        # self.viridis = cm.get_cmap('viridis', 12)
        # self.viridis = cm.get_cmap('viridis')
        # self.ax.legend()