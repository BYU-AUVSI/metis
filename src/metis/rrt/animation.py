# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import logging

import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D


_module_logger = logging.getLogger(__name__)
plt.ion()

class Animation(object):
    _logger = _module_logger.getChild('Animation')

    def __init__(self, mission, name):
        # mpl.rcParams['legend.fontsize'] = 10
        self.obstacles = mission.obstacles
        self.boundaries = mission.boundary_list
        self.bound_poly = mission.boundary_poly
        self.fig, self.ax = plt.subplots(num=name)
        plt.pause(0.001)

    def drawPath(self, path, color):
        # """ RRT class function that draws the path between a list of waypoints

        # Parameters
        # ----------
        # path : msg_ned
        #     List of waypoints
        # color : string
        #     Desired color in format for matplot (e.g. 'r','y','b',etc.)
        # """
        # for i in range(0, len(path) - 1):
        #     way1 = path[i]
        #     way2 = path[i + 1]
        #     self.ax.plot([way1.n, way2.n], [way1.e, way2.e], [-way1.d, -way2.d], color=color)
        pass

class Animation2D(Animation):
    def __init__(self, mission):
        # Backends:
        # https://stackoverflow.com/questions/30809316/how-to-create-a-plot-in-matplotlib-without-using-pyplot
        super(Animation2D, self).__init__(mission, '2D Animation')
        # nodes is a dictionary from Node to matplotlib Line objects.
        self.nodes = {}
        ax = self.ax
        # ax.cla()
        for obstacle in self.obstacles:
            ax.add_artist(plt.Circle((obstacle.e, obstacle.n), obstacle.r, color='r'))
        # Plot reversed exterior boundaries since we want (N, E) -> (E, N).
        ax.plot(*self.bound_poly.exterior.xy[::-1])
        minN, minE, maxN, maxE = self.bound_poly.bounds
        ax.set_xlim((int(1.1*minE), int(1.1*maxE)))
        ax.set_ylim((int(1.1*minN), int(1.1*maxN)))
        ax.set_aspect('equal')  
        plt.pause(0.000001)

    def add_path(self, waypoints, valid=True):
        """
        Adds colored paths between the list of waypoints received. Or, adds 
        gray paths between the list of nodes received.

        Parameters
        ----------
        waypoints : list of metis.messages.msg_ned
            A list of waypoint messages.
        valid : bool
            Paths are official, not random potential paths (default True).
        """
        pass

    def add_node(self, node):
        """
        Adds gray paths between the list of nodes received.

        Parameters
        ----------
        nodes : list of metis.messages.msg_ned
            The nodes to be connected by a potential random path.
        """
        pass

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