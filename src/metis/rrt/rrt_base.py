# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import logging

import numpy as np


_module_logger = logging.getLogger(__name__)

class Config(object):
    clearance = 5.0
    max_distance = 50.0
    min_radius = 20.0
    max_incline = 0.5
    # max_rel_chi = 15*np.pi/16
    max_rel_chi = np.radians(60)
    iterations = 50
    resolution = 0.5
    scale_height = 1.5
    distance = 15

    def __init__(self, clearance=clearance, max_distance=max_distance, min_radius=min_radius, max_incline=max_incline, max_rel_chi=max_rel_chi, iterations=iterations, resolution=resolution, scale_height=scale_height, distance=distance):
        """
        Parameters
        ----------
        clearance : float, optional
            The minimum distance between the path and all obstacles (default 5.0).
        max_distance : float, optional
            Max distance between each added leaf (default 50.0).
        min_radius : float, optional
            Minimum turn radius of the aircraft for planning fillet paths (default 20).
        max_incline : float, optional
            The maximum incline or decline angle of the planned path (default 0.5).
        max_rel_chi : float, optional
            The maximum difference in the chi angles of path segments/leaves (default 15*pi/16).
        iterations : int, optional
            The amount of leaves that will be added until a successful path is found, or
            how many sets of random points it will add each time until solution is found (default 50).
        resolution : float, optional
            The spacing of points along the path that are checked for collisions. This method was chosen for ease of use
            and could be improved later if wanted. But it should be good enough for our purposes and it runs quickly (default 0.5).
        scale_height : float, optional
            This is a scaling value when finding which leaf is closest to the randomly chosen new point. This scales the
            height in the distance formula so that preference is given to leaves that are closer to the ending altitude (default 1.5).
        distance : float, optional
            In order to fly directly through a primary waypoint, an additional waypoint is added on the opposite side of the primary
            waypoint at a distance of distance. This forces the plane to go through the primary waypoint before beginning a fillet turn (default 15).
        """
        self.clearance = clearance
        self.max_distance = max_distance
        self.min_radius = min_radius
        self.max_incline = max_incline
        self.max_rel_chi = max_rel_chi
        self.iterations = iterations
        self.resolution = resolution
        self.scale_height = scale_height
        self.distance = distance


class Tree(object):
    _logger = _module_logger.getChild('Tree')

    def __init__(self, root=None):
        """
        Creates a tree object and sets root as the root node.

        Parameters
        ----------
        root : Node
            The root Node of the tree.
        """
        super(Tree, self).__init__()
        self.nodes = [root] if root else []

    def __getitem__(self, index):
        return self.nodes[index]

    def __len__(self):
        """
        Returns the number of nodes stored in the tree.

        Returns
        -------
        int
            The number of nodes stored in the tree.
        """
        return len(self.nodes)

    def closest(self, node):
        """
        Finds the node in the tree closest to some other node.

        Parameters
        ----------
        node : Node
            A random node we're trying to find the nearest neighbor of.
        
        Returns
        -------
        Node
            The node already stored in the tree that is closest to the passed 
            in Node.
        """
        dists = [child.distance(node) for child in self.nodes]
        min_idx = np.argmin(dists)
        return self.nodes[min_idx]

    def add(self, node):
        self.nodes.append(node)


class RRT(object):
    """
    An RRT object plans plans flyable paths in the mission environment. 
    It also holds the information concerning the physical boundaries and 
    obstacles in the competition.
    """
    _logger = _module_logger.getChild('RRT')

    # TODO: Just take a mission object instead of obstacles and boundaries?
    def __init__(self, mission, animate=True, config=Config()):
        """The constructor for the RRT class.

        Parameters
        ----------
        obstaclesList : list of metis.messages.msg_ned, optional
            A list of all the obstacles in the mission area
        boundariesList : list of metis.messages.msg_ned, optional
            A list of all the boundary points of the mission area
        animate : boolean
            True if a visual output is wanted, False otherwise
        """
        self._logger.info(mission)
        # np.random.seed(1111) # For Debugging
        self.config = config
        # self.mission = mission

        # Save obstacles and boundaries
        self.obstaclesList = mission.obstacles
        self.boundariesList = mission.boundary_list

        self.animation = Animation2D(mission) if animate else None

        # Boundaries now contained in a Polygon object
        self.bound_poly = mission.boundary_poly

    def find_full_path(self, waypoints, connect=False):
        raise NotImplementedError

    def find_path(self, wpp_start, wpp_end, map_):
        raise NotImplementedError

    def pointsAlongPath(self, start_node, end_node, Del):
        raise NotImplementedError

    def extendTree(self, tree, end_node, segmentLength, map_, pd):
        raise NotImplementedError

    def findMinimumPath(self, tree, end_node):
        raise NotImplementedError

    def smoothPath(self, path, map_):
        raise NotImplementedError
    
def generateRandomNode(self, map_, pd, chi):
    raise NotImplementedError

def collision(self, n, e, d):
    """
    n, e, d, are arrays of floats.
    Collision needs access to the mission boundaries and obstacles.
    """
    raise NotImplementedError