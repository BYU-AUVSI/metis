# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import copy
import logging

import numpy as np

from metis.location import Waypoint, convert_point
from .rrt_base import *
from .rrt_straight import *
from .animation import Animation2D


_module_logger = logging.getLogger(__name__)

straight_config = Config(
    clearance=15,
    # 120 good for objective and payload
    # max_rel_chi=np.radians(165),
    # max_rel_chi=15.*np.pi/16.,
    max_rel_chi=np.radians(80),
    iterations=3,
)


class StraightRRTStar(StraightRRT):
    """
    An RRT object plans plans flyable paths in the mission environment.
    It also holds the information concerning the physical boundaries and
    obstacles in the competition.

    Extends metis.rrt.rrt_base.RRT.
    """
    _logger = _module_logger.getChild('StraightRRT')

    def __init__(self, mission, animate=True, config=straight_config):
        super(StraightRRTStar, self).__init__(mission, animate=animate, config=config)

    def find_full_path(self, waypoints, connect=False):
        """
        Finds a path to all of the waypoints passed in. This path accounts for
        obstacles, boundaries, and all other parameters set in __init__.

        Overrides RRT.find_full_path.

        Parameters
        ----------
        waypoints : list of metis.messages.msg_ned
            A list of waypoints.
        connect : bool, optional
            If true, the path will be generated such that an additional waypoint is created after every primary waypoint to
            force the plane to go through the primary waypoint before beginning a turn

        Returns
        -------
        full_path : list of metis.messages.msg_ned
            The full list of waypoints which outlines a safe path to follow in order to reach all of the waypoints
            passed in.
        """
        # Avoid modifying the original object
        waypoints_ = self.filter_invalid_waypoints(copy.deepcopy(waypoints))

        if self.animation:
            for p in waypoints_:
                self.animation.add_waypoint(p.n, p.e, p.d)


        # Plan a path for each adjacent set of waypoints and append to full_path
        full_path = []
        way1 = waypoints_[0]
        for way2 in waypoints_[1:]:
            full_path += self.find_path(way1, way2)
            way1 = full_path[-1]
            if self.animation:
                self.animation.new_color()

        full_path = self.filter_duplicate_waypoints(full_path)
        if self.animation:
            for way1, way2 in zip(full_path[:-1], full_path[1:]):
                ned = self.points_along_path(way1, way2)
                self.animation.add_path(ned, True)
            for way in full_path:
                self.animation.add_waypoint(way.n, way.e, way.d)
        return full_path

    def find_path(self, start, end):
        """
        Finds a path between two waypoints passed in, accounting for obstacles
        and boundaries.

        Parameters
        ----------
        start : metis.core.Waypoint
            The starting waypoint
        end : metis.core.Waypoint
            The ending waypoint. Super creative names, I know.
        connect : boolean, optional
            If True, the path will be generated such that an additional waypoint
            is created after every primary waypoint to force the plane to go
            through the primary waypoint before beginning a turn (default False).

        Returns
        -------
        smoothedPath : msg_ned
            The list of waypoints which outlines a safe path to follow in order to reach the two waypoints passed in.
        """
        _logger = _module_logger.getChild("find_path")

        start_node = convert_point(start, Node)
        end_node = convert_point(end, Node)
        tree = Tree(root=start_node)
        
        # check for if solution at the beginning
        chi = heading(start_node, end_node)
        if self.flyable_path(start_node, end_node, start_node.chi, chi):
            self.animation.add_path(self.points_along_path(start_node, end_node)) if self.animation else None
            return [convert_point(start_node, Waypoint), convert_point(end_node, Waypoint)]  # Returns the two waypoints as the succesful path
        else:
            solutions = 0
            while solutions < self.config.iterations:
                tree, flag = self.extend_tree(tree, end_node)
                if flag:
                    solutions += 1

        path = self.find_minimum_path(tree, end_node)
        path = self.smooth_path(path)
        path = self.normalize2waypoints(path)
        return path

        # # Smooth all paths and save the best
        # bestPath = []
        # bestCost = np.inf
        # for path in connectedPaths:
        #     smoothedPath, cost = smooth_path(path, start_chi, obstacles, bound_poly, config=config)

        #     if connect:
        #         print("Connect")

        #         last_node = np.array([[smoothedPath[-1].n],[smoothedPath[-1].e],[smoothedPath[-1].d]])
        #         prep_node = np.array([[smoothedPath[-2].n],[smoothedPath[-2].e],[smoothedPath[-2].d]])

        #         q = (last_node - prep_node)/np.linalg.norm(last_node - prep_node)

        #         add_node = last_node + q*config.distance

        #         if self.flyable_path(obstacles, bound_poly, smoothedPath[-1], msg_ned(add_node.item(0), add_node.item(1), add_node.item(2)), 0, 0, config):
        #             smoothedPath.append(msg_ned(add_node.item(0), add_node.item(1), add_node.item(2)))

        #     if cost <  bestCost:
        #         bestPath = smoothedPath
        #         bestCost = cost

        # # bestPathNew = []
        # # bestPathNew.append(bestPath[0])
        # # bestPathNew.append(node2)
        # # bestPathNew.append(bestPath[1:])
        # # bestPathNew = bestPathNew.append(nodel)
        # # bestPath = bestPathNew
        # # elif len(bestPath) > 2:
        # #     pass

        # # elif len(bestPath) == 2:
        # #     pass
        # # debug = False
        # # if debug:
        # #     n = np.array([item.n for item in bestPath])
        # #     e = np.array([item.e for item in bestPath])
        # #     plt.plot(e, n, 'rx-')
        # #     for i in range(len(n)):
        # #         plt.text(e[i], n[i], str(i))
        # #     plt.axis('equal')
        # #     plt.show()

        # return bestPath
        # if self.animation:
        #     # plot the last two successful waypoints as a chosen path
        #     pass
        # raise NotImplementedError

    def flyable_path(self, start, end, chi0, chi1):
        """
        Checks if flying between two points is  possible. It checks for
        collisions, chi angle, and incline.

        Parameters
        ----------
        start : metis.rrt.rrt_base.Node
            The starting node
        end : metis.rrt.rrt_base.Node
            The ending node
        chi0 : double
            The heading of the node being added to (in radians).
        chi1 : double
            The heading of the added node (in radians).

        Returns
        -------
        boolean
            Returns True if a flyable path, False if not.
        """
        _logger = _module_logger.getChild('flyable_path')

        #check for obstacles and boundaries
        ned = self.points_along_path(start, end)

        if collision(ned, self.mission.boundary_poly, self.mission.obstacles, self.config.clearance):
            return False

        # Check for new leaf now above max relative chi angle
        if chi0 != None: #If not at the root node
            chi0 = wrap2pi(chi0)
            chi1 = wrap2pi(chi1)
            dchi = delta_chi(chi0, chi1)
            # If the difference in headings is more than some amount:
            if abs(dchi) > self.config.max_rel_chi:
                _logger.debug("Chi difference too large, {} > {}".format(abs(dchi) , self.config.max_rel_chi))
                _logger.debug("initial course = {}, final course = {}".format(chi0, chi1))
                return False

        # Check incline here
        # incline = np.abs( (end.d-start.d) / end.distance(start) )
        pitch_angle = pitch(start, end)
        # if incline > self.config.max_incline+.01:  #Added fudge factor because of floating point math errors
        if pitch_angle > np.radians(25):
            _logger.debug("Incline too steep.")
            return False

        return True

    def points_along_path(self, start, end):
        """
        Creates a stepped range of values from the starting node to the ending node
        with difference in step size guaranteed to be less than `self.config.resolution`.

        Parameters
        ----------
        start : metis.rrt.rrt_base.Node
            The starting node to create a range from.
        end : metis.rrt.rrt_base.Node
            The ending node to create a range to.

        Returns
        -------
        ned : np.ndarray
            An m x 3 numpy array, where each row is an array of north, east,
            down points.
        """
        return points_along_straight(start, end, self.config.resolution)

    def extend_tree(self, tree, end):
        """
        mission is already accessible via self
        segment_length is available via self.config
        """
        """
        Extends the passed-in tree. It will continually attempt to add a leaf
        until it finds a successful one. This is the basic RRT algorithm.

        Parameters
        ----------
        tree : float
            An Nx7 array of N leaves in this format: N, E, D, cost, parentIndex, connectsToGoalFlag, chi
        end : msg_ned
            The ending waypoint.

        Returns
        -------
        tree : metis.rrt.Tree
            A Tree object containing the extended tree.
        flag : bool
            Returns True if a path to the end node was found, False if not.
        """
        _logger = _module_logger.getChild('extend_tree')
        minE, minN, maxE, maxN = self.mission.boundary_poly.bounds

        # Loop until we have a path that is viable
        flyable = False
        while not flyable:
            # Generate a random point
            _logger.debug('Generating random point')
            new_node = generate_random_node(maxN, minN, maxE, minE)

            # *********************************************************************
            # Find the nearest leaf. Preference given to leaves that are at the
            # correct altitude.
            neighbors = tree.closest(new_node, 100)
            # _logger.critical(neighbors)
            dists = [node.distance(new_node, d2=True) for node in neighbors]

            costs = []
            for node in neighbors:
                new_node.parent = node
                costs.append(new_node.cost)

            closest = neighbors[np.argmin(costs)]
            new_node.d = closest.d

            # Need to find way to get more smooth descents and ascents?? not zig zaggy
            # chi = np.arctan2((new_node.e - closest.e), (new_node.n - closest.n))
            chi = heading(closest, new_node)

            # *********************************************************************
            # Calculate the new node location

            # A new leaf only extends a maximum distance from a previous leaf
            connection = new_node.ned - closest.ned
            L = min(np.linalg.norm(connection), self.config.max_distance)
            point = closest.ned + L*(connection / np.linalg.norm(connection))
            new_node = Node(point.item(0), point.item(1), point.item(2), chi, closest.cost + L, closest, False)

            # Check for collision. If we have a flylable path, break out of the loop!
            flyable = self.flyable_path(closest, new_node, closest.chi, new_node.chi)

        dist = new_node.distance(closest, d2=True)
        max_pitch = np.radians(20)
        delta_h = dist*np.tan(max_pitch)
        if new_node.h < end.h: # We're under altitude, climb
            new_node.h = min(new_node.h + delta_h, end.h)
        elif new_node.h > end.h: # We're over altitude, descend
            new_node.h = max(new_node.h - delta_h, end.h)
        else: # We must be right at altitude
            new_node.d = end.d

        # Add this new node to the tree of viable paths.
        tree.add(new_node)
        self.animation.add_path(self.points_along_path(closest, new_node)) if self.animation else None

        winner = None
        for node in neighbors:
            if node.parent is None or node is new_node.parent:
                continue
            old_parent = node.parent
            old_cost = node.cost

            node.parent = new_node
            new_cost = node.cost
            chi = heading(new_node, node)

            if self.flyable_path(new_node, node, new_node.chi, chi) and new_cost < old_cost:
                winner = node
            else:
                node.parent = old_parent

        if winner:
            self.animation.add_path(self.points_along_path(winner, new_node)) if self.animation else None

        # Check to see if the new node can connect to the end node.
        chi = heading(new_node, end)

        # Return the extended tree with the flag of a successful path to ending node
        if self.flyable_path(new_node, end, new_node.chi, chi):
            end_n = Node(end.n, end.e, end.d, chi, new_node.cost + end.distance(new_node), new_node, True)
            tree.add(end_n)
            self.animation.add_path(self.points_along_path(new_node, end_n)) if self.animation else None
            return tree, True
        else:
            return tree, False

    def smooth_path(self, path):
        """
        mission is already accessible via self
        """
        # _logger = _module_logger.getChild('smooth_path')
        smoothed_path = [path[0]]
        for j in range(1, len(path) - 1):
            prev_node = smoothed_path[-1]
            considered = path[j]
            next_node = path[j + 1]
            points = self.points_along_path(prev_node, next_node)
            if collision(points, self.mission.boundary_poly, self.mission.obstacles, self.config.clearance):
                smoothed_path.append(considered)
        smoothed_path.append(path[-1])
        # Update costs?
        return smoothed_path
