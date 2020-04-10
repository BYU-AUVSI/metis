# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import copy
import logging

import numpy as np

from metis.location import Waypoint, NEDPoint
from .rrt_base import *
from .animation import Animation2D


_module_logger = logging.getLogger(__name__)

dubins_config = Config(
    # clearance=20,
    max_distance = 150.0,
    min_radius = 50.0,
    iterations=3,
)


class DubinsRRT(RRT):
    """
    An RRT object plans plans flyable paths in the mission environment. 
    It also holds the information concerning the physical boundaries and 
    obstacles in the competition.
    """
    _logger = _module_logger.getChild('DubinsRRT')

    def __init__(self, mission, animate=True, config=dubins_config):
        super(DubinsRRT, self).__init__(mission, animate=animate, config=config)
        self.dubin_params = DubinsParameters()

    def find_full_path(self, waypoints, connect=False):
        """
        Finds a path to all of the waypoints passed in. This path accounts for
        obstacles, boundaries, and all other parameters set in __init__.

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

        # Plan a path for each adjacent set of waypoints and append to full_path
        full_path = []
        way1 = waypoints_[0]
        for way2 in waypoints_[1:]:
            full_path += self.find_path(way1, way2)
            way1 = full_path[-1]

        full_path = self.filter_duplicate_waypoints(full_path)
        if self.animation:
            for way1, way2 in zip(full_path[:-1], full_path[1:]):
                ned = self.points_along_path(way1, way2)
                self.animation.add_path(ned, True)
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

        #START NEW TESTING CODE
        # This code attempts to implement the `connect` feature of the straight
        # line paths; namely, placing a node behind each waypoint to ensure it
        # gets flown through.
        # q = (w2.nparray - w1.nparray)/np.linalg.norm(w2.nparray - w1.nparray)

        # The two conditions below would REPLACE the currently active condition
        # above.

        # add_node = w2.nparray + q*config.distance
        # newmsg = msg_ned(add_node.item(0), add_node.item(1), add_node.item(2))

        # if self.flyable_path(obstacles, bound_poly, w1, newmsg, start_chi, chi, config) and connect:
        #     # return w1, w2, msg_ned(add_node.item(0), add_node.item(1), add_node.item(2))
        #     _logger.critical("option 1")
        #     return [w1, w2, newmsg]

        # elif self.flyable_path(obstacles, bound_poly, w1, w2, start_chi, chi, config) and not connect:
        #     _logger.critical("option 2")
        #     return [w1, w2]
        
        #END NEW TESTING CODE
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
        # FIXME: We really don't need chi0 and chi1 passed in, since that 
        # information is already available within the waypoint itself.
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
            _logger.critical('COLLISION')
            return False

        # Check for new leaf now above max relative chi angle
        # if chi0 != None: #If not at the root node
        #     chi0 = wrap2pi(chi0)
        #     chi1 = wrap2pi(chi1)
        #     dchi = delta_chi(chi0, chi1)
        #     # If the difference in headings is more than some amount:
        #     if abs(dchi) > self.config.max_rel_chi:
        #         _logger.debug("Chi difference too large, {} > {}".format(abs(dchi) , self.config.max_rel_chi))
        #         # _logger.debug("initial course = {}, final course = {}".format(chi0, chi1))
        #         return False

        # Check incline here
        incline = np.abs( (end.d-start.d) / end.distance(start) )
        if incline > self.config.max_incline+.01:  #Added fudge factor because of floating point math errors
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
        # Update the dubins parameters first
        params = self.dubin_params.update(start, end, self.config.min_radius)
        
        # Use values from the dubins parameters for the points_along_path functions
        if not params.valid:
            return None

        ned_s = points_along_arc(params.ps, params.r1, params.cs, self.config.min_radius, params.ds, self.config.resolution)
        ned_m = points_along_straight(params.r1, params.r2, self.config.resolution)
        ned_e = points_along_arc(params.r2, params.pe, params.ce, self.config.min_radius, params.de, self.config.resolution)
        ned = np.concatenate((ned_s, ned_m, ned_e), axis=0)

        # import matplotlib.pyplot as plt
        # plt.plot(ned[:,1], ned[:,0])
        # plt.axis('equal')
        # plt.show()

        return ned

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
        # TODO: Find a way to incorporate if a node already has an assigned
        # heading.
        _logger = _module_logger.getChild('extend_tree')
        minE, minN, maxE, maxN = self.mission.boundary_poly.bounds
        
        # Loop until we have a path that is viable
        flyable = False
        while not flyable:
            # Generate a random point
            new_node = generate_random_node(maxN, minN, maxE, minE)

            # *********************************************************************
            # Find the nearest leaf. Preference given to leaves that are at the 
            # correct altitude.
            closest = tree.closest(new_node)
            new_node.d = closest.d

            # Need to find way to get more smooth descents and ascents?? not zig zaggy
            # chi = np.arctan2((new_node.e - closest.e), (new_node.n - closest.n))
            chi = heading(closest, new_node)

            # *********************************************************************
            # Calculate the new node location

            # If the chosen leaf is at the ending waypoint altitude
            if(closest.d == end.d):
                # A new leaf only extends a maximum distance from a previous leaf
                connection = new_node.ned - closest.ned
                L = min(np.linalg.norm(connection), self.config.max_distance)
                point = closest.ned + L*(connection / np.linalg.norm(connection))
                new_node = Node(point.item(0), point.item(1), point.item(2), chi, closest.cost + L, closest, False)
            
            # This case is for when the nearest leaf isn't yet at the correct altitude for the ending waypoint
            else:
                hyp = np.sqrt((new_node.n-closest.n)**2 + (new_node.e-closest.e)**2)
                lessInclineWhilePlanning = 0.3
                if new_node.d > end.d: # Climb
                    downP = closest.d - hyp * self.config.max_incline * lessInclineWhilePlanning
                else: # Descend
                    downP = closest.d + hyp * self.config.max_incline * lessInclineWhilePlanning
                northP = new_node.n
                eastP = new_node.e
                q = np.array([northP - closest.n, eastP - closest.e, downP - closest.d])
                L = np.linalg.norm(q)
                L = min(L, self.config.max_distance)
                tmp = new_node.ned - closest.ned
                point = closest.ned + L*(tmp/np.linalg.norm(tmp))
                new_node = Node(point.item(0), point.item(1), point.item(2), chi, closest.cost + L, closest, False)
                
                # If we were descending and overshot (or were ascending and overshot), set to final altitude.
                if (new_node.d > closest.d and new_node.d < end.d) or (new_node.d < closest.d and new_node.d > end.d):
                    new_node.d = end.d

            _logger.debug('Random node generated at [{}, {}, {}]'.format(new_node.n, new_node.e, new_node.d))
            
            # Check for collision. If we have a flylable path, break out of the loop!
            flyable = self.flyable_path(closest, new_node, closest.chi, new_node.chi)

        # Add this new node to the tree of viable paths.
        tree.add(new_node)
        self.animation.add_path(self.points_along_path(closest, new_node)) if self.animation else None

        # Check to see if the new node can connect to the end node.
        chi = heading(new_node, end)
        end.chi = chi

        # Return the extended tree with the flag of a successful path to ending node
        if self.flyable_path(new_node, end, new_node.chi, end.chi):
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


class DubinsParameters:
    def __init__(self):
        self.p_s = np.inf*np.ones((3,1))  # the start position in re^3
        # self.chi_s = np.inf  # the start course angle
        self.p_e = np.inf*np.ones((3,1))  # the end position in re^3
        # self.chi_e = np.inf  # the end course angle
        # self.radius = np.inf  # turn radius
        # self.length = np.inf  # length of the Dubins path
        self.center_s = np.inf*np.ones((3,1))  # center of the start circle
        self.dir_s = np.inf  # direction of the start circle
        self.center_e = np.inf*np.ones((3,1))  # center of the end circle
        self.dir_e = np.inf  # direction of the end circle
        self.r1 = np.inf*np.ones((3,1))  # vector in re^3 defining half plane H1
        self.r2 = np.inf*np.ones((3,1))  # vector in re^3 defining position of half plane H2
        self.r3 = np.inf*np.ones((3,1))  # vector in re^3 defining position of half plane H3
        self.n1 = np.inf*np.ones((3,1))  # unit vector in re^3 along straight line path
        self.n3 = np.inf*np.ones((3,1))  # unit vector defining direction of half plane H3

    # def update(self, ps, chis, pe, chie, R):
    def update(self, ps, pe, R):
        """
        Update the dubins parameters.

        Parameters
        ----------
        ps : metis.location.Waypoint
            A Waypoint object defining the starting point and heading.
        pe : metis.location.Waypoint
            A Waypoint object defining the ending point and heading.
        R : float
            The turn radius to be used for each end of the Dubins path.
        """
        q0 = np.array([ps.n-pe.n, ps.e-pe.e, ps.d-pe.d])
        ell = np.linalg.norm(q0)
        if ell < 2 * R:
            print('Error in Dubins Parameters: The distance between nodes must be larger than 2R ({} !> {}).'.format(ell, 2*R))
            _module_logger.warning('Error in Dubins Parameters')
            return DubinsResult(valid=False)
        else:
            # self.p_s = ps
            chis = ps.chi
            # self.p_e = pe
            chie = pe.chi
            # self.radius = R

            # self.p_s = ps
            # self.chi_s = chis
            # self.p_e = pe
            # self.chi_e = chie
            # self.radius = R

            ps = np.array([[ps.n, ps.e, ps.d]]).T
            pe = np.array([[pe.n, pe.e, pe.d]]).T

            c_rs = ps + R * rotz(np.pi/2.).dot(np.array([[np.cos(chis), np.sin(chis), 0.]]).T)
            c_ls = ps + R * rotz(-np.pi/2.).dot(np.array([[np.cos(chis), np.sin(chis), 0.]]).T)
            c_re = pe + R * rotz(np.pi/2.).dot(np.array([[np.cos(chie), np.sin(chie), 0.]]).T)
            c_le = pe + R * rotz(-np.pi/2.).dot(np.array([[np.cos(chie), np.sin(chie), 0.]]).T)

            # print(ps, c_rs, chis)
            # print(ps, c_ls, chis)
            # print(pe, c_re, chie)
            # print(pe, c_le, chie)
            # import sys
            # sys.exit()

            theta = np.arctan2(c_re.item(1) - c_rs.item(1), c_re.item(0) - c_rs.item(0))
            L1 = np.linalg.norm(c_rs - c_re) + R*mod(2.*np.pi + mod(theta - np.pi/2.) - mod(chis - np.pi/2.)) + R * mod(2.*np.pi + mod(chie - np.pi/2.) - mod(theta - np.pi/2.))

            theta = np.arctan2(c_le.item(1) - c_rs.item(1), c_le.item(0) - c_rs.item(0))
            ell = np.linalg.norm(c_le - c_rs)
            arg = np.clip(2.*R / ell, -1., 1.)
            theta2 = theta - np.pi/2. + np.arcsin(arg)
            L2 = np.sqrt(ell**2. - 4*R**2.) + R*mod(2.*np.pi + mod(theta - theta2) - mod(chis - np.pi/2.)) + R*mod(2.*np.pi + mod(theta2 + np.pi) - mod(chie + np.pi/2.))

            theta = np.arctan2(c_re.item(1) - c_ls.item(1), c_re.item(0) - c_ls.item(0))
            ell = np.linalg.norm(c_re - c_ls)
            arg = np.clip(2.*R / ell, -1., 1.)
            theta2 = np.arccos(arg)
            L3 = np.sqrt(ell**2. - 4*R**2.) + R*mod(2*np.pi + mod(chis + np.pi/2.) - mod(theta + theta2)) + R*mod(2.*np.pi + mod(chie - np.pi/2.) - mod(theta + theta2 - np.pi))

            theta = np.arctan2(c_le.item(1) - c_ls.item(1), c_le.item(0) - c_ls.item(0))
            L4 = np.linalg.norm(c_ls - c_le) + R*mod(2*np.pi - mod(theta + np.pi/2.) + (chis + np.pi/2.)) + R * mod(2*np.pi - mod(chie + np.pi/2.) + mod(theta + np.pi/2.))

            lengths = [L1, L2, L3, L4] 
            L = min(lengths)
            # self.length = lengths[int(L)]
            # self.length = L

            e1 = np.array([[1, 0, 0]]).T

            if L == L1:
                self.center_s = c_rs
                self.dir_s = 1
                self.center_e = c_re
                self.dir_e = 1
                self.n1 = (self.center_e - self.center_s) / np.linalg.norm(self.center_e - self.center_s)
                self.r1 = self.center_s + R*rotz(-np.pi/2.).dot(self.n1)
                self.r2 = self.center_e + R*rotz(-np.pi/2.).dot(self.n1)
            
            if L == L2:
                self.center_s = c_rs
                self.dir_s = 1
                self.center_e = c_le
                self.dir_e = -1
                ell = np.linalg.norm(self.center_e - self.center_s)
                theta = np.arctan2(self.center_e.item(1) - self.center_s.item(1), self.center_e.item(0) - self.center_s.item(0))
                arg = np.clip(2.*R/ell, -1., 1.)
                theta2 = theta - np.pi/2. + np.arcsin(arg)
                self.n1 = rotz(theta2 + np.pi/2.).dot(e1)
                self.r1 = self.center_s + R*rotz(theta2).dot(e1)
                self.r2 = self.center_e + R*rotz(theta2 + np.pi).dot(e1)

            if L == L3:
                self.center_s = c_ls
                self.dir_s = -1
                self.center_e = c_re
                self.dir_e = 1
                ell = np.linalg.norm(self.center_e - self.center_s)
                theta = np.arctan2(self.center_e.item(1) - self.center_s.item(1), self.center_e.item(0) - self.center_s.item(0))
                arg = np.clip(2.*R/ell, -1., 1.)
                theta2 = np.arccos(arg)
                self.n1 = rotz(theta + theta2 - np.pi/2.).dot(e1)
                self.r1 = self.center_s + R*rotz(theta + theta2).dot(e1)
                self.r2 = self.center_e + R*rotz(theta + theta2 - np.pi).dot(e1)

            if L == L4:
                self.center_s = c_ls
                self.dir_s = -1
                self.center_e = c_le
                self.dir_e = -1
                self.n1 = (self.center_e - self.center_s) / np.linalg.norm(self.center_e - self.center_s)
                self.r1 = self.center_s + R*rotz(np.pi/2.).dot(self.n1)
                self.r2 = self.center_e + R*rotz(np.pi/2.).dot(self.n1)

            self.r3 = pe
            self.n3 = rotz(chie).dot(e1)
            
            return DubinsResult(ps, chis, self.center_s, self.r1, self.dir_s, pe, chie, self.center_e, self.r2, self.dir_e, True)
        # self.center_s = Waypoint(self.center_s.item(0), self.center_s.item(1), self.center_s.item(2))
        # self.center_e = Waypoint(self.center_e.item(0), self.center_e.item(1), self.center_e.item(2))
        # self.r1 = Waypoint(self.r1.item(0), self.r1.item(1), self.r1.item(2))
        # self.r2 = Waypoint(self.r2.item(0), self.r2.item(1), self.r2.item(2))
        # self.r3 = Waypoint(self.r3.item(0), self.r3.item(1), self.r3.item(2))


class DubinsResult(object):
    """
    A dataclass representing the results of a DubinsParameters `update` call.

    Attributes
    ----------
    ps : metis.location.Waypoint
        A waypoint for the starting position in the path.
    cs : metis.location.NEDPoint
        A NED point representing the center of the first arc traced out in the path.
    r1 : metis.location.Waypoint
        A waypoint for the position in the path transitioning from the first
        arc to the straight line segment.
    ds : int, (-1 or 1)
        The sense of rotation around the first arc, (1 = clockwise, -1 = counterclockwise).
    pe : metis.location.Waypoint
        A waypoint for the ending position in the path.
    ce : metis.location.NEDPoint
        A NED point representing the center of the second arc traced out in the path.
    r2 : metis.location.Waypoint
        A waypoint for the position in the path transitioning from the  
        straight line segment to the second arc.
    de : int, (-1 or 1)
        The sense of rotation around the second arc, (1 = clockwise, -1 = counterclockwise).
    valid : bool
        A flag showing whether the dubins parameters were calculated successfully.
    """
    def __init__(self, ps=None, chis=None, cs=None, r1=None, ds=None, pe=None, chie=None, ce=None, r2=None, de=None, valid=False):
        """
        A data object holding the values that define a Dubins path.

        Parameters
        ----------
        ps : np.ndarray
            A NumPy array with shape (3, 1) representing the starting NED position.
        chis : float
            The heading of the first waypoint.
        cs : np.ndarray
            A NumPy array with shape (3, 1) containing the starting arc's 
            center NED position.
        r1 : np.ndarray
            A NumPy array with shape (3, 1) representing the NED point along 
            the first arc the aircraft transitions to a straight line path.
        ds : int, -1 or 1
            Defines the sense of rotation for the first arc 
            (1 = clockwise, -1 = counterclockwise).
        pe : np.ndarray
            A NumPy array with shape (3, 1) representing the ending NED position.
        chie : float
            The heading of the second waypoint.
        ce : np.ndarray
            A NumPy array with shape (3, 1) containing the ending arc's 
            center NED position.firstcond arc 
            (1 = clockwise, -1 = counterclockwise).
        r2 : np.ndarray
            A NumPy array with shape (3, 1) representing the NED point along 
            the second arc the aircraft transitions from the straight line path.
        de : int, -1 or 1
            Defines the sense of rotation for the second arc 
            (1 = clockwise, -1 = counterclockwise).
        """
        self.valid = valid
        if valid:
            self.ps = Waypoint(ps.item(0), ps.item(1), ps.item(2), chis)
            self.cs = NEDPoint(cs.item(0), cs.item(1), cs.item(2))
            self.r1 = Waypoint(r1.item(0), r1.item(1), r1.item(2))
            self.ds = ds

            self.pe = Waypoint(pe.item(0), pe.item(1), pe.item(2), chie)
            self.ce = NEDPoint(ce.item(0), ce.item(1), ce.item(2))
            self.r2 = Waypoint(r2.item(0), r2.item(1), r2.item(2))
            self.de = de


def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0.],
                    [np.sin(theta), np.cos(theta), 0.],
                    [0., 0., 1.]])

def mod(x):
    # make x between 0 and 2*pi
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x
