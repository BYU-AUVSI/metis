# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import copy
import logging

import numpy as np

from metis.location import Waypoint, convert_point
from .rrt_base import *
from .animation import Animation2D


_module_logger = logging.getLogger(__name__)

fillet_config = Config(
    clearance=15,
    min_radius=np.radians(45),
    iterations=3,
)

class FilletRRT(RRT):
    """
    An RRT object plans plans flyable paths in the mission environment. 
    It also holds the information concerning the physical boundaries and 
    obstacles in the competition.
    """
    _logger = _module_logger.getChild('StraightRRT')

    def __init__(self, mission, animate=False, config=fillet_config):
        super(FilletRRT, self).__init__(mission, animate=animate, config=config)

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

    def flyable_path(self, start, end, chi0, chi1, third=None):
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
        third : metis.rrt.rrt_base.Node, optional
            A third node after the two being planned for, for fillet paths.

        Returns
        -------
        boolean
            Returns True if a flyable path, False if not.
        """
        _logger = _module_logger.getChild('flyable_path')

        #check for obstacles and boundaries
        ned = self.points_along_path(start, end, third)

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
        incline = np.abs( (end.d-start.d) / end.distance(start) )
        if incline > self.config.max_incline+.01:  #Added fudge factor because of floating point math errors
            _logger.debug("Incline too steep.")
            return False

        return True

    def points_along_path(self, start, end, future=None):
        """
        Creates a stepped range of values from the starting node to the ending node
        with difference in step size guaranteed to be less than `self.config.resolution`.

        Parameters
        ----------
        start : metis.rrt.rrt_base.Node
            The starting node to create a range from.
        end : metis.rrt.rrt_base.Node
            The ending node to create a range to.
        future : metis.rrt.rrt_base.Node, optional
            The node coming after the end node, for fillet paths.
        
        Returns
        -------
        ned : np.ndarray
            An m x 3 numpy array, where each row is an array of north, east, 
            down points.
        """
        if not future:
            return self.points_along_straight(start, end)
        else:
            R = self.config.min_radius
            stepSize = self.config.resolution
            N = np.array([])
            E = np.array([])
            D = np.array([])

            start_node = np.array([[start.n], [start.e], [start.d]])
            mid_node = np.array([[end.n], [end.e], [end.d]])
            last_node = np.array([[future.n], [future.e], [future.d]])

            q_pre = (mid_node - start_node)/np.linalg.norm(mid_node - start_node)
            q_next = (last_node - mid_node)/np.linalg.norm(last_node - mid_node)

            np.seterr(all='raise')

            if abs(np.matmul(-q_pre.T,q_next)) >= 1:
                if np.matmul(-q_pre.T,q_next) > 0:
                    theta = 0
                elif np.matmul(-q_pre.T,q_next) < 0:
                    theta = -np.pi
            else:
                theta = np.arccos(np.matmul(-q_pre.T,q_next)).item(0)

            if np.linalg.norm(q_pre - q_next) > 0.000001 and theta > 0 and abs(theta) < np.pi:
                C = mid_node - (R/np.sin(theta/2))*(q_pre - q_next)/np.linalg.norm(q_pre - q_next)
                r1 = mid_node - (R/np.tan(theta/2))*q_pre
                r2 = mid_node + (R/np.tan(theta/2))*q_next
            else:
                C = mid_node
                r1 = C
                r2 = C
            

            r1v = r1 - C
            r2v = r2 - C
            if np.linalg.norm(r1v - r2v) > 0:
                rot_theta = np.arccos(np.matmul(r1v.T/np.linalg.norm(r1v.T),r2v/np.linalg.norm(r2v))).item(0)
            else:
                rot_theta = 0

            rot_direction = -np.sign(np.cross(r1v[:,0], r2v[:,0]).item(2))

            start2hp = r1v - start_node
            start2mid = mid_node - start_node

            end2hp = r2v - last_node
            end2mid = mid_node - last_node

            dir1 = np.matmul(start2hp.T, start2mid)
            dir2 = np.matmul(end2hp.T, end2mid)

            forwards = True
            if dir1 < 0 or dir2 < 0:
                forwards = False


            # Checking here to see if the halfplanes or start and end nodes are closer
            if forwards and (np.linalg.norm(r1 - mid_node) < np.linalg.norm(start_node - mid_node)) and (np.linalg.norm(r2 - mid_node) < np.linalg.norm(last_node - mid_node)):
                current_position = np.array([[start.n], [start.e], [start.d]])
                pre_position = current_position

                while (np.linalg.norm(pre_position - r1) >= stepSize):
                    N = np.append(N, current_position.item(0))
                    E = np.append(E, current_position.item(1))
                    D = np.append(D, current_position.item(2))

                    pre_position = current_position

                    current_position = current_position + q_pre*stepSize

                    #   print("Part 1")
                    # print(current_position)

                
                current_position = r1
                ang_inc = float(stepSize)/(2*R)
                angle = 0
                while (rot_theta is not np.nan) and (angle <= rot_theta):
                    N = np.append(N, current_position.item(0))
                    E = np.append(E, current_position.item(1))
                    D = np.append(D, current_position.item(2))

                    angle = angle + ang_inc

                    Rot = np.array([[np.cos(rot_direction*ang_inc), np.sin(rot_direction*ang_inc), 0],
                                [-np.sin(rot_direction*ang_inc), np.cos(rot_direction*ang_inc), 0],
                                [0, 0, 1]])
                    
                    current_position = np.matmul(Rot,current_position - C) + C

                    # print("Part 2")
                    # print(current_position)
                
                current_position = r2
                pre_position = current_position
                while(np.linalg.norm(pre_position - last_node) >= stepSize):
                    N = np.append(N, current_position.item(0))
                    E = np.append(E, current_position.item(1))
                    D = np.append(D, current_position.item(2))

                    pre_position = current_position
                    current_position = current_position + q_next*stepSize

                    # print("Part 3")
                    #  print(current_position)
                
                current_position = last_node
                N = np.append(N, current_position.item(0))
                E = np.append(E, current_position.item(1))
                D = np.append(D, current_position.item(2))
            

                # if True: # This will plot each individual path when this function is called. Good for debugging but you don't want it accidentially running when you're trying to actually do a flight. Hence the hardcoded false option. For debuggin, switch to true
                #     fig = plt.figure()
                #     ax = fig.add_subplot(111)
                #     ax.scatter(E, N)
                #     ax.scatter(r1.item(1), r1.item(0),c='r')
                #     ax.scatter(r2.item(1), r2.item(0),c='r')
                #     ax.scatter(C.item(1), C.item(0),c='r')
                #     ax.axis('equal')
                #     plt.show()

                ned = np.stack([N, E, D], axis=1)
                return ned

            else:
                start_node = Node(start_node.item(0), start_node.item(1), start_node.item(2))
                mid_node = Node(mid_node.item(0), mid_node.item(1), mid_node.item(2))
                last_node = Node(last_node.item(0), last_node.item(1), last_node.item(2))
                ned1 = self.points_along_path(start_node, mid_node)
                ned2 = self.points_along_path(mid_node, last_node)

                return np.concatenate((ned1, ned2), axis=0)

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

            if closest.parent is not None:
                node_1 = closest.parent
                node_2 = closest
                node_3 = new_node
            else:
                node_1 = closest
                node_2 = new_node
                node_3 = None
            
            # Check for collision. If we have a flylable path, break out of the loop!
            flyable = self.flyable_path(node_1, node_2, closest.chi, new_node.chi, node_3)

        # Add this new node to the tree of viable paths.
        tree.add(new_node)
        self.animation.add_path(self.points_along_path(closest, new_node)) if self.animation else None

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