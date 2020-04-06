# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

"""
Maybe we can have an expanding circle that figures out the best direction
to approach from? Then, as it expands, it assigns weights or eliminates bad
headings that shouldn't be used as an approach.
"""

from __future__ import print_function

import copy
import logging

import numpy as np

from metis.messages import msg_ned
from metis.tools import will_collide
from .rrt_base import Config, Tree
from .animation import Animation2D

# import cProfile

# TODO: A buffer for incline when planning, that way it can smooth
# Find all possible paths, smooth all, then pick best
# Could do an improved smoother where is doesn't just check adjacent nodes. But can't be a factorial check.

_module_logger = logging.getLogger(__name__)

class Node(object):
    _logger = _module_logger.getChild('Node')

    def __init__(self, n, e, d, cost=0., parent=None, connects=False, chi=0.):
        """
        Parameters
        ----------
        n : float
            North position.
        e : float
            East position.
        d : float
            Down position.
        cost : float, optional
            Cost of the node (default 0.0).
        parent : reference to metis.rrt.Node, optional
            A reference to the parent node of the object (default None).
        connects : bool, optional
            Whether or not this node connects to the goal; true if it connects,
            false otherwise (default False).
        chi : float, optional
            Heading in radians (default 0).
        """
        super(Node, self).__init__()
        self.ned = msg_ned(n, e, d)
        self.cost = cost
        self.parent = parent
        self.connects = connects
        self.chi = chi

    def __eq__(self, other):
        """
        Equality overridden such that as long as the nodes are in exactly the
        same geographic location, they are considered to be the same node.
        """
        return self.ned == other.ned

    @property
    def n(self):
        return self.ned.n
    
    @n.setter
    def n(self, value):
        self.ned.n = value

    @property
    def e(self):
        return self.ned.e
    
    @e.setter
    def e(self, value):
        self.ned.e = value

    @property
    def d(self):
        return self.ned.d
    
    @d.setter
    def d(self, value):
        self.ned.d = value

    def distance(self, other):
        return np.linalg.norm((other.ned - self.ned).nparray)


class RRT():
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
        config : metis.rrt.Config
            A configuration object with custom RRT parameters.

        Returns
        -------
        full_path : list of metis.messages.msg_ned
            The full list of waypoints which outlines a safe path to follow in order to reach all of the waypoints
            passed in.
        """
        self._logger.info("Finding full path for {} waypoints with connect={}".format(len(waypoints), connect))
        
        config = self.config

        # Avoid modifying the original object
        waypoints_ = copy.deepcopy(waypoints)

        # Remove all waypoints that are in obstacles or otherwise infeasable,
        # excluding the start and end points.
        for waypoint in waypoints_[1:-1]:
            if will_collide(self.obstaclesList,self.bound_poly,np.array([waypoint.n]), np.array([waypoint.e]), np.array([waypoint.d]), config.clearance):
                self._logger.warning("Waypoint is out of bounds or on an obstacle: ignoring.")
                waypoints_.remove(waypoint)
        
        full_path = []
        chi = None

        # Plan a path for each adjacent set of waypoints and append to full_path
        way1 = waypoints_[0]
        for way2 in waypoints_[1:]:
            # find_path returns a set of intermediary waypoints from way1 to way2
            full_path += find_path(way1, way2, self.obstaclesList, self.bound_poly, chi, connect=connect, config=config, animation=self.animation)
            self._logger.debug("Individual path found.")
            chi = heading(full_path[-2], full_path[-1])
            way1 = full_path[-1]
            if hasattr(self, 'animation'):
                # plot the last two successful waypoints as a chosen path
                pass

        # Since the origin of each leg was the destination of the previous
        # leg, we need to remove the repetitive nodes.
        full_path = [elem for i, elem in enumerate(full_path) if i == 0 or full_path[i-1] != elem]
        return full_path

    # def findPath(self, waypoint1, waypoint2, start_chi=8888, connect=False):


    #         # #Add node between start and first node to force plane to go through the desired node
    #         # start_node = np.array([[smoothedPath[0].n],[smoothedPath[0].e],[smoothedPath[0].d]])
    #         # next_node = np.array([[smoothedPath[1].n],[smoothedPath[1].e],[smoothedPath[1].d]])
    #         # q = (next_node - start_node)/np.linalg.norm(next_node - start_node)
    #         # if np.linalg.norm(next_node - start_node) < self.config.distance:
    #         #     spacing = np.linalg.norm(next_node - start_node) / 2.0
    #         # else:
    #         #     spacing = self.config.distance

    #         # insert_node = start_node + spacing * q

    #         # smoothedPath.insert(1,msg_ned(insert_node.item(0), insert_node.item(1), insert_node.item(2)))
        
    #     # if len(bestPath) > 3:
    #     #     node1 = bestPath[0]
    #     #     node1v = np.array([[node1.n], [node1.e], [node1.d]])
    #     #     node2 = bestPath[1]
    #     #     node2v = np.array([[node2.n], [node2.e], [node2.d]])
    #     #     node_nextlast = bestPath[-2]
    #     #     node_nextlastv = np.array([[node_nextlast.n], [node_nextlast.e], [node_nextlast.d]])
    #     #     node_last = bestPath[-1]
    #     #     node_lastv = np.array([[node_last.n], [node_last.e], [node_last.d]])
    #     #     q_start = node2v - node1v
    #     #     q_end = node_lastv - node_nextlastv

    #     #     new_node2 = node1v + 5.0*q_start/np.linalg.norm(q_start)
    #     #     new_last = node_lastv + 5*q_end/np.linalg.norm(q_end)

    #     #     node2 = msg_ned(new_node2.item(0), new_node2.item(1), new_node2.item(2))
    #     #     nodel = msg_ned(new_last.item(0), new_last.item(1), new_last.item(2))

    # def shortestPath(self, tree, endNode):
    #     """RRT class function that takes in a tree with successful paths and finds which one is the shortest

    #     Parameters
    #     ----------
    #     tree : float
    #         An Nx7 array of N leaves in this format: N, E, D, cost, parentIndex, connectsToGoalFlag, chi
    #     endNode : msg_ned
    #         The ending waypoint.

    #     Returns
    #     -------
    #     path :  msg_ned
    #         An array of waypoints that expresses the shortest (but not smoothed), successful path from one waypoint
    #         to another
    #     """
    #     # Find the leaves that connect to the end node
    #     connectedNodes = []
    #     for i in range(0, np.size(tree, 0)):
    #         if tree[i,5] == 1:
    #             connectedNodes.append(i)

    #     # Find the path with the shortest distance (could find a different heuristic for choosing which path to go with,
    #     # especially because we are going to shorten the path anyway??). Choose shortest after smoothing?? Or choose for
    #     # least turns.
    #     minIndex = np.argmin(tree[connectedNodes,3])
    #     minIndex = connectedNodes[minIndex]
    #     path = []
    #     path.append(endNode)
    #     path.append(msg_ned(tree[minIndex,0],tree[minIndex,1],tree[minIndex,2]))
    #     parentNode = int(tree[minIndex,4])
    #     while parentNode > 0:
    #         path.append(msg_ned(tree[parentNode,0],tree[parentNode,1],tree[parentNode,2]))
    #         parentNode = int(tree[parentNode,4])
    #     path.append(msg_ned(tree[parentNode, 0], tree[parentNode, 1], tree[parentNode, 2])) #This adds the starting point
    #     # # The commented lines prints the shortest, but not yet smoothed, path
    #     # if self.animate:
    #     #     self.drawPath(path,'r')
    #     return path


def find_path(w1, w2, obstacles, bound_poly, start_chi=None, connect=False, config=Config, animation=None):
    """
    Finds a path between two waypoints passed in, accounting for obstacles
    and boundaries.

    Parameters
    ----------
    w1 : metis.messages.msg_ned
        The starting waypoint
    w2 : metis.messages.msg_ned
        The ending waypoint. Super creative names, I know.
    obstacles : list of metis.messages.msg_ned
        A list of obstacles.
    bound_poly : shapely.geometry.polygon.Polygon
        A Polygon object representing the mission boundaries.
    start_chi : float, optional
        The current heading of the path (default None). If None, indicates 
        first step in the full path and is ignored.
    connect : boolean, optional
        If True, the path will be generated such that an additional waypoint 
        is created after every primary waypoint to force the plane to go 
        through the primary waypoint before beginning a turn (default False).
    config : metis.rrt.Config
        Parameters for RRT.

    Returns
    -------
    smoothedPath :  msg_ned
        The list of waypoints which outlines a safe path to follow in order to reach the two waypoints passed in.
    """
    _logger = _module_logger.getChild("find_path")

    # node state vector format: N, E, D, cost, parentIndex, connectsToGoalFlag, chi
    start = Node(w1.n, w1.e, w1.d, chi=start_chi)
    tree = Tree(root=start)

    # check for if solution at the beginning    
    chi = np.arctan2((w2.e - w1.e), (w2.n - w1.n))
    if flyable_path(obstacles, bound_poly, w1, w2, start.chi, chi, config):
        _logger.critical("option 0")
        return [w1, w2]  # Returns the two waypoints as the succesful path

    #START NEW TESTING CODE
    q = (w2.nparray - w1.nparray)/np.linalg.norm(w2.nparray - w1.nparray)

    add_node = w2.nparray + q*config.distance
    newmsg = msg_ned(add_node.item(0), add_node.item(1), add_node.item(2))

    if flyable_path(obstacles, bound_poly, w1, newmsg, start_chi, chi, config) and connect:
        # return w1, w2, msg_ned(add_node.item(0), add_node.item(1), add_node.item(2))
        _logger.critical("option 1")
        return [w1, w2, newmsg]

    elif flyable_path(obstacles, bound_poly, w1, w2, start_chi, chi, config) and not connect:
        _logger.critical("option 2")
        return [w1, w2]
    
    #END NEW TESTING CODE
    else:
        _logger.critical("option 3")
        foundSolution = 0
        while foundSolution < 3: # This will keep expanding the tree the amount of iterations until solution found
            _logger.critical("Iterating foundSolution")
            for i in range(0, config.iterations):
                tree, flag = extend_tree(tree, w1, w2, obstacles, bound_poly, config)
                if flag:
                    _logger.debug("Found potential path.")
                foundSolution += 1

    # Find complete paths
    connectedPaths = []
    # for i in range(0, len(tree)):
    for leaf in tree.nodes:
        if leaf.connects:
            # _logger.critical("This leaf connects")
            connectedNodes = []
            connectedNodes.append(w2)
            connectedNodes.append(leaf.ned)
            parent = leaf.parent
            while parent is not None:
                connectedNodes.append(parent.ned)
                parent = parent.parent
            # connectedNodes.append(w1)
            connectedPaths.append(connectedNodes)

    # # Find the shortest path
    # path = self.shortestPath(tree, waypoint2)
    # # Smooth the path
    # smoothedPath = self.smoothPath(path)
    # return smoothedPath

    # Smooth all paths and save the best
    bestPath = []
    bestCost = np.inf
    for path in connectedPaths:
        smoothedPath, cost = smooth_path(path, start_chi, obstacles, bound_poly, config=config)

        if connect:
            print("Connect")

            last_node = np.array([[smoothedPath[-1].n],[smoothedPath[-1].e],[smoothedPath[-1].d]])
            prep_node = np.array([[smoothedPath[-2].n],[smoothedPath[-2].e],[smoothedPath[-2].d]])

            q = (last_node - prep_node)/np.linalg.norm(last_node - prep_node)

            add_node = last_node + q*config.distance

            if flyable_path(obstacles, bound_poly, smoothedPath[-1], msg_ned(add_node.item(0), add_node.item(1), add_node.item(2)), 0, 0, config):
                smoothedPath.append(msg_ned(add_node.item(0), add_node.item(1), add_node.item(2)))

        if cost <  bestCost:
            bestPath = smoothedPath
            bestCost = cost

    # bestPathNew = []
    # bestPathNew.append(bestPath[0])
    # bestPathNew.append(node2)
    # bestPathNew.append(bestPath[1:])
    # bestPathNew = bestPathNew.append(nodel)
    # bestPath = bestPathNew
    # elif len(bestPath) > 2:
    #     pass

    # elif len(bestPath) == 2:
    #     pass
    # debug = False
    # if debug:
    #     n = np.array([item.n for item in bestPath])
    #     e = np.array([item.e for item in bestPath])
    #     plt.plot(e, n, 'rx-')
    #     for i in range(len(n)):
    #         plt.text(e[i], n[i], str(i))
    #     plt.axis('equal')
    #     plt.show()
    
    return bestPath


def extend_tree(tree, startN, endN, obstacles, bound_poly, config):
    """
    Extends the passed-in tree. It will continually attempt to add a leaf 
    until it finds a successful one. This is the basic RRT algorithm.

    Parameters
    ----------
    tree : float
        An Nx7 array of N leaves in this format: N, E, D, cost, parentIndex, connectsToGoalFlag, chi
    startN : msg_ned
        The starting waypoint.
    endN : msg_ned
        The ending waypoint.
    obstacles : list of metis.messages.msg_ned
        The list of static obstacles to avoid.
    bound_poly : Polygon
        A Polygon object representing the boundary of the mission.
    config : Config, optional
        A configuration object for custom parameters.

    Returns
    -------
    tree : metis.rrt.Tree
        A Tree object containing the extended tree.
    flag : bool
        Returns True if a path to the end node was found, False if not.
    """
    _logger = _module_logger.getChild('extend_tree')
    _logger.debug('Entering extend_tree')
    minN, minE, maxN, maxE = bound_poly.bounds
    
    # Loop until we have a path that is viable
    flyable = False
    while not flyable:
        # Generate a random point
        _logger.debug('Generating random point')
        northP, eastP = random_point(maxN, minN, maxE, minE)

        # *********************************************************************
        # Find the nearest leaf. Preference given to leaves that are at the 
        # correct altitude.
        new_node = Node(northP, eastP, endN.d)
        closest = tree.closest(new_node)

        # Need to find way to get more smooth descents and ascents?? not zig zaggy
        # chi = np.arctan2((new_node.e - closest.e), (new_node.n - closest.n))
        chi = heading(closest.ned, new_node.ned)

        # *********************************************************************
        # Calculate the new node location

        # If the chosen leaf is at the ending waypoint altitude
        if(closest.d == endN.d):
            # A new leaf only extends a maximum distance from a previous leaf
            # L = min(np.sqrt((northP-closest.n)**2 + (eastP-closest.e)**2), config.max_distance)
            connection = (new_node.ned - closest.ned).nparray
            L = min(np.linalg.norm(connection), config.max_distance)
            newPoint = closest.ned + L*(connection / np.linalg.norm(connection))
            newNode = Node(newPoint.n, newPoint.e, newPoint.d, closest.cost + L, closest, False, chi)
        
        # This case is for when the nearest leaf isn't yet at the correct altitude for the ending waypoint
        else:
            hyp = np.sqrt((northP-closest.n)**2 + (eastP-closest.e)**2)
            lessInclineWhilePlanning = .3
            if startN.d > endN.d:
                downP = closest.d - hyp * config.max_incline * lessInclineWhilePlanning
            else:
                downP = closest.d + hyp * config.max_incline * lessInclineWhilePlanning
            q = np.array([northP - closest.n, eastP - closest.e, downP - closest.d])
            L = np.linalg.norm(q)
            L = min(L, config.max_distance)
            tmp = np.array([northP, eastP, downP]) - closest.ned.nparray
            newPoint = closest.ned + L*(tmp/np.linalg.norm(tmp))
            # Check for overshooting the correct altitude
            if (startN.d > endN.d and newPoint.d < endN.d) or (startN.d < endN.d and newPoint.d > endN.d):
                newPoint.d = endN.d
            newNode = Node(newPoint.n, newPoint.e, newPoint.d, closest.cost+L, closest, False, chi)

        # *********************************************************************
        # Find paths
        if closest.parent is not None:
            node_1 = closest.parent.ned
            node_2 = closest.ned
            node_3 = newNode.ned
        else:
            node_1 = closest.ned
            node_2 = newNode.ned
            node_3 = None
        
        # Check for collision. If we have a flylable path, break out of the loop!
        flyable = flyable_path(obstacles, bound_poly, node_1, node_2, closest.chi, chi, config, node_3)

    # Add this new node to the tree of viable paths.
    tree.add(newNode)

    # Check to see if the new node can connect to the end node.
    dist = np.linalg.norm((endN - newNode.ned).nparray) # FIXME: This does nothing?
    chi = np.arctan2((endN.e - newNode.e), (endN.n - newNode.n))

    if flyable_path(obstacles, bound_poly, newNode.ned, endN, newNode.chi, chi, config):
        # Return the extended tree with the flag of a successful path to ending node
        newNode.connects = True
        _logger.debug('Exiting extend_tree')
        return tree, True
    else:
        # Return the extended tree, and we still haven't reached the last point.
        _logger.debug('Exiting extend_tree')
        return tree, False


def smooth_path(path, prev_chi, obstacles, bound_poly, config=Config):
    """ 
    Takes in an array of waypoints and tries to find a flyable, smooth path.

    Parameters
    ----------
    path : list of metis.messages.msg_ned
        The list of waypoints.
    prev_chi : float
        Initial heading of point or 8888 if start

    Returns
    -------
    path : msg_ned
        An array of waypoints that expresses the smoothed, successful path 
        through all the waypoints in reversed order.
    """
    _logger = _module_logger.getChild('smooth_path')
    # Improve smoother. Because of chi constraint, it doesn't do well at 
    # cutting out lots of segments. First try getting all paths before 
    # trimming them.
    
    # Create a list that we'll fill as we create the smoothed path.
    # smoothedPath = [metis.messages.msg_ned]
    smoothedPath = [path[0]]

    for index in range(1, len(path) - 1):
        chi = heading(smoothedPath[-1], path[index+1])
        node_3 = None

        # This is to know if we aren't checking second to last node
        # Have to check the flyability of the node after in addition to current one
        if index + 2 < len(path):
            _logger.critical("Planning with third node")
            chi2 = heading(path[index+1], path[index+2])
            node_3 = path[index+2]
        else:
            _logger.critical("Planning WITHOUT third node")
            
        # If the "smoothed" path can't be flown (i.e. intersects an obstacle), 
        # retain the unsmoothed point.
        if not flyable_path(obstacles, bound_poly, smoothedPath[-1], path[index+1], prev_chi, chi, config, node_3):
            smoothedPath.append(path[index])
            prev_chi2 = heading(smoothedPath[-2], smoothedPath[-1])

    smoothedPath.append(path[-1])

    # Cost is simply the total length of the path.
    cost = 0
    for first, second in enumerate(range(1, len(smoothedPath))):
        cost += np.linalg.norm((smoothedPath[first] - smoothedPath[second]).nparray)
    # Path was saved in reverse order, so it had to be flipped
    _logger.info("Changes after smoothing: Before = {}, After = {}".format(len(path), len(smoothedPath)))
    path = smoothedPath[::-1]
    return path, cost


def flyable_path(obstacles, bound_poly, startNode, endNode, prevChi, chi, config, third_node=None):
    """
    Checks if flying between two points is  possible. It checks for 
    collisions, chi angle, and incline.

    Parameters
    ----------
    obstacles : list of metis.messages.msg_ned
        List of obstacles to avoid.
    bound_poly : shapely.geometry.polygon.Polygon
        The mission boundaries, as a polygon.
    startNode : msg_ned
        The starting node
    endNode : msg_ned
        The ending node
    prevChi : double
        The chi angle of the leaf being added to
    chi : double
        The chi angle made by added leaf
    config : Config
        A configuration object.

    Returns
    -------
    boolean
        Returns true if a flyable path, false if not
    """
    _logger = _module_logger.getChild('flyable_path')

    #check for obstacles and boundaries
    N, E, D = pointsAlongPath(startNode, endNode, config.resolution, third_node=third_node, R=config.min_radius)

    if will_collide(obstacles, bound_poly, N, E, D, config.clearance):
        return False

    #Check for new leaf now above max relative chi angle
    if prevChi != None: #If not at the root node
        prevChi = wrap2pi(prevChi)
        chi = wrap2pi(chi)
        wrappedPrevChi = wrapAminusBToPi(prevChi, chi)
        # If the difference in headings is more than some amount:
        if abs(wrappedPrevChi) > config.max_rel_chi: # FIXME: change > to < - don't we want to avoid tight turns?
            _logger.debug("Chi difference too large, {} > {}".format(abs(wrappedPrevChi) , config.max_rel_chi))
            _logger.debug("prevChi = {}, chi = {}".format(prevChi, chi))
            return False
        _logger.critical('Passthrough Max Rel Chi: ' + str(config.max_rel_chi))

    #Check incline here
    incline = np.abs( (endNode.d-startNode.d) / np.sqrt((endNode.n-startNode.n)**2 + (endNode.e-startNode.e)**2) )
    if incline > config.max_incline+.01:  #Added fudge factor because of floating point math errors
        _logger.debug("Incline too steep.")
        return False

    return True


def pointsAlongPath(startN, endN, stepSize, third_node=None, R=None):
    """ RRT class f
    Function that takes two nodes and returns the N, E, and D position of many points along the line
    between the two points spaced according to the step size passed in.

    Parameters
    ----------
    startN : metis.messages.msg_ned
        The starting node
    endN : metis.messages.msg_ned
        The ending node
    stepSize : float
        The desired spacing between each point along the line between the two nodes
    third_node : metis.messages.msg_ned or None
        The next node in the waypoint path. If this is not None then the points will be along a fillet path
    R : float or None
        The minimum turn radius. If None, straight line path is used

    Returns
    -------
    N : float
        An np.array of the north position of points
    E : float
        An np.array of the east position of points
    D : float
        An np.array of the down position of points
    """
    if third_node is None or R is None:
        # log.info("Using NEXTNODE paths.")
        N, E, D = stepped_range_linear(startN, endN, stepSize)
    
    else: #Plan fillet path
        # log.info("Using FILLET paths.")
        R = float(R)
        N = np.array([])
        E = np.array([])
        D = np.array([])

        start_node = np.array([[startN.n], [startN.e], [startN.d]])
        mid_node = np.array([[endN.n], [endN.e], [endN.d]])
        last_node = np.array([[third_node.n], [third_node.e], [third_node.d]])

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
            current_position = np.array([[startN.n], [startN.e], [startN.d]])
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

        else:
            start_node = msg_ned(start_node.item(0), start_node.item(1), start_node.item(2))
            mid_node = msg_ned(mid_node.item(0), mid_node.item(1), mid_node.item(2))
            last_node = msg_ned(last_node.item(0), last_node.item(1), last_node.item(2))
            N,E,D = pointsAlongPath(start_node, mid_node, stepSize)
            N2,E2,D2 = pointsAlongPath(mid_node, last_node, stepSize)
            N = np.append(N,N2)
            E = np.append(E,E2)
            D = np.append(D,D2)
            
    return N,E,D


def stepped_range_linear(startN, endN, step_size):
    """
    Creates a stepped range of values from the starting node to the ending node
    with difference in step size guaranteed to be less than step_size.

    Parameters
    ----------
    startN : metis.messages.msg_ned
        The starting node to create a range from.
    endN : metis.messages.msg_ned
        The ending node to create a range to.
    step_size : float
        The maximum delta length between steps.
    
    Returns
    -------
    (N, E, D) : tuple
        A tuple containing three arrays; the steps of N, E, and D.
    """
    q0 = np.array([endN.n-startN.n, endN.e-startN.e, endN.d-startN.d])
    points = int(np.ceil(np.linalg.norm(q0)/step_size)) + 1
    N = np.linspace(startN.n, endN.n, num=points)
    E = np.linspace(startN.e, endN.e, num=points)
    D = np.linspace(startN.d, endN.d, num=points)
    return N, E, D


def random_point(nmax, nmin, emax, emin):
    """
    Creates a random point in the 2D plane bounded by the max and min 
    boundary positions.

    Parameters
    ----------
    nmax : float
        The northernmost boundary.
    nmin : float
        The southernmost boundary.
    emax : float
        The easternmost boundary.
    emin : float
        The westernmost boundary.

    Returns
    -------
    rand : (float, float)
        A tuple of two floats representing positions in the form (north, east),
        bounded by the parameters.
    """
    n = np.random.uniform(low=nmin, high=nmax)
    e = np.random.uniform(low=emin, high=emax)
    return (n, e)


def heading(p0, p1):
    """
    Computes the navigational heading from the first waypoint to the second.

    Parameters
    ----------
    p0 : metis.messages.msg_ned
        The origin waypoint.
    p1 : metis.messages.msg_ned
        The destination waypoint.
    
    Returns
    -------
    chi : float
        The heading from the origin to the destination waypoints.

    Examples
    --------
    >>> np.degrees(heading(msg_ned(0,0), msg_ned(10, 0)))
    0.0
    >>> np.degrees(heading(msg_ned(0,0), msg_ned(0, 10)))
    90.0
    """
    return np.arctan2((p1.e - p0.e), (p1.n - p0.n))


def wrap(chi_c, chi):
    """ 
    Wraps an angle relative to another angle.

    Suppose an aircraft is flying at a compass heading of 165 degrees 
    (2.88 rad) and is then commanded a heading of 270 degrees (-1.57 rad).
    Since headings, when stored in radians, are represented on an interval 
    from (-pi, pi], the aircraft will believe that it needs to make a 
    255 degree left turn from 165 degrees back to -90 degrees when instead
    it should make 105 degree right turn. This function recognizes that the
    commanded values are closer to the actual headings and wraps the angle
    to the closest radian representation of the heading, even if it lies 
    outside of the interval (-pi, pi].

    Parameters
    ----------
    chi_c : double
        Angle that will be wrapped relative to some reference angle in radians.
    chi : double
        Reference angle in radians.

    Returns
    -------
    chi_c : double
        Returns the commanded angle wrapped relative to the reference angle.

    Examples
    --------
    >>> wrap(np.radians(170), np.radians(-170))
    -3.316125578789226
    >>> wrap(np.radians(185), np.radians(180))
    3.2288591161895095
    >>> wrap(np.radians(90), np.radians(160))
    1.5707963267948966
    >>> wrap(np.radians(100), np.radians(260))
    1.7453292519943295
    >>> wrap(np.radians(100), np.radians(-100))
    -4.537856055185257
    """
    while chi_c - chi > np.pi:
        chi_c = chi_c - 2.0 * np.pi
    while chi_c - chi < -np.pi:
        chi_c = chi_c + 2.0 * np.pi
    return chi_c


def wrap2pi(rad):
    """
    Wraps an angle in radians onto an interval from (-pi, pi].

    Parameters
    ----------
    rad : float
        The angle to be wrapped, in radians.
    
    Returns
    -------
    wrap : float
        The wrapped angle, in radians.

    Examples
    --------
    >>> wrap2pi(np.radians(-180))
    3.141592653589793
    >>> wrap2pi(np.radians(180))
    3.141592653589793
    >>> wrap2pi(np.radians(-179))
    -3.12413936106985
    >>> wrap2pi(np.radians(380))
    0.3490658503988664
    """
    # Map angle (in radians) onto an interval from [0, 2*pi).
    wrap = np.mod(rad, 2*np.pi)
    # If angle > pi, wrap around to be on an interval from (-pi, pi].
    if np.abs(wrap) > np.pi:
        wrap -= 2*np.pi*np.sign(wrap)
    return wrap


def wrapAminusBToPi(A, B):
    # Is this just a difference in headings?
    diff_wrap = wrap2pi(A - B)
    return diff_wrap


if __name__ == "__main__":
    test = RRT([msg_ned(-500,-500,10,5)],[msg_ned(-500,-500,10,5),msg_ned(-500,500,0), msg_ned(500, 500, 0), msg_ned(500, -500, 0)])
    test.find_full_path([msg_ned(0., 0., 0.0), msg_ned(0., 100., 0.0),msg_ned(100., 0., 0.0)])
