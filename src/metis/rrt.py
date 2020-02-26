# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

from __future__ import print_function

import logging

import numpy as np
# import matplotlib as mpl
# import matplotlib.pyplot as plt
# from matplotlib import cm
# from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from metis.messages import msg_ned
from metis.tools import collisionCheck

# import cProfile

# To do: A buffer for incline when planning, that way it can smooth
# Find all possible paths, smooth all, then pick best
# Could do an improved smoother where is doesn't just check adjacent nodes. But can't be a factorial check.

log = logging.getLogger('METIS')
_module_logger = logging.getLogger(__name__)
# log = _module_logger

class Config(object):
    clearance = 5.0
    max_distance = 50.0
    min_radius = 20.0
    max_incline = 0.5
    max_rel_chi = 15*np.pi/16
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
        


class Animation(object):
    _logger = _module_logger.getChild('Animation')

    def __init__(self, obstacles, boundaries):
        pass
#         mpl.rcParams['legend.fontsize'] = 10
#         self.fig = plt.figure()
#         self.ax = self.fig.gca(projection='3d')
#         for obstacle in obstacles:
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
#         first = True
#         boundaries = []
#         last = []
#         for bounds in boundaries:
#             if first:
#                 boundaries = np.array([[bounds.n, bounds.e, 0.]])
#                 last = np.array([[bounds.n, bounds.e, 0.]])
#                 first = False
#                 continue
#             boundaries = np.append(boundaries, [[bounds.n, bounds.e, 0.]], axis=0)
#         boundaries = np.append(boundaries, last, axis=0)
#         self.ax.plot(boundaries[:, 0], boundaries[:, 1], boundaries[:, 2], label='Boundaries')
#         self.ax.set_xlabel('X axis')
#         self.ax.set_ylabel('Y axis')
#         self.ax.set_zlabel('Z axis')
#         plt.xlim(self.maxN*1.1, self.minN*1.1)
#         plt.ylim(self.minE * 1.1, self.maxE * 1.1)
#         self.ax.elev = 90 #55
#         self.ax.azim = 0 #80
#         self.viridis = cm.get_cmap('viridis', 12)
#         self.viridis = cm.get_cmap('viridis')
#         self.ax.legend()

    # def drawPath(self, path, color):
    #     """ RRT class function that draws the path between a list of waypoints

    #     Parameters
    #     ----------
    #     path : msg_ned
    #         List of waypoints
    #     color : string
    #         Desired color in format for matplot (e.g. 'r','y','b',etc.)
    #     """
    #     for i in range(0, len(path) - 1):
    #         way1 = path[i]
    #         way2 = path[i + 1]
    #         self.ax.plot([way1.n, way2.n], [way1.e, way2.e], [-way1.d, -way2.d], color=color)

class Tree(object):
    _logger = _module_logger.getChild('Tree')

    def __init__(self, root=None):
        """
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
        return len(self.nodes)

    def closest(self, node):
        """
        Parameters
        ----------
        node : Node
            The node we're trying to get close to.
        
        Returns
        -------
        Node
            The closest node already in the tree to the passed in Node.
        """
        dists = []
        for child in self.nodes:
            dists.append(child.distance(node))
        min_idx = np.argmin(dists)
        return self.nodes[min_idx]

    def add(self, node):
        self.nodes.append(node)


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
        cost : float
            Cost of the node.
        parent : reference to metis.rrt.Node
            A reference to the parent node of the object.
        connects : bool
            Whether or not this node connects to the goal; true if it connects,
            false otherwise.
        chi : float
            Heading.
        """
        super(Node, self).__init__()
        self.ned = msg_ned(n, e, d)
        self.cost = cost
        self.parent = parent
        self.connects = connects
        self.chi = chi

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

    # @property
    # def isroot(self):
    #     return self.parent == None

    def distance(self, other):
        return np.linalg.norm((other.ned - self.ned).nparray)


class RRT():
    """
    An RRT object plans plans flyable paths in the mission environment. It also holds the information concerning
    the physical boundaries and obstacles in the competition.
    """
    _logger = _module_logger.getChild('RRT')

    def __init__(self, obstaclesList, boundariesList, animate=False, config=Config()):
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
        # np.random.seed(1111) # For Debugging
        self.config = config

        # Save obstacles and boundaries
        self.obstaclesList = obstaclesList
        self.boundariesList = boundariesList
        self.animation = Animation(self.obstaclesList, self.boundariesList) if animate else None

        # Boundaries now contained in a Polygon object
        self.bound_poly = Polygon([[bound.n, bound.e] for bound in boundariesList])

    def findFullPath(self, waypoints, connect=False, config=Config):
        """
        Finds a path to all of the waypoints passed in. This path accounts for
        obstacles, boundaries, and all other parameters set in __init__.

        Parameters
        ----------
        waypoints : list of metis.messages.msg_ned
            A list of waypoints
        connect : bool, optional
            If true, the path will be generated such that an additional waypoint is created after every primary waypoint to 
            force the plane to go through the primary waypoint before beginning a turn

        Returns
        -------
        fullPath : msg_ned
            The full list of waypoints which outlines a safe path to follow in order to reach all of the waypoints
            passed in.
        """
        self._logger.info("Finding full path for {} waypoints with connect={}".format(len(waypoints), connect))
        numWaypoints = len(waypoints)
        fullPath = []
        index = 0 
        way1 = waypoints[index]
        chi = 8888

        # for i, j in zip(range(5), range(1,6)):
        #     print(i, j)

        while index < numWaypoints-1:  # Do each segment of the path 

            index2 = index+1
            way2 = waypoints[index2]
            # Check to make sure if the waypoints are possible
            if not collisionCheck(self.obstaclesList,self.bound_poly,np.array([way1.n]), np.array([way1.e]), np.array([way1.d]), config.clearance):
                #Don't worry about if the first waypoint is flyable because that is the start position and we want to plot a path even if we start out of bounds
                if index != 0:
                    index += 1
                    way1 = waypoints[index]
                    continue

            while not collisionCheck(self.obstaclesList, self.bound_poly, np.array([way2.n]), np.array([way2.e]), np.array([way2.d]), config.clearance):
                if index2 < numWaypoints-1:
                    index2 += 1
                    way2 = waypoints[index2]
                    print("Waypoint is out of bounds or on an obstacle")
                else:
                    break

            # newPath = self.findPath(way1, way2, chi, connect=connect)  # call the findPath function to find path between these two waypoints
            newPath = find_path(way1, way2, self.obstaclesList, self.bound_poly, chi, connect=connect)

            print("Individual Path Found")
            if (len(fullPath) > 0) and (fullPath[-1] == newPath[0]):
                newPath = newPath[1:]
            fullPath += newPath  # Append this segment of the path to the full path
            index += index2-index
            way1 = fullPath[-1]
            pre_way = fullPath[-2]
            chi = np.arctan2((way1.e - pre_way.e), (way1.n - pre_way.n))

        return fullPath

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


def find_path(w1, w2, obstacles, bound_poly, start_chi=None, connect=False, config=Config):
    """RRT class function that finds a path between two waypoints passed in. This solved path takes into account obstacles,
    boundaries, and all other parameters set in the init function.

    Parameters
    ----------
    w1 : msg_ned
        The starting waypoint
    w2 : msg_ned
        The ending waypoint. Super creative names, I know.
    start_chi : float
        The current heading of the path. If 8888, indicates first step in the full path and is ignored
    connect : boolean
        If true, the path will be generated such that an additional waypoint is created after every primary waypoint to 
        force the plane to go through the primary waypoint before beginning a turn

    Returns
    -------
    smoothedPath :  msg_ned
        The list of waypoints which outlines a safe path to follow in order to reach the two waypoints passed in.
    """
    # node state vector format: N, E, D, cost, parentIndex, connectsToGoalFlag, chi
    start = Node(w1.n, w1.e, w1.d, chi=start_chi)
    tree = Tree(root=start)

    # check for if solution at the beginning    
    chi = np.arctan2((w2.e - w1.e), (w2.n - w1.n))
    if flyablePath(obstacles, bound_poly, w1, w2, start.chi, chi):
        return w1, w2  # Returns the two waypoints as the succesful path

    #START NEW TESTING CODE
    q = (w2.nparray - w1.nparray)/np.linalg.norm(w2.nparray - w1.nparray)

    add_node = w2.nparray + q*config.distance
    newmsg = msg_ned(add_node.item(0), add_node.item(1), add_node.item(2))

    if flyablePath(obstacles, bound_poly, w1, newmsg, start_chi, chi) and connect:
        # return w1, w2, msg_ned(add_node.item(0), add_node.item(1), add_node.item(2))
        log.critical("option 1")
        return w1, w2, newmsg

    elif flyablePath(obstacles, bound_poly, w1, w2, start_chi, chi) and not connect:
        log.critical("option 2")
        return w1, w2
    
    #END NEW TESTING CODE
    else:
        log.critical("option 3")
        foundSolution = 0
        while foundSolution < 3: # This will keep expanding the tree the amount of iterations until solution found
            for i in range(0, config.iterations):
                tree, flag = extend_tree(tree, w1, w2, obstacles, bound_poly)
                if flag:
                    print("Found Potential Path")
                foundSolution += 1

    # Find complete paths
    connectedPaths = []
    # for i in range(0, len(tree)):
    for leaf in tree.nodes:
        if leaf.connects:
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
        smoothedPath, cost = smooth_path(path, start_chi, obstacles, bound_poly)

        if connect:
            print("Connect")

            last_node = np.array([[smoothedPath[-1].n],[smoothedPath[-1].e],[smoothedPath[-1].d]])
            prep_node = np.array([[smoothedPath[-2].n],[smoothedPath[-2].e],[smoothedPath[-2].d]])

            q = (last_node - prep_node)/np.linalg.norm(last_node - prep_node)

            add_node = last_node + q*config.distance

            if flyablePath(obstacles, bound_poly, smoothedPath[-1], msg_ned(add_node.item(0), add_node.item(1), add_node.item(2)), 0, 0):
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
    
    return bestPath


def extend_tree(tree, startN, endN, obstacles, bound_poly, config=Config):
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
    minN, minE, maxN, maxE = bound_poly.bounds
    
    # Loop until we have a path that is viable
    flyable = False
    while not flyable:
        # Generate a random point
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
        flyable = flyablePath(obstacles, bound_poly, node_1, node_2, closest.chi, chi, node_3, config.min_radius)

    # Add this new node to the tree of viable paths.
    tree.add(newNode)

    # Check to see if the new node can connect to the end node.
    dist = np.linalg.norm((endN - newNode.ned).nparray)
    chi = np.arctan2((endN.e - newNode.e), (endN.n - newNode.n))

    if flyablePath(obstacles, bound_poly, newNode.ned, endN, newNode.chi, chi):
        # Return the extended tree with the flag of a successful path to ending node
        newNode.connects = True
        return tree, True
    else:
        # Return the extended tree, and we still haven't reached the last point.
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
            chi2 = heading(path[index+1], path[index+2])
            node_3 = path[index+2]
            
        if not flyablePath(obstacles, bound_poly, smoothedPath[-1], path[index+1], prev_chi, chi, node_3, config.min_radius):
            smoothedPath.append(path[index])
            prev_chi2 = heading(smoothedPath[-2], smoothedPath[-1])

    smoothedPath.append(path[-1])

    cost = 0
    for first, second in enumerate(range(1, len(smoothedPath))):
        cost += np.linalg.norm((smoothedPath[first] - smoothedPath[second]).nparray)
    reversePath = smoothedPath[::-1]  # Path was saved in reverse order, so it had to be flipped
    return reversePath, cost

def flyablePath(obstacles, bound_poly, startNode, endNode, prevChi, chi, third_node=None, R=None, config=Config):
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

    Returns
    -------
    boolean
        Returns true if a flyable path, false if not
    """
    #check for obstacles and boundaries
    N, E, D = pointsAlongPath(startNode, endNode, config.resolution, third_node=third_node, R=config.min_radius)

    collisionChecked = collisionCheck(obstacles, bound_poly, N, E, D, config.clearance)
    if not collisionChecked:
        return False

    #Check for new leaf now above max relative chi angle
    if prevChi != None: #If not at the root node
        prevChi = wrapToPi(prevChi)
        chi = wrapToPi(chi)
        wrappedPrevChi = wrapAminusBToPi(prevChi, chi)
        if abs(wrappedPrevChi) > config.max_rel_chi: # changed > to < - don't we want to avoid tight turns?
            print("Chi difference too large, {} > {}".format(abs(wrappedPrevChi) , config.max_rel_chi))
            print("prevChi = {}, chi = {}".format(prevChi, chi))
            return False

    #Check incline here
    incline = np.abs((endNode.d - startNode.d)/np.sqrt((endNode.n - startNode.n) ** 2 + (endNode.e - startNode.e) ** 2) )
    if incline > config.max_incline+.01:  #Added fudge factor because of floating point math errors
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
    rand : tuple
        A tuple of two floats representing positions in the form (north, east), bounded by the parameters.
    """
    n = np.random.uniform(low=nmin, high=nmax)
    e = np.random.uniform(low=emin, high=emax)
    return (n, e)

def heading(p0, p1):
    """
    Computes the heading from one waypoint to another.

    Parameters
    ----------
    p0 : metis.messages.msg_ned
        The origin waypoint.
    p1 : metis.messages.msg_ned
        The destination waypoint.
    
    Returns
    -------
    chi : float
        The heading from the origin to the destinationw waypoints.
    """
    return np.arctan2((p1.e - p0.e), (p1.n - p0.n))

def wrap(chi_c, chi):
    """ RRT class function that wraps an angle.

    Parameters
    ----------
    chi_c : double
        Angle that will be wrapped
    chi : double
        Reference angle

    Returns
    -------
    chi_c : double
        Returns the wrapped angle
    """
    while chi_c - chi > np.pi:
        chi_c = chi_c - 2.0 * np.pi
    while chi_c - chi < -np.pi:
        chi_c = chi_c + 2.0 * np.pi
    return chi_c

def wrapToPi(x):
    wrap = np.mod(x, 2*np.pi)
    if np.abs(wrap) > np.pi:
        wrap -= 2*np.pi*np.sign(wrap)
    return wrap

def wrapAminusBToPi(A, B):
    diff_wrap = wrapToPi(A - B)
    return diff_wrap

if __name__ == "__main__":
    test = RRT([msg_ned(-500,-500,10,5)],[msg_ned(-500,-500,10,5),msg_ned(-500,500,0), msg_ned(500, 500, 0), msg_ned(500, -500, 0)])
    test.findFullPath([msg_ned(0., 0., 0.0), msg_ned(0., 100., 0.0),msg_ned(100., 0., 0.0)])
