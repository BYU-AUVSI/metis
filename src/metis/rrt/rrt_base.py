# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import logging

import numpy as np
from shapely.geometry import Point

from metis.location import Waypoint, convert_point
from .animation import Animation2D


_module_logger = logging.getLogger(__name__)

class Config(object):
    clearance = 5.0
    # max_distance = 150.0 # 50.0
    max_distance = 50.0
    min_radius = 20.0
    # min_radius = 30.0
    max_incline = 0.5
    max_rel_chi = 15*np.pi/16
    # max_rel_chi = np.radians(60)
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
            Minimum turn radius of the aircraft for planning fillet and dubins
            paths (default 20).
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
        root : metis.core.NEDPoint
            The root Node of the tree.
        """
        super(Tree, self).__init__()
        if root:
            if type(root) is not Node:
                raise TypeError('Tree can only hold Node types')
            self.nodes = [root]
        else:
            self.nodes = []

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

    def closest(self, node, distance=None):
        """
        Finds the node in the tree closest to some other node.

        Parameters
        ----------
        node : metis.core.NEDPoint
            A random node we're trying to find the nearest neighbor of.
        distance : float
            Finds all neighbors within some distance from the node. If None,
            returns the single closest node (default None).
        
        Returns
        -------
        metis.core.NEDPoint
            The node already stored in the tree that is closest to the passed 
            in node.
        """
        if distance:
            neighbors = [child for child in self.nodes if child.distance(node, d2=True) <= distance and not child.connects]
            if len(neighbors) != 0:
                return neighbors
        # If no neighbors, do default behavior: find closest.
        dists = [child.distance(node, d2=True) for child in self.nodes if not child.connects]
        min_idx = np.argmin(dists)
        return [self.nodes[min_idx]]

    def add(self, node):
        if type(node) is not Node:
            raise TypeError('Tree can only hold Node types')
        self.nodes.append(node)


class Node(Waypoint):
    _logger = _module_logger.getChild('Node')

    def __init__(self, n=0.0, e=0.0, d=0.0, chi=0.0, cost=0.0, parent=None, connects=False):
        """
        Parameters
        ----------
        waypoint : metis.location.Waypoint
            Waypoint of node.
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
        super(Node, self).__init__(n, e, d, chi)
        # self.cost = cost
        self.parent = parent
        self.connects = connects

    def __eq__(self, other):
        """
        Equality overridden such that as long as the nodes are in exactly the
        same geographic location, they are considered to be the same node.
        """
        return self.ned == other.ned and \
            self.chi == other.chi

    @property
    def cost(self, d2=True):
        if self.parent is None:
            return 0.0

        return self.parent.cost + self.distance(self.parent, d2=True)
        # parent = self.parent
        # cost = 0.0
        # while parent is not None:
        #     cost += starter.distance(starter.parent, d2=True)
        #     starter = starter.parent
        # return cost


class RRT(object):
    """
    An RRT Interface that should be subclassed.

    An RRT object plans plans flyable paths in the mission environment. 
    To perform this task, it also holds a copy of the information concerning 
    the physical boundaries and obstacles in the competition.
    
    This class models the structure of an RRT planner class. It contains basic
    reusable functions as well as stubbed functions that ought to be 
    implemented by subclasses.
    """
    _logger = _module_logger.getChild('RRT')

    # TODO: Just take a mission object instead of obstacles and boundaries?
    def __init__(self, mission, animate=False, config=Config()):
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
        self.mission = mission

        self.animation = Animation2D(mission) if animate else None

    def filter_invalid_waypoints(self, waypoints):
        """
        Removes all waypoints that are in obstacles or otherwise infeasable,
        excluding the start and end points.

        Parameters
        ----------
        waypoints : list of metis.location.Waypoint
            A list of waypoints to be filtered. The first and last waypoint in
            the list are always left.
        
        Returns
        -------
        waypoints : list of metis.location.Waypoint
            The valid waypoints that are outside of obstacles and within 
            bounds.
        """
        for waypoint in waypoints[1:-1]:
            if collision(waypoint.ned, self.mission.boundary_poly, self.mission.obstacles, self.config.clearance):
                self._logger.warning("Waypoint is out of bounds or on an obstacle: ignoring.")
                waypoints.remove(waypoint)
        return waypoints

    def filter_duplicate_waypoints(self, waypoints):
        """
        Removes adjacent waypoints in a list if they are duplicates.

        Typically used after `find_full_path`. Since the origin of each leg 
        was the destination of the previous leg, we need to remove the 
        repeated nodes.

        Parameters
        ----------
        waypoints : list of metis.location.Waypoint
            A list of waypoints to be filtered.
        
        Returns
        -------
        waypoints : list of metis.location.Waypoint
            The list of waypoints with no adjacent duplicates.
        """
        return [elem for i, elem in enumerate(waypoints) if i == 0 or waypoints[i-1] != elem]

    def find_full_path(self, waypoints):
        """
        Finds a path to all of the waypoints passed in. This path accounts for
        obstacles, boundaries, and all other parameters set in __init__.

        Parameters
        ----------
        waypoints : list of metis.core.Waypoint
            A list of waypoints to be passed through.

        Returns
        -------
        full_path : list of metis.core.Waypoint
            The full list of waypoints which outlines a safe path to follow in 
            order to reach all of the waypoints passed in.
        """
        raise NotImplementedError

    def find_path(self, start, end):
        raise NotImplementedError

    def points_along_path(self, start, end):
        raise NotImplementedError

    def extend_tree(self, tree, goal, seg_length):
        """
        Notes
        -----
        Preference for altitude is not implemented, but could be done by 
        applying a cost penalty to nodes that are at incorrect altitudes.
        This penalty could be increase with the square of the altitude error,
        to push preference strongly towards close nodes.
        """
        raise NotImplementedError

    @staticmethod
    def find_minimum_path(tree, end):
        viable = []
        for node in tree:
            if node.connects:
                viable.append(node)
        costs = [node.cost for node in viable]
        idx = np.argmin(costs)
        path = [viable[idx]]
        while path[-1].parent is not None:
            path.append(path[-1].parent)
        return path[::-1]

    def smooth_path(self, path):
        raise NotImplementedError

    @staticmethod
    def normalize2waypoints(nodes):
        normalized = []
        for node in nodes:
            if type(node) is Waypoint:
                normalized.append(node)
            elif type(node) is Node:
                normalized.append(convert_point(node, Waypoint))
            else:
                raise ValueError('Encountered value that is not a Node or Waypoint')
        return normalized
    
def generate_random_node(nmax, nmin, emax, emin):
    """
    Creates a random point in the 2D plane bounded by the max and min 
    boundary positions.

    Parameters
    ----------
    nmax : float
        The northernmost boundary (in meters).
    nmin : float
        The southernmost boundary (in meters).
    emax : float
        The easternmost boundary (in meters).
    emin : float
        The westernmost boundary (in meters).

    Returns
    -------
    rand : metis.core.Waypoint
        A Waypoint within the region bounded by the parameters (altitude is at
        ground level).
    """
    # _logger = _module_logger.getChild('generate_random_node')
    rand = Node(n=np.random.uniform(low=nmin, high=nmax), e=np.random.uniform(low=emin, high=emax))
    return rand

def points_along_straight(start, end, res):
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
    step_size = res
    q0 = end.ned - start.ned
    points = int(np.ceil(np.linalg.norm(q0)/step_size)) + 1
    n = np.linspace(start.n, end.n, num=points)
    e = np.linspace(start.e, end.e, num=points)
    d = np.linspace(start.d, end.d, num=points)
    ned = np.stack([n, e, d], axis=1)
    return ned

def points_along_arc(start, end, center, radius, lam, res):
    '''
    Calculates the points along an arc.

    Points along the arc are guaranteed to be spaced by at most `res`
    distance apart. Note that this function will work even if the starting
    and ending nodes don't lie on the arc; this is because this function
    simply calculates an angle from the center to the start/end nodes and uses
    that angle to create the points along the arc. This function assumes that 
    the start and end nodes are already on the arc but does not perform any
    error checking.

    Parameters
    ----------
    start : metis.rrt.rrt_base.Node
        The starting node to create a range from.
    end : metis.rrt.rrt_base.Node
        The ending node to create a range to.
    center : subclass of metis.location.NEDPoint
        A point specifying the center of the circle forming the arc.
    radius : float
        The radius of the circle making up the arc.
    lam : int
        Sense of rotation (the direction going around circle). Valid values
        are (1 = clockwise, -1 = counterclockwise).
    res : float
        The maximum distance between the points returned.

    Returns
    -------
    ned : np.ndarray
        An m x 3 numpy array, where each row is an array of north, east, 
        down points.
    '''
    # Headings are all referenced to the NED frame of the vehicle.
    theta1 = heading(center, start)
    theta2 = heading(center, end)
    theta1, theta2 = directional_wrap(theta1, theta2, lam)
    
    # Find how many points we need to get a minimum distance step size
    arc_length = abs(theta1 - theta2) * radius
    points = int(np.ceil(arc_length/res)) + 1

    # Calculate the ned coordinates for the arc
    t = np.linspace(theta1, theta2, points)
    n = radius*np.cos(t) + center.n
    e = radius*np.sin(t) + center.e
    d = np.linspace(end.d, end.d, points)
    
    ned = np.stack([n, e, d], axis=1)
    return ned

def directional_wrap(theta1, theta2, lam):
    '''Wrap theta1 relative to theta2, given the direction of rotation lambda.

    Note that there is no guarantee whether it is theta1 or theta2 that will
    be adjusted; it is only guaranteed that the returned values will be
    correct relative to each other, relative to the sense of rotation.

    Parameters
    ----------
    theta1 : float
        Initial angle (in radians).
    theta2 : float
        Final angle (in radians).
    lam : -1 or 1
        Sense of rotation around circle (1 = clockwise, -1 = counterclockwise).

    Returns
    -------
    theta1 : float
        Angle equivalent to the passed in `theta1` but modified relative to 
        `theta2` and the direction of rotation.
    theta2 : float
        Angle equivalent to the passed in `theta2` but modified relative to 
        `theta1` and the direction of rotation.

    Raises
    ------
    ValueError
        If `lam` is not either int(-1) or int(1).
    '''
    theta1 = wrap(theta1, theta2)
    
    # Clockwise
    if lam == 1:
        if theta1 > theta2:
            theta1 -= 2*np.pi
    # Counter-clockwise
    elif lam == -1: 
        if theta2 > theta1:
            theta2 -= 2*np.pi
    else:
        raise ValueError("Direction '{}' not recognized".format(lam))
    return theta1, theta2

def collision(ned, boundaries, obstacles, clearance=Config.clearance):
    """
    Parameters
    ----------
    ned : np.ndarray
        An n x 3 matrix of north, east, down points. Each row corresponds
        to one coordinate point. The matrix contains n points.
    boundaries : shapely.geometry.polygon.Polygon
        The polygon object representing the boundaries.
    obstacles : list of metis.core.CircularObstacle
        The obstacles in the mission.

    Returns
    -------
    collides : bool
        True if the path collides with something in the mission; False 
        otherwise.
    """
    _logger = _module_logger.getChild('collision')
    if ned is None:
        _logger.warning("'ned' is None")
        return True
        
    # Check for collision with obstacles
    for obstacle in obstacles:
        # first check if the path is entirely above the obstacle
        if all(-ned[:,2] > obstacle.h + clearance):
            continue
        # if it's not, then check if any part of it runs into the obstacle
        else:
            dist = np.linalg.norm(ned[:,:2] - obstacle.ned[:,:2], axis=1)
            if any(dist < obstacle.r + clearance):
                _logger.warning('Points run through obstacle')
                return True

    # Check for out of boundaries
    for p in ned:
        if not boundaries.contains(Point(p.item(1), p.item(0))):
            _logger.warning('Points are out of bounds')
            return True
    return False

def down_at_ne(ne, obstacles):
    """
    Parameters
    ----------
    ne : np.ndarray
        An n x 2 matrix of north and east points. Each row corresponds to one
        coordinate point. The matrix therefore contains n points.

    Returns
    -------
    d : np.ndarray
        A n x 1 matrix of down positions, indexed in the same order as the
        ne points provided as a parameter.
    """
    pass

def heading(p0, p1):
    """
    Computes the navigational heading from the first waypoint to the second.

    Parameters
    ----------
    p0 : metis.rrt.rrt_base.Node
        The origin node.
    p1 : metis.rrt.rrt_base.Node
        The destination node.
    
    Returns
    -------
    chi : float
        The heading from the origin to the destination waypoints (in radians)
        on the interval (-pi, pi].

    Examples
    --------
    >>> np.degrees(heading(msg_ned(0,0), msg_ned(10, 0)))
    0.0
    >>> np.degrees(heading(msg_ned(0,0), msg_ned(0, 10)))
    90.0
    """
    return np.arctan2((p1.e - p0.e), (p1.n - p0.n))

def pitch(p1, p2):
    """
    Calculates the pitch angle travelling from the first waypoint to the second.

    Parameters
    ----------
    p1 : subclass instance of metis.location.NEDPoint
        The first waypoint.
    p2 : subclass instance of metis.location.NEDPoint
        The second waypoint.
    """
    return np.arctan2((p2.h - p1.h), p2.distance(p1, d2=True))

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

def delta_chi(chi0, chi1):
    """
    Calculates the change in heading from chi0 to chi1.

    The sign of the result indicates whether the quickest turn to get to
    the desired heading is a left or a right turn.

    Parameters
    ----------
    chi0 : float
        The previous heading in radians.
    chi1 : float
        The new heading in radians.

    Examples
    --------
    >>> np.degrees(delta_chi(np.radians(345), np.radians(185)))
    -160.0
    >>> np.degrees(delta_chi(np.radians(-15), np.radians(90)))
    105.0
    """
    return wrap2pi(chi1-chi0)

def waypoints2ned(waypoints):
    """Converts a list of waypoints to a ned matrix.

    Parameters
    ----------
    waypoints : list of metis.core.Waypoints
        A list of waypoints to be converted.

    Returns
    -------
    ned : np.ndarray
        An m x 3 NumPy array of north, east, down points, where there are m
        coordinate points.
    """
    ned = np.zeros((len(waypoints), 3))
    for i, p in enumerate(waypoints):
        ned[i] = p.ned
    return ned
