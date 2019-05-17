import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import sys
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
sys.path.append('..')
from messages.ned import msg_ned
from tools.tools import collisionCheck

# To do: A buffer for incline when planning, that way it can smooth
# Find all possible paths, smooth all, then pick best
# Could do an improved smoother where is doesn't just check adjacent nodes. But can't be a factorial check.



class RRT():
    """
    An RRT object plans plans flyable paths in the mission environment. It also holds the information concerning
    the physical boundaries and obstacles in the competition.
    """
    def __init__(self, obstaclesList, boundariesList, clearance=5., maxDistance=10., maxIncline=.5, maxRelChi=np.pi/2, iterations=50, resolution=1.1, scaleHeight=1.5, animate=False):
        """The constructor for the RRT class.

        Parameters
        ----------
        obstaclesList : msg_ned
            A list of all the obstacles in the mission area

        boundariesList : msg_ned
            A list of all the boundary points of the mission area

        clearance : double
            The minimum distance the path can be to any obstacle or boundary

        maxDistance : double
            The maximum distance the next leaf will be extended from the previous leaf

        maxIncline : double
            The maximum incline or decline angle of the planned path

        maxRelChi : double
            The maximum difference in the chi angles of path segments

        iterations : int
            The amount of leaves that will be added until a successful path is found

        resolution : double
            The spacing of points along the path that are checked for collisions. This method was chosen for ease of use
            and could be improved later if wanted. But it should be good enough for our purposes and it runs quickly.

        scaleHeight : double
            This is a scaling value when finding which leaf is closest to the randomly chosen new point. This scales the
            height in the distance formula so that preference is given to leaves that are closer to the ending altitude.

        animate : boolean
            True if a visual output is wanted, False otherwise
        """
        # np.random.seed(1111) # For Debugging

        #save obstacles and boundaries
        self.obstaclesList = obstaclesList
        self.boundariesList = boundariesList
        self.clearance = clearance  # The minimum distance between the path and all obstacles
        self.maxDistance = maxDistance  # Max distance between each added leaf
        self.maxIncline = maxIncline  # Max slope the vehicle can climb or decend at
        self.maxRelChi = maxRelChi  # Max relative chi between leaves
        self.iterations = iterations  # How many sets of random points it will add each time until solution is found
        self.resolution = resolution  # The segment lengths checked for collisions
        self.scaleHeight = scaleHeight  # Scales the height in the cost function for assigning random points to leaves
        self.animate = animate
        pointList = []  # For boundary points formatted into the Point object for shapely use
        nList = []
        eList = []
        for point in boundariesList:
            nList.append(point.n)
            eList.append(point.e)
            pointList.append(Point(point.n,point.e))
        self.polygon = Polygon([[p.x, p.y] for p in pointList]) #Boundaries now contained in a Polygon object
        self.maxN = max(nList) #Max north position of boundaries
        self.maxE = max(eList) #Max east position of boundaries
        self.minN = min(nList) #Min north position of boundaries
        self.minE = min(eList) #Min east position of boundaries

        if animate:
            mpl.rcParams['legend.fontsize'] = 10
            self.fig = plt.figure()
            self.ax = self.fig.gca(projection='3d')
            for obstacle in obstaclesList:
                # Cylinder
                x = np.linspace((obstacle.n - obstacle.r), (obstacle.n + obstacle.r), 100)
                z = np.linspace(0, obstacle.d, 100)
                # x = np.linspace(-1, 1, 25)
                # z = np.linspace(-2, 2, 25)
                Xc, Zc = np.meshgrid(x, z)
                Yc = np.sqrt(obstacle.r**2 - (Xc - obstacle.n)**2) + obstacle.e

                # Draw parameters
                self.ax.plot_surface(Xc, Yc, Zc, alpha=0.9, color='b')
                self.ax.plot_surface(Xc, (2.*obstacle.e-Yc), Zc, alpha=0.9, color='b')
            first = True
            boundaries = []
            last = []
            for bounds in boundariesList:
                if first:
                    boundaries = np.array([[bounds.n, bounds.e, 0.]])
                    last = np.array([[bounds.n, bounds.e, 0.]])
                    first = False
                    continue
                boundaries = np.append(boundaries, [[bounds.n, bounds.e, 0.]], axis=0)
            boundaries = np.append(boundaries, last, axis=0)
            self.ax.plot(boundaries[:, 0], boundaries[:, 1], boundaries[:, 2], label='Boundaries')
            self.ax.set_xlabel('X axis')
            self.ax.set_ylabel('Y axis')
            self.ax.set_zlabel('Z axis')
            plt.xlim(self.maxN*1.1, self.minN*1.1)
            plt.ylim(self.minE * 1.1, self.maxE * 1.1)
            self.ax.elev = 90 #55
            self.ax.azim = 0 #80
            self.viridis = cm.get_cmap('viridis', 12)
            self.viridis = cm.get_cmap('viridis')
            self.ax.legend()


    def findFullPath(self, waypoints):
        """RRT class function that finds a path to all of the waypoints passed in. This path takes into account obstacles,
        boundaries, and all other parameters set in the init function.

        Parameters
        ----------
        waypoints : msg_ned
            A list of waypoints

        Returns
        -------
        fullPath : msg_ned
            The full list of waypoints which outlines a safe path to follow in order to reach all of the waypoints
            passed in.
        """
        numWaypoints = len(waypoints)
        if self.animate:
            self.wayMax = 0  # This min/max stuff is for making the animated path different colors depending on altitude
            self.wayMin = 0
            if waypoints[0].d > self.wayMax:
                self.wayMax = waypoints[0].d
            if waypoints[0].d < self.wayMin:
                self.wayMin = waypoints[0].d

        fullPath = []
        index = 0
        while index < numWaypoints-1:  # Do each segment of the path individually
            way1 = waypoints[index]
            index2 = index+1
            way2 = waypoints[index2]
            # Check to make sure if the waypoints are possible
            if not collisionCheck(self.obstaclesList,self.polygon,np.array([way1.n]), np.array([way1.e]), np.array([way1.d]), self.clearance):
                index += 1
                continue
            while not collisionCheck(self.obstaclesList, self.polygon, np.array([way2.n]), np.array([way2.e]), np.array([way2.d]), self.clearance):
                if index2 < numWaypoints-1:
                    index2 += 1
                    way2 = waypoints[index2]
                else:
                    break

            if self.animate:
                if way2.d > self.wayMax:
                    self.wayMax = way2.d
                if way2.d < self.wayMin:
                    self.wayMin = way2.d
            newPath = self.findPath(way1, way2)  # call the findPath function to find path between these two waypoints
            fullPath += newPath  # Append this segment of the path to the full path
            index += index2-index
        if self.animate:  # This block of animate code shows the full planned path
            for i in range(0, len(fullPath)-1):
                way1 = fullPath[i]
                way2 = fullPath[i+1]
                if (self.wayMax == self.wayMin):
                    scaler = 1
                else:
                    scaler = (self.wayMin - way2.d) / (self.wayMin - self.wayMax)
                self.ax.plot([way1.n, way2.n], [way1.e, way2.e],[-way1.d, -way2.d], color=self.viridis(scaler))
            plt.gcf()
            plt.gca()
            plt.show()
        return fullPath

    def findPath(self, waypoint1, waypoint2):
        """RRT class function that finds a path between two waypoints passed in. This solved path takes into account obstacles,
        boundaries, and all other parameters set in the init function.

        Parameters
        ----------
        waypoint1 : msg_ned
            The starting waypoint

        waypoint2 : msg_ned
            @param waypoint2: The ending waypoint. Super creative names, I know.

        Returns
        -------
        smoothedPath :  msg_ned
            The list of waypoints which outlines a safe path to follow in order to reach the two waypoints passed in.
        """
        if self.animate: # draws the two waypoints
            begEndPoints = self.ax.scatter([waypoint1.n, waypoint2.n], [waypoint1.e, waypoint2.e],
                                           [-waypoint1.d, -waypoint2.d], c='r', marker='o')
        # node state vector format: N, E, D, cost, parentIndex, connectsToGoalFlag, chi
        startNode = np.array([waypoint1.n, waypoint1.e, waypoint1.d, 0., -1., 0., 8888])
        tree = np.array([startNode])
        # check for if solution at the beginning
        dist = np.sqrt((waypoint1.n-waypoint2.n)**2 + (waypoint1.e-waypoint2.e)**2 + (waypoint1.d-waypoint2.d)**2)
        chi = np.arctan2((waypoint2.e - waypoint1.e), (waypoint2.n - waypoint1.n))
        if dist < self.maxDistance and self.flyablePath(waypoint1, waypoint2, startNode[6], chi):
            return waypoint1, waypoint2  # Returns the two waypoints as the succesful path
        else:
            foundSolution = 0
            while not foundSolution: # This will keep expanding the tree the amount of iterations until solution found
                for i in range(0, self.iterations):
                    tree, flag = self.extendTree(tree, waypoint1, waypoint2)
                    foundSolution += flag

        # # Find the shortest path
        # path = self.shortestPath(tree, waypoint2)
        # # Smooth the path
        # smoothedPath = self.smoothPath(path)
        # return smoothedPath

        # Find complete paths
        connectedPaths = []
        for i in range(0, np.size(tree, 0)):
            if tree[i, 5] == 1:
                connectedNodes = []
                connectedNodes.append(waypoint2)
                connectedNodes.append(msg_ned(tree[i, 0], tree[i, 1], tree[i, 2]))
                parentNode = int(tree[i, 4])
                while parentNode > 0:
                    connectedNodes.append(msg_ned(tree[parentNode,0],tree[parentNode,1],tree[parentNode,2]))
                    parentNode = int(tree[parentNode, 4])
                connectedNodes.append(waypoint1)
                connectedPaths.append(connectedNodes)

        # Smooth all paths and save the best
        bestPath = []
        bestCost = np.inf
        for path in connectedPaths:
            smoothedPath, cost = self.smoothPath(path)
            if cost <  bestCost:
                bestPath = smoothedPath
                bestCost = cost
        return bestPath


    def extendTree(self, tree, startN, endN):
        """RRT class function that extends the passed-in tree. It will continue to attempt adding a leaf until it finds a
        successful one. This is the basic RRT algorithm.

        Parameters
        ----------
        tree : float
            @param tree: An Nx7 array of N leaves in this format: N, E, D, cost, parentIndex, connectsToGoalFlag, chi

        startN : msg_ned
            @param startN: The starting waypoint.

        endN : msg_ned
            The ending waypoint.

        Returns
        -------
        tree:  float
            An Nx7 array of N leaves in this format: N, E, D, cost, parentIndex, connectsToGoalFlag, chi

        int
            Returns a 1 if a path to the end node was found, 0 if not.
        """
        successFlag = False
        while not successFlag:
            # Generate Random Point
            northP, eastP = self.randomPoint()

            # Find nearest leaf. Preference given to leaves that are at the correct altitude
            distances = ((northP-tree[:,0])**2 + (eastP-tree[:,1])**2 + self.scaleHeight*(endN.d - tree[:,2])**2)
            minIndex = np.argmin(distances)  # could loop through a second time to try second best node?? This might help with smoother ascending and descending.
            # Need to find way to get more smooth descents and ascents?? not zig zaggy
            chi = np.arctan2((eastP - tree[minIndex, 1]), (northP - tree[minIndex, 0]))

            # Calculate the new node location
            if(tree[minIndex,2]==endN.d):  # If the chosen leaf is at the ending waypoint altitude
                # A new leaf only extends a maximum distance from a previous leaf
                L = min(np.sqrt((northP-tree[minIndex,0])**2 + (eastP-tree[minIndex,1])**2), self.maxDistance)
                downP = endN.d
                tmp = np.array([northP, eastP, downP]) - np.array([tree[minIndex,0], tree[minIndex,1], tree[minIndex,2]])
                newPoint = np.array([tree[minIndex, 0], tree[minIndex, 1], tree[minIndex, 2]]) + L * (tmp / np.linalg.norm(tmp))
                newNode = np.array([[newPoint.item(0), newPoint.item(1), newPoint.item(2), tree[minIndex, 3] + L, minIndex, 0., chi]])
                # # The following commented lines are for seeing the randomly chosen point
                # if self.animate:
                #     scat = self.ax.scatter([northP, northP], [eastP, eastP], [0, -downP], c='r', marker='+')
                #     scat.remove()
            else:
                # This case is for when the nearest leaf isn't yet at the correct altitude for the ending waypoint
                hyp = np.sqrt((northP-tree[minIndex,0])**2 + (eastP-tree[minIndex,1])**2)
                lessInclineWhilePlanning = .3
                if startN.d > endN.d:
                    downP = tree[minIndex, 2] - hyp * self.maxIncline * lessInclineWhilePlanning
                else:
                    downP = tree[minIndex, 2] + hyp * self.maxIncline * lessInclineWhilePlanning
                q = np.array([northP - tree[minIndex, 0], eastP - tree[minIndex, 1], downP - tree[minIndex, 2]])
                L = np.linalg.norm(q)
                L = min(L, self.maxDistance)
                tmp = np.array([northP, eastP, downP]) - np.array([tree[minIndex,0], tree[minIndex,1], tree[minIndex,2]])
                newPoint = np.array([tree[minIndex,0], tree[minIndex,1], tree[minIndex,2]]) + L*(tmp/np.linalg.norm(tmp))
                if (startN.d > endN.d and newPoint.item(2) < endN.d) or (startN.d < endN.d and newPoint.item(2) > endN.d):  # Check for overshooting the correct altitude
                    newPoint[2] = endN.d
                newNode = np.array([[newPoint.item(0), newPoint.item(1), newPoint.item(2), tree[minIndex,3]+L, minIndex, 0., chi]])
                # # The following commented lines are for seeing the randomly chosen point
                # if self.animate:
                #     scat = self.ax.scatter([northP, northP], [eastP, eastP], [0, -downP], c='r', marker='+')
                #     scat.remove()

            # Check for Collision
            if self.flyablePath(msg_ned(tree[minIndex,0],tree[minIndex,1],tree[minIndex,2]), msg_ned(newNode.item(0),newNode.item(1),newNode.item(2)), tree[minIndex,6] ,chi):
                successFlag = True
                # # The animate lines below draw the fully explored RRT Tree
                # if self.animate:
                #     if(endN.d==startN.d):
                #         scaler = 1
                #     else:
                #         scaler = (endN.d - newNode.item(2))/(endN.d-startN.d)
                #     spider = self.ax.plot([tree[minIndex,0],newNode.item(0)], [tree[minIndex,1],newNode.item(1)], [-tree[minIndex,2],-newNode.item(2)], color=self.viridis(scaler))
                tree = np.append(tree, newNode,axis=0)  # Append new node to the full tree

                # Check to see if the new node can connect to the end node
                dist = np.sqrt((endN.n - newNode.item(0)) ** 2 + (endN.e - newNode.item(1)) ** 2 + (endN.d - newNode.item(2)) ** 2)
                chi = np.arctan2((endN.e - newNode.item(1)), (endN.n - newNode.item(0)))
                if dist < self.maxDistance and self.flyablePath(msg_ned(newNode.item(0), newNode.item(1), newNode.item(2)), endN, newNode.item(6), chi):
                    tree[np.size(tree, 0)-1, 5] = 1
                    return tree, 1  # Return the extended tree with the flag of a successful path to ending node
                else:
                    return tree, 0

    def shortestPath(self, tree, endNode):
        """RRT class function that takes in a tree with successful paths and finds which one is the shortest

        Parameters
        ----------
        tree : float
            An Nx7 array of N leaves in this format: N, E, D, cost, parentIndex, connectsToGoalFlag, chi

        endNode : msg_ned
            The ending waypoint.

        Returns
        -------
        path :  msg_ned
            An array of waypoints that expresses the shortest (but not smoothed), successful path from one waypoint
            to another
        """
        # Find the leaves that connect to the end node
        connectedNodes = []
        for i in range(0, np.size(tree, 0)):
            if tree[i,5] == 1:
                connectedNodes.append(i)

        # Find the path with the shortest distance (could find a different heuristic for choosing which path to go with,
        # especially because we are going to shorten the path anyway??). Choose shortest after smoothing?? Or choose for
        # least turns.
        minIndex = np.argmin(tree[connectedNodes,3])
        minIndex = connectedNodes[minIndex]
        path = []
        path.append(endNode)
        path.append(msg_ned(tree[minIndex,0],tree[minIndex,1],tree[minIndex,2]))
        parentNode = int(tree[minIndex,4])
        while parentNode > 0:
            path.append(msg_ned(tree[parentNode,0],tree[parentNode,1],tree[parentNode,2]))
            parentNode = int(tree[parentNode,4])
        path.append(msg_ned(tree[parentNode, 0], tree[parentNode, 1], tree[parentNode, 2])) #This adds the starting point
        # # The commented lines prints the shortest, but not yet smoothed, path
        # if self.animate:
        #     self.drawPath(path,'r')
        return path

    def smoothPath(self, path):
        """ RRT class function that takes in an array of waypoints and tries to find a flyable, smooth path.

        Parameters
        ----------
        path : msg_ned
            The list of waypoints.

        Returns
        -------
        path : msg_ned
            An array of waypoints that expresses the smoothed, successful path through all the waypoints in reversed
            order.
        """
        # Improve smoother. Because of chi constraint, it doesn't do well at cutting out lots of segments. First try
        # getting all paths before trimming them.
        smoothedPath = [path[0]]
        prev_chi = 8888
        cost = 0
        index = 1
        while index < len(path)-1:
            chi = np.arctan2((path[index+1].e - smoothedPath[len(smoothedPath)-1].e), (path[index+1].n - smoothedPath[len(smoothedPath)-1].n))
            not_last = index + 2 < len(path)  # This is to know if we aren't checking second to last node
            if not_last:  # Have to check the flyability of the node after in addition to current one
                chi2 = np.arctan2((path[index+2].e - path[index+1].e), (path[index+2].n - path[index+1].n))
            if not self.flyablePath(smoothedPath[len(smoothedPath)-1], path[index+1], prev_chi, chi) or \
                    (not_last and not self.flyablePath(path[index+1], path[index+2], chi, chi2)):
                smoothedPath.append(path[index])
                prev_chi = np.arctan2((smoothedPath[len(smoothedPath)-1].e - smoothedPath[len(smoothedPath)-2].e),
                                      (smoothedPath[len(smoothedPath)-1].n - smoothedPath[len(smoothedPath)-2].n))
            index += 1

        smoothedPath.append(path[len(path)-1])
        for i in range(0,len(smoothedPath)-1): #Could add other things to this cost function if wanted
            cost += np.sqrt((smoothedPath[i].n-smoothedPath[i+1].n)**2 + (smoothedPath[i].e-smoothedPath[i+1].e)**2 + \
                            (smoothedPath[i].d-smoothedPath[i+1].d)**2)
        reversePath = smoothedPath[::-1]  # Path was saved in reverse order, so it had to be flipped
        # # Commented lines draw the shortened path
        # if self.animate:
        #     self.drawPath(reversePath, 'y')
        return reversePath, cost

    def randomPoint(self):
        """ RRT class function that creates a random point in the 2d plane bounded by the max and min boundary positions

        Returns
        -------
        pointN : float
            A random north position of a point

        pointE : float
            A random east position of a point
        """
        return np.random.uniform(low=-self.maxN, high=self.maxN), np.random.uniform(low=-self.maxE, high=self.maxE)

    def flyablePath(self, startNode, endNode, prevChi, chi):
        """ RRT class function that checks if flying between two points is possible. It checks for collisions, chi angle,
        and incline.

        Parameters
        ----------
        startNode : msg_ned
            The starting node

        endNode : msg_ned
            The ending node

        prevChi : double
            The chi angle of the leaf being added to

        chi : double
            The chie angle made by added leaf

        Returns
        -------
        boolean
            Returns true if a flyable path, false if not
        """
        #check for obstacles and boundaries
        N, E, D = self.pointsAlongPath(startNode, endNode, self.resolution)
        collisionChecked = collisionCheck(self.obstaclesList,self.polygon, N, E, D, self.clearance)
        if not collisionChecked:
            return False


        #Check for new leaf now above max relative chi angle
        if prevChi != 8888: #If not at the root node
            wrappedPrevChi = self.wrap(prevChi, chi)
            if abs(wrappedPrevChi-chi) > self.maxRelChi:
                return False

        #Check incline here
        incline = np.abs((endNode.d - startNode.d)/np.sqrt((endNode.n - startNode.n) ** 2 + (endNode.e - startNode.e) ** 2) )
        if incline > self.maxIncline+.01:  #Added fudge factor because of floating point math errors
            return False

        return True


    def pointsAlongPath(self, startN, endN, stepSize):
        """ RRT class function that takes two nodes and returns the N, E, and D position of many points along the line
        between the two points spaced according to the step size passed in.

        Parameters
        ----------
        startN : msg_ned
            The starting node

        endN : msg_ned
            The ending node

        stepSize : double
            The desired spacing between each point along the line between the two nodes

        Returns
        -------
        N : double
            An np.array of the north position of points
        E : double
            An np.array of the east position of points
        D : double
            An np.array of the down position of points
        """
        N = np.array([startN.n])
        E = np.array([startN.e])
        D = np.array([startN.d])

        q = np.array([endN.n-startN.n, endN.e-startN.e, endN.d-startN.d])
        L = np.linalg.norm(q)
        q = q / L

        w = np.array([startN.n, startN.e, startN.d])
        for i in range(1, int(np.ceil(L/stepSize))):
            w += stepSize*q
            N = np.append(N, w.item(0))
            E = np.append(E, w.item(1))
            D = np.append(D, w.item(2))
        N = np.append(N, endN.n)
        E = np.append(E, endN.e)
        D = np.append(D, endN.d)
        return N, E, D

    def drawPath(self, path, color):
        """ RRT class function that draws the path between a list of waypoints

        Parameters
        ----------
        path : msg_ned
            List of waypoints

        color : string
            Desired color in format for matplot (e.g. 'r','y','b',etc.)
        """
        for i in range(0, len(path) - 1):
            way1 = path[i]
            way2 = path[i + 1]
            self.ax.plot([way1.n, way2.n], [way1.e, way2.e], [-way1.d, -way2.d], color=color)

    def wrap(self, chi_c, chi):
        """ RRT class function that wraps an angle

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
        while chi_c-chi > np.pi:
            chi_c = chi_c - 2.0 * np.pi
        while chi_c-chi < -np.pi:
            chi_c = chi_c + 2.0 * np.pi
        return chi_c
