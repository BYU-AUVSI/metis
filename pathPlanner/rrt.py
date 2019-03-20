import time
import numpy as np
import random
import math
import os
# import pyqtgraph as pg
# import pyqtgraph.opengl as gl
# import pyqtgraph.Vector as Vector
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
from matplotlib.colors import ListedColormap, LinearSegmentedColormap
from matplotlib import cm

import sys
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
sys.path.append('..')
from messages.ned import msg_ned





class RRT():

    def __init__(self, obstaclesList, boundariesList, animate):
        #save obstacles and boundaries
        self.obstaclesList = obstaclesList
        self.boundariesList = boundariesList
        self.clearance = 1. #The minimum distance between the path and all obstacles
        self.travelDistance = 10. #Max distance between each added leaf
        self.maxIncline = .5 #Max slope the vehicle can climb or decend at
        self.maxRelChi = np.pi/2 #Max relative chi between leaves
        self.iterations = 50 #How many sets of random points it will add each time until solution is found
        self.animate = animate
        pointList = []
        nList = []
        eList = []
        for point in boundariesList:
            nList.append(point.n)
            eList.append(point.e)
            pointList.append(Point(point.n,point.e))
        self.polygon = Polygon([[p.x, p.y] for p in pointList])
        self.maxN = max(nList)
        self.maxE = max(eList)
        self.minN = min(nList)
        self.minE = min(eList)

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
            self.ax.elev = 90 #55
            self.ax.azim = 0 #80
            self.viridis = cm.get_cmap('viridis', 12)
            self.ax.legend()

    def findFullPath(self, waypoints):
        numWaypoints = len(waypoints)
        if self.animate:
            self.wayMax = 0
            self.wayMin = 0
            if waypoints[0].d > self.wayMax:
                self.wayMax = waypoints[0].d
            if waypoints[0].d < self.wayMin:
                self.wayMin = waypoints[0].d

        fullPath = []
        if self.animate:
            self.update_plot() #How pause and view the graph??
        for i in range(0, numWaypoints-1): #Do each segment of the path individually
            way1 = waypoints[i]
            way2 = waypoints[i+1]
            #Check to make sure if the waypoint is possible??
            if self.animate:
                if way2.d > self.wayMax:
                    self.wayMax = way2.d
                if way2.d < self.wayMin:
                    self.wayMin = way2.d
            newPath = self.findPath(way1, way2)
            fullPath += newPath
        if self.animate:
            for i in range(0, len(fullPath)-1):
                way1 = fullPath[i]
                way2 = fullPath[i+1]
                if (self.wayMax == self.wayMin):
                    scaler = 1
                else:
                    scaler = (self.wayMin - way2.d) / (self.wayMin - self.wayMax)
                self.ax.plot([way1.n, way2.n], [way1.e, way2.e],[-way1.d, -way2.d], color=self.viridis(scaler))
        return fullPath

    def findPath(self, waypoint1, waypoint2):
        if self.animate:
            begEndPoints = self.ax.scatter([waypoint1.n, waypoint2.n], [waypoint1.e, waypoint2.e], [-waypoint1.d, -waypoint2.d], c='r', marker='o')
        #node state N, E, D, cost, parentIndex, connectsToGoalFlag, chi
        startNode = np.array([waypoint1.n, waypoint1.e, waypoint1.d, 0., -1., 0., ])
        tree = np.array([startNode])
        #check for if solution at the beginning
        dist = np.sqrt((waypoint1.n-waypoint2.n)**2 + (waypoint1.e-waypoint2.e)**2 + (waypoint1.d-waypoint2.d)**2)
        incline = np.abs((waypoint2.d-waypoint1.d)/np.sqrt((waypoint2.n-waypoint1.n)**2 + (waypoint2.e-waypoint1.e)**2))
        if dist < self.travelDistance and self.flyablePath(waypoint1, waypoint2, self.maxIncline) and incline < self.maxIncline:
            return waypoint1, waypoint2
        else:
            foundSolution = 0
            while not foundSolution:
                for i in range(0, self.iterations):
                    tree, flag = self.extendTree(tree, waypoint1, waypoint2, self.travelDistance, self.maxIncline)
                    foundSolution += flag

        #Find the shortest path
        path = self.shortestPath(tree, waypoint2)
        #Smooth the path
        smoothedPath = self.smoothPath(path, self.maxIncline)
        return smoothedPath
            # closestConfig = self.nearestNode(rrtGraph, np, ep, rrtGraph, np.sqrt((rrtGraph.n-np)**2 + (rrtGraph.e-ep)**2) + (rrtGraph.d-waypoint2.d)**2)

    def extendTree(self, tree, startN, endN, maxSegLength, maxIncline):
        successFlag = False
        while not successFlag:
            #Generate Random Point
            northP, eastP = self.randomPoint()


            #Find nearest leaf
            #Add preference to giving priority to leaves that are at the correct altitude
            scaleHeight = 1.5
            distances = ((northP-tree[:,0])**2 + (eastP-tree[:,1])**2 + scaleHeight*(endN.d - tree[:,2])**2)
            minIndex = np.argmin(distances) ##could loop through a second time to try second best node??

            if(tree[minIndex,2]==endN.d):
                L = min(np.sqrt((northP-tree[minIndex,0])**2 + (eastP-tree[minIndex,1])**2), maxSegLength)
                downP = endN.d
                tmp = np.array([northP, eastP, downP]) - np.array([tree[minIndex,0], tree[minIndex,1], tree[minIndex,2]])
                newPoint = np.array([tree[minIndex, 0], tree[minIndex, 1], tree[minIndex, 2]]) + L * (tmp / np.linalg.norm(tmp))
                newNode = np.array([[newPoint.item(0), newPoint.item(1), newPoint.item(2), tree[minIndex, 3] + L, minIndex, 0.]])
                # scat = self.ax.scatter([northP, northP,newNode.item(0)], [eastP, eastP,newNode.item(1)], [0, -downP,-newNode.item(2)], c = 'r', marker = '+')
                # scat.remove()
            else:
                hyp = np.sqrt((northP-tree[minIndex,0])**2 + (eastP-tree[minIndex,1])**2)
                downP = tree[minIndex,2] - hyp*maxIncline
                # tester = (northP - tree[minIndex, 0])
                q = np.array([northP - tree[minIndex, 0], eastP - tree[minIndex, 1], downP - tree[minIndex, 2]])
                L = np.linalg.norm(q)
                q = q / L
                L = min(L, maxSegLength)
                tmp = np.array([northP, eastP, downP]) - np.array([tree[minIndex,0], tree[minIndex,1], tree[minIndex,2]])
                newPoint = np.array([tree[minIndex,0], tree[minIndex,1], tree[minIndex,2]]) + L*(tmp/np.linalg.norm(tmp))
                if newPoint.item(2) < endN.d:
                    newPoint[2] = endN.d
                newNode = np.array([[newPoint.item(0), newPoint.item(1), newPoint.item(2), tree[minIndex,3]+L,minIndex, 0.]])
                # if self.animate:
                #     scat = self.ax.scatter([northP, northP], [eastP, eastP], [0, -downP], c='r', marker='+')
                #     scat.remove()

            #Check for Collision
            if self.flyablePath(msg_ned(tree[minIndex,0],tree[minIndex,1],tree[minIndex,2]), msg_ned(newNode.item(0),newNode.item(1),newNode.item(2)),maxIncline):
                successFlag = True
                if(endN.d==startN.d):
                    scaler = 1
                else:
                    scaler = (endN.d - newNode.item(2))/(endN.d-startN.d)
                #The lines below draw the fully explored RRT Tree
                # if self.animate:
                #     spider = self.ax.plot([tree[minIndex,0],newNode.item(0)], [tree[minIndex,1],newNode.item(1)], [-tree[minIndex,2],-newNode.item(2)], color=self.viridis(scaler))
                tree = np.append(tree, newNode,axis=0)

            #Check to see if can connect to end node
            dist = np.sqrt((endN.n - newNode.item(0)) ** 2 + (endN.e - newNode.item(1)) ** 2 + (endN.d - newNode.item(2)) ** 2)
            if(endN.d == newNode.item(2)):
                incline = 0
            else:
                incline = np.abs((endN.d - newNode.item(2))/np.sqrt((endN.n - newNode.item(0)) ** 2 + (endN.e - newNode.item(1)) ** 2) )
            if dist < maxSegLength and self.flyablePath(msg_ned(newNode.item(0), newNode.item(1), newNode.item(2)), endN,maxIncline) and incline < maxIncline:
                tree[np.size(tree, 0)-1, 5] = 1
                return tree, 1
            else:
                return tree, 0

    def shortestPath(self, tree, waypoint2):
        #find the leaves that connect to the end node
        connectedNodes = []
        for i in range(0, np.size(tree, 0)):
            if tree[i,5]==1:
                # connectedNodes.append(tree[i,:])
                # added = np.reshape(tree[i,:], (1,6))
                # connectedNodes = np.append(connectedNodes, added, axis=0)
                connectedNodes.append(i)

        #find the path with the shortest distance
        minIndex = np.argmin(tree[connectedNodes,3])
        minIndex = connectedNodes[minIndex]
        path = []
        path.append(waypoint2)
        path.append(msg_ned(tree[minIndex,0],tree[minIndex,1],tree[minIndex,2]))
        parentNode = int(tree[minIndex,4])
        while parentNode > 0:
            path.append(msg_ned(tree[parentNode,0],tree[parentNode,1],tree[parentNode,2]))
            parentNode = int(tree[parentNode,4])
        path.append(msg_ned(tree[parentNode, 0], tree[parentNode, 1], tree[parentNode, 2])) #This adds the starting point
        # if self.animate:
        #     self.drawPath(path,'r')
        return path

    def smoothPath(self, path, maxIncline):
        smoothedPath = [path[0]]
        index = 1
        while index < len(path)-1:
            if not self.flyablePath(smoothedPath[len(smoothedPath)-1],path[index+1],maxIncline):
                smoothedPath.append(path[index])
            index += 1

        smoothedPath.append(path[len(path)-1])
        #remove the first node
        # smoothedPath = smoothedPath[1:len(smoothedPath)]
        reversePath = smoothedPath[::-1]
        # if self.animate:
        #     self.drawPath(reversePath, 'y')
        return reversePath


    # def foundSolution(self, node, waypoint2):
    #     if(waypoint2.n == node.n and waypoint2.e == node.e and waypoint2.d == node.d):
    #         return True
    #     else:
    #         for item in node.child:
    #             if self.foundSolution(item, waypoint2):
    #                 return True
    #         return False

    def randomPoint(self):
        return np.random.uniform(low=-self.maxN, high=self.maxN), np.random.uniform(low=-self.maxE, high=self.maxE)

    # def nearestNode(self, node, northP, eastP, bestNode, bestDistance):
    #     if(node.child.isempty()):
    #         return bestNode, bestDistance
    #     else:
    #         for item in node.child:
    #             rNode, rDist = self.nearestNode(item,northP,eastP,bestNode, bestDistance)
    #             if rDist < bestDistance:
    #                 bestNode = rNode#How do reference?
    #                 bestDistance = rDist
    #         return bestNode, bestDistance

    def flyablePath(self, startNode, endNode,maxIncline):
        #check for obstacles
        p1 = np.array([startNode.n, startNode.e])
        p2 = np.array([endNode.n, endNode.e])
        X, Y, Z = self.pointsAlongPath(startNode, endNode, 1.1)

        for obstacle in self.obstaclesList:
            #first check if path is above obstacle
            if (all(Z < -obstacle.d-self.clearance)):
                continue
            #then check if runs into obstacle
            else:
                # p3 = np.array([obstacle.n, obstacle.e])
                # distToLine = np.linalg.norm(np.cross(p2 - p1, p1 - p3)) / np.linalg.norm(p2 - p1)
                distToLine = np.sqrt((X-obstacle.n)**2 + (Y-obstacle.e)**2)
                if(any(distToLine < obstacle.r + self.clearance)):
                    return False

        #Should I check for flyablility?? E.g. I can't do a 180 degree flip around.

        #Could check incline here
        incline = np.abs((endNode.d - startNode.d)/np.sqrt((endNode.n - startNode.n) ** 2 + (endNode.e - startNode.e) ** 2) )
        if incline > maxIncline+.01: #Added fudge factor because of floating point math errors
            return False
        #check for boundaries
            # point = Point(-90, 40)
            # self.ax.scatter(-90, 40, 0, s=20, c=np.array([1, 0, 0]))
            # plt.show()
        for i in range(0,len(X)):
            if not self.polygon.contains(Point(X[i], Y[i])):
                return False
        return True


    def pointsAlongPath(self, startN, endN, stepSize):
        X = np.array([startN.n])
        Y = np.array([startN.e])
        Z = np.array([startN.d])

        q = np.array([endN.n-startN.n, endN.e-startN.e, endN.d-startN.d])
        L = np.linalg.norm(q)
        q = q / L

        w = np.array([startN.n, startN.e, startN.d])
        for i in range(1, int(np.ceil(L/stepSize))):
            w += stepSize*q
            X = np.append(X, w.item(0))
            Y = np.append(Y, w.item(1))
            Z = np.append(Z, w.item(2))
        X = np.append(X, endN.n)
        Y = np.append(Y, endN.e)
        Z = np.append(Z, endN.d)
        return X, Y, Z

    def drawPath(self, path, color):
        for i in range(0, len(path) - 1):
            way1 = path[i]
            way2 = path[i + 1]
            self.ax.plot([way1.n, way2.n], [way1.e, way2.e], [-way1.d, -way2.d], color=color)


    def update_plot(self):
        """
        Update the drawing. , path, state

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.pn  # north position
            state.pe  # east position
            state.h   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        # mav_position = np.array([[state.pn], [state.pe], [-state.h]])  # NED coordinates
        # # attitude of mav as a rotation matrix R from body to inertial
        # R = Euler2Rotation(state.phi, state.theta, state.psi)
        # # rotate and translate points defining mav
        # rotated_points = self._rotate_points(self.points, R.T) #Whay had to transpose??
        # translated_points = self._translate_points(rotated_points, mav_position)
        # # convert North-East Down to East-North-Up for rendering
        # R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        # translated_points = R @ translated_points
        # # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        # mesh = self._points_to_mesh(translated_points)


        path = np.array([[3,3,9], #Nx3
                        [8,7,7],
                        [10, 10, 9]])
        # self.ax.plot(path[0,:], path[1,:], path[2,:], label='blah')
        # self.ax.scatter(path[0,:], path[1,:], path[2,:], s=20,c=np.array([1, 0, 0]))
        # self.ax.legend()
        # plt.show()

        # initialize the drawing the first time update() is called
        # if not self.plot_initialized:
        #     # if path.flag=='line':
        #     straight_line_object = self.straight_line_plot(path)
        #     self.window.addItem(straight_line_object)  # add straight line to plot
        #     # else:  # path.flag=='orbit
        #     #     orbit_object = self.orbit_plot(path)
        #     #     self.window.addItem(orbit_object)
        #     # initialize drawing of triangular mesh.
        #     # self.body = gl.GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
        #     #                           vertexColors=self.meshColors, # defines mesh colors (Nx1)
        #     #                           drawEdges=True,  # draw edges between mesh elements
        #     #                           smooth=False,  # speeds up rendering
        #     #                           computeNormals=False)  # speeds up rendering
        #     # self.window.addItem(self.body)  # add body to plot
        #     self.plot_initialized = True
        #
        # # else update drawing on all other calls to update()
        # # else:
        #     # reset mesh using rotated and translated points
        #     # self.body.setMeshData(vertexes=mesh, vertexColors=self.meshColors)
        #
        # # update the center of the camera view to the mav location
        # #view_location = Vector(state.pe, state.pn, state.h)  # defined in ENU coordinates
        # #self.window.opts['center'] = view_location
        # # redraw
        # self.app.processEvents()

    # def straight_line_plot(self, points):
    #     # points = np.array([[path.line_origin.item(0),
    #     #                     path.line_origin.item(1),
    #     #                     path.line_origin.item(2)],
    #     #                    [path.line_origin.item(0) + self.scale * path.line_direction.item(0),
    #     #                     path.line_origin.item(1) + self.scale * path.line_direction.item(1),
    #     #                     path.line_origin.item(2) + self.scale * path.line_direction.item(2)]])
    #     # # convert North-East Down to East-North-Up for rendering
    #     # R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    #     # points = points @ R.T
    #     red = np.array([[1., 0., 0., 1]])
    #     path_color = np.concatenate((red, red), axis=0)
    #     object = gl.GLLinePlotItem(pos=points,
    #                                color=path_color,
    #                                width=2,
    #                                antialias=True,
    #                                mode='lines')
    #     return object

class rrtNode():

    def __init__(self, parent, cost, pn, pe, pd):
        self.parent = parent
        if parent == 'Parent':
            self.cost = 0.
        else:
            self.cost = parent.cost + cost
        self.n = pn
        self.e = pe
        self.d = pd
        self.child = []

    def addChild(self, newNode):
        self.child.append(newNode)
