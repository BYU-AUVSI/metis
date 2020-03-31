# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg and Kalin Norman

import numpy as np
# import rospy
from matplotlib import pyplot as plt
from metis.messages import msg_ned
# from uav_msgs.msg import JudgeMission, State #,NED_list, NED_pt

# from metis import tools
# from metis.ros import utils

class MissionPlotter:
    """
    A general purpose 2D plotter, capable of drawing mission related features.
    """
    def __init__(self, mission=None):
        """
        Initializes the MissionPlotter object.

        Parameters
        ----------
        mission : metis.core.Mission, optional
            A Mission object for initializing all boundaries and waypoints.
        """
        fig, ax = plt.subplots()
        self.fig = fig
        self.ax = ax
        self.state_counter = 0
        self.state_track_n = []
        self.state_track_e = []
        self.state_track_length = 1000
        self.state_plt = None 
        self.nedWaypoints = None
        self.arrayWaypoints = None
        self.idxToUpdate = None
        self.doubleclick = False
        self.selected = False
        self.mission = mission
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick) # Variable that will allow for event callbacks
        #self.ax.scatter(self.state_track_n, self.state_track_e, label="Plane State", c="red", s=15)
        if self.mission:
            self.add_region(self.mission.search_area, "Search Region", color="orange")
            self.add_region(self.mission.boundary_list, "Boundary")
            self.add_obstacles(self.mission.obstacles, "Obstacles")
            self.add_waypoints([self.mission.drop_location], "Drop Target", color="green", size=25, marker="X")
            self.add_waypoints(self.mission.waypoints,"Objective Waypoints",color="blue",size=12,marker="o",)

    # def initRosNode():
    #     # rospy.Subscriber("/fixedwing/state", State, self.plotState)
    #     rospy.Subscriber("/state", State, self.plotState)


    def add_region(self, regionPoints, label, color='black'):
        """
        Add a region polygon to be drawn.
        
        Parameters
        ----------
        regionPoints : NED_list
            A list of NED points defining the vertices of the polygon
        lable : string
            A name of the region to be used for the legend
        """
        if len(regionPoints) > 0:
            points = self.NEDListToNEnp(regionPoints)
            poly = plt.Polygon(points, closed=True, fill=False, color=color, label=label)
            self.ax.add_artist(poly)
            self.ax.set_xlim((int(1.1*min(points[:,0])), int(1.1*max(points[:,0]))))
            self.ax.set_ylim((int(1.1*min(points[:,1])), int(1.1*max(points[:,1]))))
            self.ax.set_aspect('equal')

    def add_obstacles(self, obstacles, label, color='red'):
        """
        Add circles representing obstacles
        
        Parameters
        ----------
        obstacles
            A list of obstacles to draw
        label : string
            A name of the region to be used for the legend
        """
        for obs in obstacles:
            circ = plt.Circle((obs.e, obs.n), obs.r, fill=False, color=color, label=label)
            self.ax.add_artist(circ)

    def add_waypoints(self, pointList, label, color='green', size=1, marker='.'):
        """
        Add waypoints
        """
        points = self.NEDListToNEnp(pointList)
        self.wapts = self.ax.scatter(points[:,0], points[:,1], label=label, c=color, s=size, marker=marker)
        for i, point in enumerate(pointList):
            self.ax.text(point.e, point.n, str(i))

    def add_pathway(self, pointList, label, ptColor='red', pathColor='cyan'):
        self.nedWaypoints = pointList
        self.add_waypoints(pointList, label, color=ptColor, size=6, marker='x')
        points = self.NEDListToNEnp(pointList)
        self.arrayWaypoints = points
        self.pths = self.ax.plot(points[:,0], points[:,1], c=pathColor)

    def plotState(self, msg):
        self.state_counter += 1
        if self.state_counter < 50:
            return
        self.state_counter = 0

        state_n = msg.position[0]
        state_e = msg.position[1]
        self.state_track_n.append(state_n)
        self.state_track_e.append(state_e)
        if len(self.state_track_n) > self.state_track_length:
            del self.state_track_n[0]
            del self.state_track_e[0]

        if self.state_plt is not None:
            self.state_plt.remove()
        self.state_plt = self.ax.scatter(self.state_track_n, self.state_track_e, label="Plane State", c="red", s=15)

        self.show()


    def show(self, block=True):
        plt.legend()
        plt.show(block=block)

    def updatePlot(self):
        self.selection.remove() # Remove circle around selected point
        del self.ax.lines[0] # Remove lines from plot
        self.ax.collections.remove(self.wapts) # Remove scatterplot points from plot
        self.add_pathway(self.nedWaypoints, "Planned pathway") # Add new lines and scatterplot to plot with updated waypoints
        self.fig.show() # Show the plot
    
    def showSelected(self):
        circ = plt.Circle(self.arrayWaypoints[self.idxToUpdate], 15, fill=False, color="orange")
        self.selection = self.ax.add_artist(circ)
        self.fig.show()
    
    def removeSelected(self):
        self.selection.remove()
        self.show()

    def onclick(self, event):
        if self.doubleclick:
            if event.button == 1: # Left click
                self.doubleclick = False
                self.removeSelected()
            elif event.button == 2: # Middle click
                self.doubleclick = False
                self.updateNEDList(event, remove=True)
                self.updatePlot()
            elif event.button == 3: # Right click
                self.doubleclick = False
                self.idxToUpdate += 1
                self.updateNEDList(event, add=True)
                self.updatePlot()
        elif event.dblclick:
            self.doubleclick = True
            diffx = event.xdata - self.arrayWaypoints[:,0]
            diffy = event.ydata - self.arrayWaypoints[:,1]
            dist = diffx * diffx + diffy * diffy
            self.idxToUpdate = np.argmin(dist)
            self.showSelected()
        elif event.button == 3 and self.selected:
            self.selected = False
            self.removeSelected()
        elif event.button == 3:
            self.selected = True
            diffx = event.xdata - self.arrayWaypoints[:,0]
            diffy = event.ydata - self.arrayWaypoints[:,1]
            dist = diffx * diffx + diffy * diffy
            self.idxToUpdate = np.argmin(dist)
            self.showSelected()
        elif event.button == 1 and self.selected:
            self.selected = False
            self.arrayWaypoints[self.idxToUpdate] = [event.xdata, event.ydata]
            self.updateNEDList(event, update=True)
            self.updatePlot()


    def NEDListToNEnp(self, pointList):
        """
        Convert a list of NED points to a nx2 numpy array with col 1 = E and col 2 = N
        """
        npPoints = np.empty((0,2))
        for nedPoint in pointList:
            nePt = np.array([[nedPoint.e, nedPoint.n]])
            npPoints = np.append(npPoints, nePt, axis=0)

        return npPoints

    def updateNEDList(self, eventdata, update=False, remove=False, add=False):
        """
        Convert an nx2 numpy array with col 1 = E and col 2 = N to a list of NED points
        """
        if update:
            self.nedWaypoints[self.idxToUpdate].e = eventdata.xdata
            self.nedWaypoints[self.idxToUpdate].n = eventdata.ydata
        elif add:
            e = eventdata.xdata
            n = eventdata.ydata
            d = -0.0
            r = 0
            if self.idxToUpdate >= len(self.nedWaypoints):
                print(self.nedWaypoints)
                d = self.nedWaypoints[self.idxToUpdate-1].d
                self.nedWaypoints.append(msg_ned(n, e, d, r))
                print(self.nedWaypoints)
            else:
                a = self.nedWaypoints[self.idxToUpdate-1].to_nparray()
                b = self.nedWaypoints[self.idxToUpdate].to_nparray()
                self.nedWaypoints.insert(self.idxToUpdate,msg_ned(n, e, d, r))
                cur = self.nedWaypoints[self.idxToUpdate].to_nparray()
                dista = np.linalg.norm(cur[0:1]-a[0:1])
                distb = np.linalg.norm(cur[0:1]-b[0:1])
                distpercentage = dista / (dista+distb)
                d = a[2] - ((a[2]-b[2]) * distpercentage)
                self.nedWaypoints[self.idxToUpdate].d = d
        elif remove:
            del self.nedWaypoints[self.idxToUpdate]
        else:
            pass
        
    def getNEDlist(self):
        return self.nedWaypoints



# if __name__ == "__main__":
#     rospy.init_node("missionPlotter")

#     #Get ref lat, lon from launch file
#     ref_lat = rospy.get_param("ref_lat")
#     ref_lon = rospy.get_param("ref_lon")
#     ref_h = rospy.get_param("ref_h")
#     ref_pos = [ref_lat, ref_lon, ref_h]
#     mission_type, obstacles, boundary_list, boundary_poly, drop_location = utils.get_server_data(JudgeMission.MISSION_TYPE_DROP, ref_pos)
#     _, _, _, _, objective_waypts = utils.get_server_data(JudgeMission.MISSION_TYPE_WAYPOINT, ref_pos)
#     _, _, _, _, search_boundary = utils.get_server_data(JudgeMission.MISSION_TYPE_SEARCH, ref_pos)

#     print("Obstacles")
#     for obstacle in obstacles:
#         print(obstacle.n, obstacle.e, obstacle.d, obstacle.r)
#     print("Boundaries")
#     for boundary in boundary_list:
#         print(boundary.n, boundary.e)
#     print("Drop")
#     for drop in drop_location:
#         print(drop.n, drop.e, drop.d)

#     mp = MissionPlotter()
#     mp.addRegion(search_boundary, "Search Region", color='orange')
#     mp.addRegion(boundary_list, "Boundary")
#     mp.addObstacles(obstacles, "Obstacles")
#     mp.addWaypoints(drop_location, "Drop Target", color='green', size=25, marker='X')
#     mp.addWaypoints(objective_waypts, "Objective Waypoints", color='blue', size=12, marker='o')
#     mp.show()

#     rospy.spin()