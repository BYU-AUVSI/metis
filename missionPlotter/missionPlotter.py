#!/usr/bin/python
# a general purpose 2D plotter, capable of drawing mission related features


import sys

import rospy
import rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('metis'))

import numpy as np
from matplotlib import pyplot as plt
from uav_msgs.msg import JudgeMission, NED_pt, NED_list, State
from tools import tools

class MissionPlotter:
    def __init__(self):
        fig, ax = plt.subplots()
        self.fig = fig
        self.ax = ax
        self.state_counter = 0
        self.state_track_n = []
        self.state_track_e = []
        self.state_track_length = 1000
        self.state_plt =  None #self.ax.scatter(self.state_track_n, self.state_track_e, label="Plane State", c="red", s=15)
        rospy.Subscriber("/fixedwing/state", State, self.plotState)


    def addRegion(self, regionPoints, label, color='black'):
        """
        Add a region polygon to be drawn
        ---------
        Parameters

        regionPoints : NED_list
        A list of NED points defining the vertices of the polygon

        lable : string
        A name of the region to be used for the legend
        """
        points = self.NEDListToNEnp(regionPoints)
        poly = plt.Polygon(points, closed=True, fill=False, color=color, label=label)
        self.ax.add_artist(poly)
        self.ax.set_xlim((int(1.1*min(points[:,0])), int(1.1*max(points[:,0]))))
        self.ax.set_ylim((int(1.1*min(points[:,1])), int(1.1*max(points[:,1]))))
        self.ax.set_aspect('equal')

    def addObstacles(self, obstacles, label, color='red'):
        """
        Add circles representing obstacles
        ---------
        Parameters

        obstacles
        A list of obstacles to draw

        label : string
        A name of the region to be used for the legend
        """
        for obs in obstacles:
            circ = plt.Circle((obs.e, obs.n), obs.r, fill=False, color=color, label=label)
            self.ax.add_artist(circ)

    def addWaypoints(self, pointList, label, color='green', size=1, marker='.'):
        """
        Add waypoints
        """
        points = self.NEDListToNEnp(pointList)
        self.ax.scatter(points[:,0], points[:,1], label=label, c=color, s=size, marker=marker);


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


    def show(self):
        plt.legend()
        plt.show()



    def NEDListToNEnp(self, pointList):
        """
        Convert a list of NED points to a nx2 numpy array with col 1 = E and col 2 = N
        """
        npPoints = np.empty((0,2))
        for nedPoint in pointList:
            nePt = np.array([[nedPoint.e, nedPoint.n]])
            npPoints = np.append(npPoints, nePt, axis=0)

        return npPoints




if __name__ == "__main__":
    rospy.init_node("missionPlotter")

    #Get ref lat, lon from launch file
    ref_lat = rospy.get_param("ref_lat")
    ref_lon = rospy.get_param("ref_lon")
    ref_h = rospy.get_param("ref_h")
    ref_pos = [ref_lat, ref_lon, ref_h]
    mission_type, obstacles, boundary_list, boundary_poly, drop_location = tools.get_server_data(JudgeMission.MISSION_TYPE_DROP, ref_pos)
    _, _, _, _, objective_waypts = tools.get_server_data(JudgeMission.MISSION_TYPE_WAYPOINT, ref_pos)
    _, _, _, _, search_boundary = tools.get_server_data(JudgeMission.MISSION_TYPE_SEARCH, ref_pos)

    print("Obstacles")
    for obstacle in obstacles:
        print(obstacle.n, obstacle.e, obstacle.d, obstacle.r)
    print("Boundaries")
    for boundary in boundary_list:
        print(boundary.n, boundary.e)
    print("Drop")
    for drop in drop_location:
        print(drop.n, drop.e, drop.d)

    mp = MissionPlotter()
    mp.addRegion(search_boundary, "Search Region", color='orange')
    mp.addRegion(boundary_list, "Boundary")
    mp.addObstacles(obstacles, "Obstacles")
    mp.addWaypoints(drop_location, "Drop Target", color='green', size=25, marker='X')
    mp.addWaypoints(objective_waypts, "Objective Waypoints", color='blue', size=12, marker='o')
    mp.show()

    rospy.spin()
