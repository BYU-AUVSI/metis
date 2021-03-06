import sys
sys.path.append('..')


import numpy as np
import matplotlib.pyplot as plt
# from matplotlib.patches import Polygon
#from matplotlib.patches import PatchCollection
from tools.tools import collisionCheck, makeBoundaryPoly, convert
from messages.ned import msg_ned


class PayloadPlanner():
    """
    inputs: drop gps location, wind, obstacles
    outputs: X number of waypoints with a single drop waypoint
    """
    def __init__(self,dropLocation,obstacles,boundariesList,boundariesPolygon,wind=np.array([0.,0.,0.])):
        """
        initializes global variables

        Parameters
        ----------
        dropLocation : NED class
            The goal drop location
        obstacles : list of NED classes
            static obastacles
        boundaries : polygon
            polygon defined by boundaries
        wind : list
            current estimate for the wind vector in [north, east, down]
        """
        self.dropLocation = dropLocation            # location of where on the ground we want to hit [N, E, D]
        self.wind = wind                            # current wind vector [Wn,We,Wd]
        self.obstacles = obstacles                  # competition obstacles
        self.boundariesPolygon = boundariesPolygon                # polygon of competition boundaries
        self.boundariesList = boundariesList       # This is only used once it seems, to plot the boundaries. Could we change it so we get the plotting points from the polygon? -JTA, 5/3/2019
        self.drop_altitude = 34.0                   # altitude for waypoints in meters above 0.0 of ground station
        self.time_delay = 2.5                       # seconds between command to open and baydoor opening
        self.time_to_open_parachute = 1.61          # seconds between baydoor opening and parachute opening
        self.ff = 1.0                               # fudge factor for closed parachute drag
        self.terminal_velocity = 2.28               # from experimental data [m/s]
        self.Va = 17.0  # pull from yaml            # competition design Va [m/s]
        self.gravity = 9.8147 # pull from yaml      # acceleration of gravity [m/s**2]
        self.course_command = 0.0                   # initialize course commmand
        self.NED_parachute_open = np.array([0.0,0.0,0.0])   # location where the parachute oepns [N, E, D]
        self.NED_release_location = np.array([0.0,0.0,0.0]) # location where the command to relase should be given [N, E, D]
        self.waypoints = np.zeros((1,3))            # flight waypoints
        self.waypoints_array = np.zeros((1,3))      # numpy list of flight waypoints
        self.waypoint_spread = 25.0                 # distance between supporting waypoints in meters
        self.supporting_points = 2                  # number of supporting waypoints on either side of drop waypoint
        self.chi_offset = 0.0                       # offset for commanded chi calculation if waypoint is inside an obstacle
        self.ii = 0

    def plan(self,wind=np.array([0.,0.,0.])):

        """
        function called by mainplanner that returns bombdrop waypoint

        Parameters
        ----------
        wind : list
            current estimate for the wind vector in [north, east, down]

        Returns
        -------
        waypoints : list of NED class objects
            A list of NED class objects where each object describes the NED position of each waypoint
        """
        self.wind = wind

        # Debugging
        print("wind = ",self.wind)
        print("drop location = ",self.dropLocation.n,",",self.dropLocation.e,",",self.dropLocation.d)



        flag = True
        # keep creating release locations while the location is inside an obstacle or out of bounds
        while not(self.validateWaypoints()) or flag:
            flag = False
            self.calcCourseCommand(self.chi_offset)
            self.displacement0_1 = self.calcClosedParachuteDrop()
            self.displacement1_2 = self.calcOpenParachuteDrop()
            self.calcReleaseLocation(self.displacement0_1,self.displacement1_2)
            self.waypoints = self.calcSupportingPoints()
            self.chi_offset += np.radians(15.)
            #print("planned for angle of ",self.chi_offset)
        drop_output = [self.NED_release_location.item(0),self.NED_release_location.item(1),self.NED_release_location.item(2)]
        #print("drop output =",drop_output)
        return self.waypoints, drop_output

    def calcClosedParachuteDrop(self):
        """
        Calculates the motion between the commanded relase and the parachute opening

        Returns
        -------
        displacement0_1 : numpy array
            array that includes the displacement of the closed parachute drop in the north and east directions
        """
        V0_north = self.Va*np.cos(self.course_command) + self.wind.item(0)  # initial north velocity in inertial frame
        V0_east = self.Va*np.sin(self.course_command) + self.wind.item(1)   # initial east velocity in inertial frame
        V0_down = self.wind.item(2)                                         # initial down velocity in inertial frame
        time0_1 = self.time_delay + self.time_to_open_parachute             # time between command and the parachute opening
        dNorth0_1 = self.ff*V0_north*time0_1                                        # time 0 to 1 north difference in inertial frame
        dEast0_1 = self.ff*V0_east*time0_1                                          # time 0 to 1 east difference in inertial frame
        dDown0_1 = 0.5*self.gravity*self.time_to_open_parachute**2             # time 0 to 1 down difference in inertial frame (Xf = X0 + V0*t + 0.5*a*t**2)
        self.NED_parachute_open[2] = -self.drop_altitude + dDown0_1         # Down position at parachute opening
        return np.array([dNorth0_1,dEast0_1])

    def calcOpenParachuteDrop(self):
        """
        Calculates the motion between the parachute opening and hitting the ground target

        Returns
        -------
        displacement1_2 : numpy array
            array that includes the displacement of the open parachute drop in the north and east directions
        """
        target_north = self.dropLocation.n    # target north location
        target_east = self.dropLocation.e     # target east location
        target_down = self.dropLocation.d   # target down location
        time1_2 = (target_down - self.NED_parachute_open.item(2))/self.terminal_velocity  # calculate time from parahute opening to hitting the target (Xf - X0 = v*t)
        dNorth1_2 = self.wind.item(0)*time1_2       # time 1 to 2 north difference in inertial frame
        dEast1_2 = self.wind.item(1)*time1_2        # time 1 to 2 east difference in inertial frame
        return np.array([dNorth1_2,dEast1_2])

    def calcReleaseLocation(self,displacement0_1,displacement1_2):
        """
        calculates desired location for the release command

        Parameters
        ----------
        displacement0_1 : numpy array
            array that includes the displacement of the closed parachute drop in the north and east directions

        displacement1_2 : numpy array
            array that includes the displacement of the open parachute drop in the north and east directions

        """
        dNorth0_1 = displacement0_1.item(0)                                 # region 1 north displacement
        dEast0_1 = displacement0_1.item(1)                                  # region 1 east displacement
        dNorth1_2 = displacement1_2.item(0)                                 # region 2 north displacement
        dEast1_2 = displacement1_2.item(1)                                  # region 2 east displacement
        release_north = self.dropLocation.n - dNorth1_2 - dNorth0_1   # release north position
        release_east = self.dropLocation.e - dEast1_2 - dEast0_1      # release east position
        release_down = -self.drop_altitude                                  # release down position
        self.NED_release_location = np.array([release_north,release_east,release_down])

    def calcCourseCommand(self,offset):
        """
        calculates the command course angle to be directly into the wind

        Parameters
        ----------
        offset : float
            offset for commanded chi calculation if waypoint is inside an obstacle
        """
        wind_north = self.wind.item(0)
        wind_east = self.wind.item(1)
        psi_wind = np.arctan2(wind_east,wind_north)
        course_command = np.arctan2(-wind_east,-wind_north)
        self.course_command = course_command + offset

    def calcSupportingPoints(self):
        """
        given a command release location plot X points for a straight line of deployment run

        Returns
        -------
        waypoints : list of NED class objects
            A list of NED class objects where each object describes the NED position of each waypoint
        """
        length = self.supporting_points*2 + 1
        self.waypoints_array = np.zeros((length,3))
        # North and East difference at course angle for given distance between waypoints
        dNorth = self.waypoint_spread*np.cos(self.course_command)
        dEast = self.waypoint_spread*np.sin(self.course_command)
        # create waypoints before the release location
        for ii in range(self.supporting_points):
            self.waypoints_array[ii] = self.NED_release_location - (self.supporting_points-ii)*np.array([dNorth,dEast,0.0])
        # add the release location waypoint
        self.waypoints_array[self.supporting_points] = self.NED_release_location
        # create waypoints after the release location
        for ii in range(self.supporting_points):
            self.waypoints_array[ii+self.supporting_points+1] = self.NED_release_location + (ii+1.0)*np.array([dNorth,dEast,0.0])
        waypointsList = []
        for ii in range(length):
            waypointsList.append(msg_ned(self.waypoints_array[ii,0],self.waypoints_array[ii,1],self.waypoints_array[ii,2]))

        return waypointsList

    def plot(self):
        """
        Plots the waypoints, wind, and drop path
        """
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # target location
        ax.scatter(self.dropLocation.item(0),self.dropLocation.item(1),self.dropLocation.item(2),c='r', marker='o')
        ax.quiver(0.,0.,0.,self.wind.item(0),self.wind.item(1),self.wind.item(2),length=20.0,normalize=True)

        # closed baydoor movement
        V0_north = self.Va*np.cos(self.course_command) + self.wind.item(0)  # initial north velocity in inertial frame
        V0_east = self.Va*np.sin(self.course_command) + self.wind.item(1)   # initial east velocity in inertial frame
        time00 = np.linspace(0.0,self.time_delay,50)
        north00 = self.ff*V0_north*time00
        east00 = self.ff*V0_east*time00
        down00 = 0.0*time00
        ax.plot(self.NED_release_location.item(0)+north00,self.NED_release_location.item(1)+east00,self.NED_release_location.item(2)+down00,c='r',linewidth=5.0)

        # closed parachute dropping
        time11 = np.linspace(self.time_delay,self.time_to_open_parachute+self.time_delay,50)        # time between command and the parachute opening
        north11 = self.ff*V0_north*time11                                        # time 0 to 1 north difference in inertial frame
        east11 = self.ff*V0_east*time11                                          # time 0 to 1 east difference in inertial frame
        time12 = np.linspace(0.0,self.time_to_open_parachute,50)
        down11 = 0.5*self.gravity*time12**2             # time 0 to 1 down difference in inertial frame (Xf = X0 + V0*t + 0.5*a*t**2)
        ax.plot(self.NED_release_location.item(0)+north11,self.NED_release_location.item(1)+east11,self.NED_release_location.item(2)+down11,c='r',linewidth=5.0)

        # open parachute
        self.NED_parachute_open[0] = self.NED_release_location.item(0) + self.displacement0_1.item(0)
        self.NED_parachute_open[1] = self.NED_release_location.item(1) + self.displacement0_1.item(1)
        #ax.scatter(self.NED_parachute_open.item(0),self.NED_parachute_open.item(1),self.NED_parachute_open.item(2),c='b',marker='+')
        ax.plot([self.NED_parachute_open.item(0),self.dropLocation.item(0)],[self.NED_parachute_open.item(1),self.dropLocation.item(1)],[self.NED_parachute_open.item(2),self.dropLocation.item(2)],c='r',linewidth=5.0)

        # release Location
        for ii in range(len(self.waypoints)):
            ax.scatter(self.waypoints[ii].n,self.waypoints[ii].e,self.waypoints[ii].d, c='g',linewidth=1.0,marker='^')
        #ax.quiver(self.waypoints[0].n,self.waypoints[0].e,self.waypoints[0].d,np.cos(self.course_command),np.sin(self.course_command),0.,length=20.0,color='g')
        #ax.plot([self.waypoints[0].n,self.waypoints[-1].n],[self.waypoints[0].e,self.waypoints[-1].e],[self.waypoints[0].d,self.waypoints[-1].d], c='g',linewidth=5.0)


        for obstacle in self.obstacles:
            # Cylinder
            x = np.linspace((obstacle.n - obstacle.r), (obstacle.n + obstacle.r), 100)
            z = np.linspace(-obstacle.d,0., 100)
            # x = np.linspace(-1, 1, 25)
            # z = np.linspace(-2, 2, 25)
            Xc, Zc = np.meshgrid(x, z)
            Yc = np.sqrt(obstacle.r**2 - (Xc - obstacle.n)**2) + obstacle.e

            # Draw parameters
            ax.plot_surface(Xc, Yc, Zc, alpha=0.5, color='b')
            ax.plot_surface(Xc, (2.*obstacle.e-Yc), Zc, alpha=0.9, color='b')
        first = True
        boundaries = []
        last = []
        for bounds in self.boundariesList:
            if first:
                boundaries = np.array([[bounds.n, bounds.e, 0.]])
                last = np.array([[bounds.n, bounds.e, 0.]])
                first = False
                continue
            boundaries = np.append(boundaries, [[bounds.n, bounds.e, 0.]], axis=0)
        boundaries = np.append(boundaries, last, axis=0)
        ax.plot(boundaries[:, 0], boundaries[:, 1], boundaries[:, 2], label='Boundaries',c='b',linewidth=5.0)

        for t in ax.xaxis.get_major_ticks(): t.label.set_fontsize(12)
        for t in ax.yaxis.get_major_ticks(): t.label.set_fontsize(12)
        for t in ax.zaxis.get_major_ticks(): t.label.set_fontsize(12)


        ax.set_xlabel('North [m]',fontsize=20)
        ax.set_ylabel('East [m]',fontsize=20)
        ax.set_zlabel('Down [m]',fontsize=20)
        ax.xaxis.labelpad = 20
        ax.yaxis.labelpad = 20
        ax.zaxis.labelpad = 20
        #ax.xaxis._axinfo['label']['space_factor'] = 3.0
        #ax.yaxis._axinfo['label']['space_factor'] = 3.0
        #ax.zaxis._axinfo['label']['space_factor'] = 3.0


        ax.view_init(azim=-180.,elev=-120.)
        ax.axis([-750.,750.,-750.,750.])
        plt.show()

    def validateWaypoints(self):
        N = self.waypoints_array[:,0]
        E = self.waypoints_array[:,1]
        D = self.waypoints_array[:,2]
        clearance = 1.0
        if collisionCheck(self.obstacles, self.boundariesPolygon, N, E, D, clearance):
            return True
        else:
            return False

if __name__ == '__main__':

    #List of obastacles and boundaries
    obstaclesList = []
    obstaclesList.append(msg_ned(-350.,450.,50.,50.))
    obstaclesList.append(msg_ned(300.,-150.,100.,25.))
    obstaclesList.append(msg_ned(-300.,-250.,75.,75.))
    boundariesList = []

    home = [38.146269,-76.428164, 0.0]
    bd0 = [38.146269,-76.428164, 0.0]
    bd1 = [38.151625,-76.428683, 0.0]
    bd2 = [38.151889, -76.431467, 0.0]
    bd3 = [38.150594, -76.435361, 0.0]
    bd4 = [38.147567, -76.432342, 0.0]
    bd5 = [38.144667, -76.432947, 0.0]
    bd6 = [38.143256, -76.434767, 0.0]
    bd7 = [38.140464, -76.432636, 0.0]
    bd8 = [38.140719, -76.426014, 0.0]
    bd9 = [38.143761, -76.421206, 0.0]
    bd10 = [38.147347, -76.423211, 0.0]
    bd11 = [38.146131, -76.426653, 0.0]

    bd0_m = convert(home[0],home[1],home[2],bd0[0],bd0[1],bd0[2])
    bd1_m = convert(home[0],home[1],home[2],bd1[0],bd1[1],bd1[2])
    bd2_m = convert(home[0],home[1],home[2],bd2[0],bd2[1],bd2[2])
    bd3_m = convert(home[0],home[1],home[2],bd3[0],bd3[1],bd3[2])
    bd4_m = convert(home[0],home[1],home[2],bd4[0],bd4[1],bd4[2])
    bd5_m = convert(home[0],home[1],home[2],bd5[0],bd5[1],bd5[2])
    bd6_m = convert(home[0],home[1],home[2],bd6[0],bd6[1],bd6[2])
    bd7_m = convert(home[0],home[1],home[2],bd7[0],bd7[1],bd7[2])
    bd8_m = convert(home[0],home[1],home[2],bd8[0],bd8[1],bd8[2])
    bd9_m = convert(home[0],home[1],home[2],bd9[0],bd9[1],bd9[2])
    bd10_m = convert(home[0],home[1],home[2],bd10[0],bd10[1],bd10[2])
    bd11_m = convert(home[0],home[1],home[2],bd11[0],bd11[1],bd11[2])

    boundariesList.append(msg_ned(bd0_m[0],bd0_m[1]))
    boundariesList.append(msg_ned(bd1_m[0],bd1_m[1]))
    boundariesList.append(msg_ned(bd2_m[0],bd2_m[1]))
    boundariesList.append(msg_ned(bd3_m[0],bd3_m[1]))
    boundariesList.append(msg_ned(bd4_m[0],bd4_m[1]))
    boundariesList.append(msg_ned(bd5_m[0],bd5_m[1]))
    boundariesList.append(msg_ned(bd6_m[0],bd6_m[1]))
    boundariesList.append(msg_ned(bd7_m[0],bd7_m[1]))
    boundariesList.append(msg_ned(bd8_m[0],bd8_m[1]))
    boundariesList.append(msg_ned(bd9_m[0],bd9_m[1]))
    boundariesList.append(msg_ned(bd10_m[0],bd10_m[1]))
    boundariesList.append(msg_ned(bd11_m[0],bd11_m[1]))
    boundaryPoly = makeBoundaryPoly(boundariesList)


    dropLocation_gps = [38.145861, -76.426389, 0.0]
    dropLocation = convert(home[0],home[1],home[2],dropLocation_gps[0],dropLocation_gps[1],dropLocation_gps[2])
    dropLocation = np.array(dropLocation)
    wind = np.array([0.5,12.8,0.1])
    test = PayloadPlanner(dropLocation,obstaclesList,boundaryPoly,wind)
    result = test.plan(wind)
    test.plot()
