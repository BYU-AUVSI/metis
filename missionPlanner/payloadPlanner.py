
import numpy as np
import matplotlib.pyplot as plt


class PayloadPlanner():
    """
    inputs: drop gps location, wind, obstacles
    outputs: X number of waypoints with a single drop waypoint
    """
    def __init__(self,dropLocation,wind,obstacles,boundaries):
        """
        initializes global variables
        """
        self.dropLocation = dropLocation            # location of where on the ground we want to hit [N, E, D]
        self.wind = wind                            # current wind vector [Wn,We,Wd]
        self.obstacles = obstacles                  # competition obstacles
        self.command_time = 0.15                    # seconds between command to drop and UGV leaves plane
        self.drop_altitude = 100.0                  # altitude for waypoints in meters above 0.0 of ground station
        self.time_delay = 0.5                       # seconds between command to open and baydoor opening
        self.time_to_open_parachute = 1.61          # seconds between baydoor opening and parachute opening
        self.terminal_velocity = 3.59               # from experimental data [m/s]
        self.Va = 17.5  # pull from                 # competition design Va [m/s]
        self.gravity = 9.8147 # pull from yaml      # acceleration of gravity [m/s**2]
        self.course_command = 0.0                   # initialize course commmand
        self.NED_parachute_open = np.array([0.0,0.0,0.0])   # location where the parachute oepns [N, E, D]
        self.NED_release_location = np.array([0.0,0.0,0.0]) # location where the command to relase should be given [N, E, D]


    def plan(self,wind):
        """
        function called by mainplanner that returns bombdrop waypoint
        """
        self.wind = wind
        self.calcCourseCommand()
        displacement0_1 = self.calcClosedParachuteDrop()
        displacement1_2 = self.calcOpenParachuteDrop()
        self.calcReleaseLocation(displacement0_1,displacement1_2)
        waypoints = self.calcSupportingPoints()
        return waypoints

    def calcClosedParachuteDrop(self):
        """
        Calculates the motion between the commanded relase and the parachute opening
        """
        V0_north = self.Va*np.cos(self.course_command) + self.wind.item(0)  # initial north velocity in inertial frame
        V0_east = self.Va*np.sin(self.course_command) + self.wind.item(1)   # initial east velocity in inertial frame
        V0_down = self.wind.item(2)                                         # initial down velocity in inertial frame
        time0_1 = self.time_delay + self.time_to_open_parachute             # time between command and the parachute opening
        dNorth0_1 = V0_north*time0_1                                        # time 0 to 1 north difference in inertial frame
        dEast0_1 = V0_east*time0_1                                          # time 0 to 1 east difference in inertial frame
        dDown0_1 = V0_down*time0_1 + 0.5*self.gravity*time0_1**2            # time 0 to 1 down difference in inertial frame (Xf = X0 + V0*t + 0.5*a*t**2)
        self.NED_parachute_open[2] = -self.drop_altitude + dDown0_1         # Down position at parachute opening
        return np.array([dNorth0_1,dEast0_1])

    def calcOpenParachuteDrop(self):
        """
        Calculates the motion between the parachute opening and hitting the ground target
        """
        target_north = self.dropLocation.item(0)    # target north location
        target_east = self.dropLocation.item(1)     # target east location
        target_down = self.dropLocation.item(2)    # target down location
        time1_2 = (target_down - self.NED_parachute_open.item(2))/self.terminal_velocity  # calculate time from parahute opening to hitting the target (Xf - X0 = v*t)
        dNorth1_2 = self.wind.item(0)*time1_2       # time 1 to 2 north difference in inertial frame
        dEast1_2 = self.wind.item(1)*time1_2        # time 1 to 2 east difference in inertial frame
        return np.array([dNorth1_2,dEast1_2])

    def calcReleaseLocation(self,displacement0_1,displacement1_2):
        """
        calculates desired location for the release command
        """
        dNorth0_1 = displacement0_1.item(0)                                 # region 1 north displacement
        dEast0_1 = displacement0_1.item(1)                                  # region 1 east displacement
        dNorth1_2 = displacement1_2.item(0)                                 # region 2 north displacement
        dEast1_2 = displacement1_2.item(1)                                  # region 2 east displacement
        release_north = self.dropLocation.item(0) + dNorth1_2 + dNorth0_1   # release north position
        release_east = self.dropLocation.item(1) + dEast1_2 + dEast0_1      # release east position
        release_down = -self.drop_altitude                                  # release down position
        self.NED_release_location = np.array([release_north,release_east,release_down])

    def calcCourseCommand(self):
        """
        calculates the command course angle to be directly into the wind
        """
        wind_north = self.wind.item(0)
        wind_east = self.wind.item(1)
        psi_wind = np.arctan2(wind_east,wind_north)
        course_command = np.arctan2(-wind_east,-wind_north)
        self.course_command = course_command
        """
        print(course_command)
        plt.arrow(0.0,0.0,np.sin(course_command),np.cos(course_command))
        plt.arrow(0.0,0.0,wind_east,wind_north)
        plt.show()
        """

    def calcSupportingPoints(self):
        """
        given a command release location plot X points for a straight line of deployment run
        """
        waypoints = self.NED_release_location
        return waypoints


dropLocation = np.array([0.0,0.0,0.0])
wind = np.array([0.1,0.5,0.08])
obstacles = np.array([0.0,0.0,0.0])
boundaries = np.array([0.0,0.0,0.0])
test = PayloadPlanner(dropLocation,wind,obstacles,boundaries)
result = test.plan(wind)
print(result)
