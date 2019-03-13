
import numpy as np
import matplotlib.pyplot as plt


class PayloadPlanner():
    """
    inputs: drop gps location, wind, obstacles
    outputs: X number of waypoints with a single drop waypoint
    """
    def __init__(self,dropLocation,obstacles,wind=[0.,0.,0.]):
        """
        initializes global variables
        """
        self.dropLocation = dropLocation            # location of where on the ground we want to hit [N, E, D]
        self.wind = wind                            # current wind vector [Wn,We,Wd]
        self.obstacles = obstacles                  # competition obstacles
        self.command_time = 0.15                    # seconds between command to drop and UGV leaves plane
        self.drop_altitude = 100.0                  # altitude for waypoints in meters
        self.time_delay = 0.5                       # seconds between command to open and baydoor opening
        self.time_to_open_parachute = 1.61          # seconds between baydoor opening and parachute opening
        self.terminal_velocity = 3.59               # from experimental data [m/s]

    def plan(self):
        """
        function called by mainplanner that returns bombdrop waypoint
        """
        NED_parachute_open = openParachute() # NED location we want the parachute to be fully open
        NED_command_release = self.commandRelease(NED_parachute_open) # NED location we want the command to happen
        waypoints = self.supportingPoints(NED_command_release)
        course_command = calcCourseCommand()
        return waypoints

    def openParachute(self,height):
        """
        calculates desired location for the parachute to open
        """
        target_north = self.dropLocation.item(0)
        target_east = self.dropLocation.item(1)
        target_down = self.dropLocation.itemm(2)
        dropTime =
        open_down = target_down -
        NED_open_parachute =
        return NED_open_parachute

    def commandRelease(self,NED_parachute_open):
        """
        calculates desired location for the release command
        """
        NED_command_release = 0.0
        return NED_command_release

    def calcCourseCommand(self):
        """
        calculates the command course angle to be directly into the wind
        """
        wind_north = self.wind.item(0)
        wind_east = self.wind.item(1)
        psi_wind = np.arctan2(wind_east,wind_north)
        course_command = np.arctan2(-wind_east,-wind_north)
        """
        print(course_command)
        plt.arrow(0.0,0.0,np.sin(course_command),np.cos(course_command))
        plt.arrow(0.0,0.0,wind_east,wind_north)
        plt.show()
        """
        return course_command

    def supportingPoints(self,NED_command_release):
        """
        given a command release location plot X points for a straight line of deployment run
        """
        wayponits = NED_command_release
        return waypoints


dropLocation = np.array([5.0,5.0,0.0])
wind = np.array([0.1,0.2,0.08])
obstacles = np.array([0.0,0.0,0.0])
test = PayloadPlanner(dropLocation,wind,obstacles)
test.calcCourseCommand()
