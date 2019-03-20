#!/usr/bin/python

#Get the path to the package so we can use files in the tools folder
#ROS doesn't use the normal path so we can't just sys.path.append('..')
import sys
import rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('metis'))

import rospy
import numpy as np
from tools.tools import convert, wypts2msg
from payloadPlanner import PayloadPlanner
from loiterPlanner import LoiterPlanner
from searchPlanner import SearchPlanner
from objectivePointsPlanner import ObjectivePointsPlanner
from uav_msgs.msg import JudgeMission, NED_list
from uav_msgs.srv import GetMissionWithId, PlanMissionPoints
class mainPlanner():
    """
    inputs: commands from gui
    outputs:
    """

    def __init__(self):


        #Get ref lat, lon from launch file
        ref_lat = rospy.get_param("ref_lat")
        ref_lon = rospy.get_param("ref_lon")
        ref_h = rospy.get_param("ref_h")
        self.ref_pos = [ref_lat, ref_lon, ref_h]

        #Keeps track of what task is currently being executed
        self.task = 0

        self._sub_waypoints = rospy.Subscriber('approved_path', NED_list, self.update_path_callback, queue_size=5)
        #Proposing a switch to a service call rather than a topic to get info from GUI. If that holds then delete this line
        #self._sub_mission = rospy.Subscriber('task_command', JudgeMission, self.update_task, queue_size=5)

        self._pub_task = rospy.Publisher('current_task', JudgeMission, queue_size=5)
        #Proposing a switch to a service call rather than a topic to get info to the GUI. If that holds, delete this line
        #self._pub_waypoints = rospy.Publisher('desired_waypoints', NED_list, queue_size=5)

        self._plan_server = rospy.Service('plan_mission', PlanMissionPoints, self.update_task)

        #Load the values that identify the various objectives
        #This needs to match what is being used in the GUI
        self.SEARCH_PLANNER = JudgeMission.MISSION_TYPE_SEARCH
        self.PAYLOAD_PLANNER = JudgeMission.MISSION_TYPE_DROP
        self.LOITER_PLANNER = JudgeMission.MISSION_TYPE_LOITER
        self.OBJECTIVE_PLANNER = JudgeMission.MISSION_TYPE_WAYPOINT

        #Wait for the interop client service call to initiate
        rospy.wait_for_service('get_mission_with_id')

        #Get the obstacles, boundaries, and drop location in order to initialize the planner classes
        mission_type, obstacles, boundaries, drop_location = self.get_server_data(JudgeMission.MISSION_TYPE_DROP)

        #Initiate the planner classes
        self.plan_payload = PayloadPlanner(drop_location, obstacles, boundaries)
        self.plan_loiter = LoiterPlanner(obstacles)
        self.plan_search = SearchPlanner(obstacles)
        self.plan_objective = ObjectivePointsPlanner(obstacles)

        #This code is just used to visually check that everything worked ok. Can be removed anytime.
        print("obstacles")
        print(obstacles)
        print("boundaries")
        print(boundaries)
        print("drop")
        print(drop_location)



    def get_server_data(self, mission_type):
        """
        Polls the interop server node via a service call to get the data associated with a specific mission
        Returns a tuple of all the information

        Parameters
        ----------
        mission_type : int
            The mission type number as defined in the JudgeMission message

        Returns
        -------
        mission_type : int
            The mission type number is simply passed through as a reference
        obstacles : list of list
            A list of lists where each inner list describes the NED position, height, and radius of each obstacle in meters
        boundaries : list of lists
            A list of lists where each inner list describes the NED position of each boundary position
        waypoints : list of lists
            A list of lists where each inner list describes the NED position of each waypoint associated with the desired mission_type
        """

        #Set up a service call to poll the interop server
        mission_data = rospy.ServiceProxy('get_mission_with_id', GetMissionWithId)

        #Send the service call with the desired mission type number
        resp = mission_data(mission_type)

        #Get boundaries, obstacles, flight waypoints
        obstacles = self.convert_obstacles(resp.mission, self.ref_pos)
        boundaries = self.convert_boundaries(resp.mission, self.ref_pos)
        waypoints =  self.convert_waypoints(resp.mission, self.ref_pos)

        return mission_type, obstacles, boundaries, waypoints


    def update_path_callback(self, msg):
        """
        This function is called when waypoints are approved by the GUI and adds the waypoints to the approved path.
        The approved waypoints are sent from the GUI to the path manner via the approve_path
        message and topic.
        """

        self.waypoints.append(msg.waypoints)

    def convert_obstacles(self, msg, ref_pos):
        """
        Converts the obstacles from the rospy message to a list of lists

        Parameters
        ----------
        msg : JudgeMission message
            The message received from the interop server
        ref_pos : list
            The reference latitude, longitude, and height (in m)

        Returns
        -------
        list
            a list of lists describing all the obstacles
            each inner list has the North location, East location, Down location, Obstacle height, and Obstacle radius of a single obstacle
            all measurements are in meters

        """
        ref_lat = ref_pos[0]
        ref_lon = ref_pos[1]
        ref_h = ref_pos[2]

        obstacle_list = []

        for i in msg.stationary_obstacles:
            obs_NED = convert(ref_lat, ref_lon, ref_h, i.point.latitude, i.point.longitude, i.point.altitude)
            obs_NED.append(i.cylinder_height)
            obs_NED.append(i.cylinder_radius)
            obstacle_list.append(obs_NED)

        return obstacle_list

    def convert_waypoints(self, msg, ref_pos):
        """
        Converts the waypoints obtained from the interop server to NED coordinates
        This function doesn't care about what mission is being run, it just gets the waypoints
        which can be for the drop location, flight location, or search boundaries, and converts them


        Parameters
        ----------
        msg : JudgeMission message
            The message received from the interop server
        ref_pos : list
            The reference latitude, longitude, and height (in m)

        Returns
        -------
        list
            a list of lists describing all the waypoints
            each inner list has the North location, East location, and Down location of a single waypoint
            all measurements are in meters

        """
        ref_lat = ref_pos[0]
        ref_lon = ref_pos[1]
        ref_h = ref_pos[2]

        waypoint_list = []

        for i in msg.waypoints:
            wpt_NED = convert(ref_lat, ref_lon, ref_h, i.point.latitude, i.point.longitude, i.point.altitude)
            waypoint_list.append(wpt_NED)

        return waypoint_list

    def convert_boundaries(self, msg, ref_pos):
        """
        Converts the boundary points obtained from the interop server to NED coordinates

        Parameters
        ----------
        msg : JudgeMission message
            The message received from the interop server
        ref_pos : list
            The reference latitude, longitude, and height (in m)

        Returns
        -------
        list
            a list of lists describing all the boundary points
            each inner list has the North location, East location, and Down location of a single boundary point
            all measurements are in meters

        """
        ref_lat = ref_pos[0]
        ref_lon = ref_pos[1]
        ref_h = ref_pos[2]

        boundary_list = []

        for i in msg.boundaries:
            bnd_NED = convert(ref_lat, ref_lon, ref_h, i.point.latitude, i.point.longitude, i.point.altitude)
            boundary_list.append(bnd_NED)

        return boundary_list

    #TODO This function should probably be a service call callback function
    #TODO This function should probably call the interop server to get the desired waypoints
    def update_task(self, req):
        """
        This function is called when the desired task is changed by the GUI. The proper mission is then called.
        """

        self.task = req.mission_type

        mission_type, obstacles, boundaries, waypoints = self.get_server_data(self.task)

        #Each task_planner class function should return a NED_list msg
        #These classes can be switched out depending on the desired functionality
        if(self.task == self.SEARCH_PLANNER):
            rospy.loginfo('PATH PLANNER TASK BEING PLANNED')
            planned_points = self.plan_search.plan(waypoints)

        elif(self.task == self.PAYLOAD_PLANNER):
            rospy.loginfo('PAYLOAD PLANNER TASK BEING PLANNED')
            planned_points = self.plan_payload.plan(waypoints)

        elif(self.task == self.LOITER_PLANNER):
            rospy.loginfo('LOITER PLANNER TASK BEING PLANNED')
            planned_points = self.plan_loiter.plan(waypoints)

        elif(self.task == self.OBJECTIVE_PLANNER):
            rospy.loginfo('OBJECTIVE PLANNER TASK BEING PLANNED')
            planned_points = self.plan_objective.plan(waypoints)

        else:
            rospy.logfatal('TASK ASSIGNED BY GUI DOES NOT HAVE ASSOCIATED PLANNER')

        wypts_msg = wypts2msg(planned_points,self.task)
        
        return wypts_msg


#Run the main planner
if __name__ == "__main__":
    rospy.init_node('main_planner', anonymous=True)
    test_planner = mainPlanner()
    while not rospy.is_shutdown():
        rospy.spin()
