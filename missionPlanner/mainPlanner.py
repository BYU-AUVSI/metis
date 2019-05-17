#!/usr/bin/python


#Get the path to the package so we can use files in the tools folder
#ROS doesn't use the normal path so we can't just sys.path.append('..')
import sys

import rospy
import rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('metis'))
from uav_msgs.msg import JudgeMission, NED_list
from uav_msgs.srv import GetMissionWithId, PlanMissionPoints


import numpy as np

from messages.ned import msg_ned
from tools import tools
from payloadPlanner import PayloadPlanner
from loiterPlanner import LoiterPlanner
from searchPlanner import SearchPlanner
from objectivePointsPlanner import ObjectivePointsPlanner
from offaxisPlanner import OffaxisPlanner

class mainPlanner():
    """Handles the passing of information for the competition

    This class handles passing information between the interop server, the GUI, the path planner, and the various mission planners
    """

    def __init__(self):
        """brief Creates a new main planner objectives

        This initializes a new main planner. The reference latitude, longitude, and altitude are taken from the .launch files
        A service call is made to get the stationary obstacles, boundaries, and payload drop location.
        The various mission planners are initilized with the obstacles and boundaries. The paylod planer additionally is initialized
        with the drop location known.
        """


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

        self._plan_server = rospy.Service('plan_mission', PlanMissionPoints, self.update_task_callback)

        #Load the values that identify the various objectives
        #This needs to match what is being used in the GUI
        self._SEARCH_PLANNER = JudgeMission.MISSION_TYPE_SEARCH
        self._PAYLOAD_PLANNER = JudgeMission.MISSION_TYPE_DROP
        self._LOITER_PLANNER = JudgeMission.MISSION_TYPE_LOITER
        self._OBJECTIVE_PLANNER = JudgeMission.MISSION_TYPE_WAYPOINT

        

        #Get the obstacles, boundaries, and drop location in order to initialize the planner classes
        mission_type, obstacles, boundary_list, boundary_poly, drop_location = tools.get_server_data(JudgeMission.MISSION_TYPE_DROP, self.ref_pos)

        #Initiate the planner classes
        self._plan_payload = PayloadPlanner(drop_location[0], obstacles, boundary_list, boundary_poly)
        self._plan_loiter = LoiterPlanner(obstacles)
        self._plan_search = SearchPlanner(boundary_list, obstacles)
        self._plan_objective = ObjectivePointsPlanner(obstacles)
        self._plan_offaxis = OffaxisPlanner(boundary_list, obstacles)

        #-----START DEBUG----
        #This code is just used to visually check that everything worked ok. Can be removed anytime.
        print("Obstacles")
        for obstacle in obstacles:
            print(obstacle.n, obstacle.e, obstacle.d, obstacle.r)
        print("Boundaries")
        for boundary in boundary_list:
            print(boundary.n, boundary.e)
        print("Drop")
        for drop in drop_location:
            print(drop.n, drop.e, drop.d)
        #-----END DEBUG----


    #TODO replace lists of lists with NED msg type
    #TODO remove obstacle D position



    def update_path_callback(self, msg):
        """
        This function is called when waypoints are approved by the GUI and adds the waypoints to the approved path.
        The approved waypoints are sent from the GUI to the path manner via the approve_path
        message and topic.
        """

        self.waypoints.append(msg.waypoints)

    

    def update_task_callback(self, req):
        """
        This function is called when the desired task is changed by the GUI. The proper mission is then called.


        """

        self.task = req.mission_type

        mission_type, obstacles, boundary_list, boundary_poly, waypoints = tools.get_server_data(self.task, self.ref_pos)

        #Each task_planner class function should return a NED_list msg
        #These classes can be switched out depending on the desired functionality
        if(self.task == self._SEARCH_PLANNER):
            rospy.loginfo('SEARCH TASK BEING PLANNED')
            planned_points = self._plan_search.plan(waypoints)

        elif(self.task == self._PAYLOAD_PLANNER):
            rospy.loginfo('PAYLOAD TASK BEING PLANNED')
            planned_points = self._plan_payload.plan()

        elif(self.task == self._LOITER_PLANNER):
            rospy.loginfo('LOITER PLANNER TASK BEING PLANNED')
            planned_points = self._plan_loiter.plan(waypoints)

        elif(self.task == self._OBJECTIVE_PLANNER): # This is the task that deals with flying the mission waypoints. We call it objective to avoid confusion with the waypoints that are used to define the drop flight path or search flight path
            rospy.loginfo('OBJECTIVE PLANNER TASK BEING PLANNED')
            planned_points = self._plan_objective.plan(waypoints)

        elif(self.task == JudgeMission.MISSION_TYPE_OFFAXIS):
            rospy.loginfo('OFFAXIS PLANNER TASK BEING PLANNED')
            planned_points = self._plan_offaxis.plan(waypoints)

        elif(self.task == JudgeMission.MISSION_TYPE_EMERGENT): # I believe the emergent object is just within the normal search boundaries
            pass

        else:
            rospy.logfatal('TASK ASSIGNED BY GUI DOES NOT HAVE ASSOCIATED PLANNER')

        #Convert python NED class to rospy ned msg
        wypts_msg = tools.wypts2msg(planned_points,self.task)
        print(wypts_msg)
        return wypts_msg



#Run the main planner
if __name__ == "__main__":

    rospy.init_node('main_planner', anonymous=True)
    test_planner = mainPlanner()
    while not rospy.is_shutdown():
        rospy.spin()

