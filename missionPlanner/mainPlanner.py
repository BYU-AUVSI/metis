#!/usr/bin/python


#Get the path to the package so we can use files in the tools folder
#ROS doesn't use the normal path so we can't just sys.path.append('..')
import sys

import rospy
import rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('metis'))
from uav_msgs.msg import JudgeMission, Waypoint, State #Waypoint, State are copied from rosplane_msgs so rosplane is not neededs for metis
from uav_msgs.srv import GetMissionWithId, PlanMissionPoints, UploadPath, NewWaypoints #NewWaypoints is copied from the rosplane_msgs so rosplane is not needed for metis

#from rosplane_msgs.msg import Waypoint #This is where the msgs and srv were originally but couldn't import them
#from rosplane_msgs.srv import NewWaypoints

#from rosplane_msgs.msg import State

import numpy as np

from messages.ned import msg_ned
from tools import tools
from payloadPlanner import PayloadPlanner
from loiterPlanner import LoiterPlanner
from searchPlanner import SearchPlanner
from objectivePointsPlanner import ObjectivePointsPlanner
from offaxisPlanner import OffaxisPlanner
from landingPlanner import LandingPlanner

from pathPlanner.rrt import RRT

class mainPlanner():
    """Handles the passing of information for the competition

    This class handles passing information between the interop server, the GUI, the path planner, and the various mission planners
    """

    def __init__(self, Va=17):
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

        self._ser_waypoints = rospy.Service('approved_path', UploadPath, self.update_path_callback)

        self._ser_clear = rospy.Service('clear_wpts', UploadPath, self.clear_waypoints)
        #Proposing a switch to a service call rather than a topic to get info from GUI. If that holds then delete this line
        #self._sub_mission = rospy.Subscriber('task_command', JudgeMission, self.update_task, queue_size=5)

        self._pub_task = rospy.Publisher('current_task', JudgeMission, queue_size=5)
        #Proposing a switch to a service call rather than a topic to get info to the GUI. If that holds, delete this line
        #self._pub_waypoints = rospy.Publisher('desired_waypoints', NED_list, queue_size=5)

        #self._plan_server = rospy.Service('plan_mission', PlanMissionPoints, self.update_task_callback)
        self._plan_server = rospy.Service('plan_path', PlanMissionPoints, self.update_task_callback)

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
        self._plan_loiter = LoiterPlanner(obstacles, boundary_poly)
        self._plan_search = SearchPlanner(boundary_list, obstacles)
        self._plan_objective = ObjectivePointsPlanner(obstacles)
        self._plan_offaxis = OffaxisPlanner(boundary_list, obstacles)
        self._plan_landing = LandingPlanner(boundary_list, obstacles)

        self.rrt = RRT(obstacles, boundary_list, animate=False) #Other arguments are available but have been given default values in the RRT constructor

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

        self.Va = Va


    #TODO replace lists of lists with NED msg type
    #TODO remove obstacle D position



    def update_path_callback(self, req):
        """
        This function is called when waypoints are approved by the GUI and adds the waypoints to the approved path.
        The approved waypoints are sent from the GUI to the path manner via the approve_path
        message and topic.
        """

        print("Waiting For Rosplane service")
        #Wait for the interop client service call to initiate
        rospy.wait_for_service('waypoint_path')

        #Set up a service call to poll the interop server
        waypoint_update = rospy.ServiceProxy('waypoint_path', NewWaypoints)

        msg = NewWaypoints()

        waypoints = []

        for point in self.planned_waypoints:
            new_point = Waypoint()
            new_point.w = [point.n, point.e, point.d]
            new_point.Va_d = self.Va
            new_point.drop_bomb = False
            new_point.landing = False
            new_point.set_current = False
            new_point.clear_wp_list = False
            new_point.loiter_point = False
            new_point.priority = 1

            waypoints.append(new_point)

        waypoints[-1].loiter_point = True
        waypoints[-1].priority = 0

        #Send the service call with the desired mission type number
        resp = waypoint_update(waypoints)

        print("Waypoints sent")

        return True

    def clear_waypoints(self, req):

        print("Clearing waypoints")

        rospy.wait_for_service('waypoint_path')

        #Set up a service call to poll the interop server
        waypoint_update = rospy.ServiceProxy('waypoint_path', NewWaypoints)

        waypoints = []


        new_point = Waypoint()

        new_point.clear_wp_list = True


        waypoints.append(new_point)

        #Send the service call with the desired mission type number
        resp = waypoint_update(waypoints)

        print("Waypoints cleared")

        return True


    def update_task_callback(self, req):
        """
        This function is called when the desired task is changed by the GUI. The proper mission is then called.


        """

        self.task = req.mission_type

        # interop server doesn't provide landing info
        if(self.task != JudgeMission.MISSION_TYPE_LAND):
            mission_type, obstacles, boundary_list, boundary_poly, waypoints = tools.get_server_data(self.task, self.ref_pos)

        #Each task_planner class function should return a NED_list msg
        #These classes can be switched out depending on the desired functionality
        connect = False

        if(self.task == self._SEARCH_PLANNER):
            rospy.loginfo('SEARCH TASK BEING PLANNED')
            planned_points = self._plan_search.plan(waypoints)

        elif(self.task == self._PAYLOAD_PLANNER):
            rospy.loginfo('PAYLOAD TASK BEING PLANNED')
            try:
                state_msg = rospy.wait_for_message("/state", State, timeout=10)
                wind = np.array([state_msg.wn,state_msg.we,0.])
            except rospy.ROSException as e:
                print("no state message received")
                wind = np.array([0.0,0.0,0.0])

            planned_points, drop_location = self._plan_payload.plan(wind)
            rospy.set_param('DROP_LOCATION', drop_location)

        elif(self.task == self._LOITER_PLANNER):
            rospy.loginfo('LOITER PLANNER TASK BEING PLANNED')
            try:
                pos_msg = rospy.wait_for_message("/state", State, timeout=1)
                current_pos = msg_ned(pos_msg.position[0],pos_msg.position[1],pos_msg.position[2])
            except rospy.ROSException as e:
                print("Loiter - No State msg recieved")
                current_pos = msg_ned(0.,0.,0.)
            planned_points = self._plan_loiter.plan(current_pos)

        elif(self.task == self._OBJECTIVE_PLANNER): # This is the task that deals with flying the mission waypoints. We call it objective to avoid confusion with the waypoints that are used to define the drop flight path or search flight path
            rospy.loginfo('OBJECTIVE PLANNER TASK BEING PLANNED')
            planned_points = self._plan_objective.plan(waypoints)
            connect = True

        elif(self.task == JudgeMission.MISSION_TYPE_OFFAXIS):
            rospy.loginfo('OFFAXIS PLANNER TASK BEING PLANNED')
            planned_points = self._plan_offaxis.plan(waypoints)

        elif(self.task == JudgeMission.MISSION_TYPE_LAND):
            rospy.loginfo('LANDING PATH BEING PLANNED')
            landing_msg = req.landing_waypoints
            if (len(landing_msg.waypoint_list) == 2):
                landing_wypts = tools.msg3wypts(landing_msg)
                planned_points = self._plan_landing.plan(landing_wypts)
            else:
                planned_points = [msg_ned(0, 0, 0)]
                print("No landing waypoints specified")

        elif(self.task == JudgeMission.MISSION_TYPE_EMERGENT): # I believe the emergent object is just within the normal search boundaries
            pass

        else:
            rospy.logfatal('TASK ASSIGNED BY GUI DOES NOT HAVE ASSOCIATED PLANNER')

        try:
            pos_msg = rospy.wait_for_message("/state", State, timeout=1)
            current_pos = msg_ned(pos_msg.position[0],pos_msg.position[1],pos_msg.position[2])
        except rospy.ROSException as e:
            print("No State msg recieved")
            current_pos = msg_ned(0.,0.,0.)

        planned_points.insert(0,current_pos)

        final_path = self.rrt.findFullPath(planned_points, connect=connect)


        #Convert python NED class to rospy ned msg
        wypts_msg = tools.wypts2msg(final_path,self.task)
        self.planned_waypoints = final_path
        print(wypts_msg)
        return wypts_msg



#Run the main planner
if __name__ == "__main__":

    rospy.init_node('main_planner', anonymous=True)
    test_planner = mainPlanner()
    while not rospy.is_shutdown():
        rospy.spin()
