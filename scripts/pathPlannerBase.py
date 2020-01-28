#!/usr/bin/env python
import cProfile

import numpy as np
import rospy
from uav_msgs.msg import JudgeMission
from uav_msgs.srv import PlanMissionPoints

from metis import tools
from metis.rrt import RRT
from metis.ros import utils

class pathPlannerBase():

    def __init__(self):
        print("Initializing...")
        # self.command_sub_ = rospy.Subscriber('controller_commands_dev', Controller_Commands, self.cmdCallback, queue_size=5)
        # self.command_pub_ = rospy.Publisher('controller_commands', Controller_Commands, queue_size=1)
        # self.state_sub_ = rospy.Subscriber('state', State, self.stateCallback, queue_size=1)

        ref_lat = rospy.get_param("/fixedwing/ref_lat")
        ref_lon = rospy.get_param("/fixedwing/ref_lon")
        ref_h = rospy.get_param("/fixedwing/ref_h")
        self.ref_pos = [ref_lat, ref_lon, ref_h]

        print("Reference position received")
        #Get the obstacles, boundaries, and drop location in order to initialize the RRT class
        mission_type, self.obstacles, self.boundary_list, self.boundary_poly, drop_location = utils.get_server_data(JudgeMission.MISSION_TYPE_DROP, self.ref_pos)
        print("Server response:", mission_type, self.obstacles)

        self.RRT_planner = RRT(self.obstacles, self.boundary_list, animate=True) #Other arguments are available but have been given default values in the RRT constructor
        
        # Advertise the path planning service call
        self._plan_server = rospy.Service('plan_path_rrt', PlanMissionPoints, self.waypoints_callback)

        #Set up a service call to get waypoints from the mission planner
        self.mission_data = rospy.ServiceProxy('plan_mission', PlanMissionPoints)

        

        #Receive set of waypoints
        #Receive map features
        #Sends out full set of waypoints
        #Doesn't care where they came from

        #initialize rrt object with map features



    def waypoints_callback(self, req):

        #Send the service call with the desired mission type number
        resp = self.mission_data(req.mission_type)
        waypoints_ned = tools.msg2wypts(resp)

        #final_waypoints = self.RRT_planner.findFullPath(waypoints_ned)

        prof = cProfile.Profile()
        final_waypoints = prof.runcall(self.RRT_planner.findFullPath, waypoints_ned)
        prof.dump_stats("TimingData")

        reply = tools.wypts2msg(final_waypoints, req.mission_type)

        return(reply)

        #call rrt

if __name__ == '__main__':
    print("Initializing pathPlannerBase")
    rospy.init_node('pathPlannerBase', anonymous=True)
    print("Node initialized")
    pathPlan = pathPlannerBase()
    print("Class instantiated")
    while not rospy.is_shutdown():
        rospy.spin()


