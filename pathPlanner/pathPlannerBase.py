#!/usr/bin/env python
import rospy
import time
import numpy as np
import os
import sys
# from rosplane_msgs.msg import Controller_Commands, State
sys.path.append('..')
from messages.ned import msg_ned



class pathPlannerBase():

    def __init__(self):
        # self.command_sub_ = rospy.Subscriber('controller_commands_dev', Controller_Commands, self.cmdCallback, queue_size=5)
        # self.command_pub_ = rospy.Publisher('controller_commands', Controller_Commands, queue_size=1)
        # self.state_sub_ = rospy.Subscriber('state', State, self.stateCallback, queue_size=1)

        #Receive set of waypoints
        #Receive map features
        #Sends out full set of waypoints
        #Doesn't care where they came from

        #initialize rrt object with map features





        while not rospy.is_shutdown():
            rospy.spin()

        def waypointsCallback(self, msg):
            #call rrt

if __name__ == '__main__':
    rospy.init_node('pathPlannerBase')
    pathPlan = pathPlannerBase()
