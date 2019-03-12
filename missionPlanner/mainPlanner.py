import rospy
import numpy as np
import task_planner # import the various path planning classes
from uav_msgs import interop_info, task, NED_pt, NED_list

class mainPlanner():
    """
    inputs: commands from gui
    outputs: 
    """
    
    def __init__(self):
        
        self.waypoints = []
        self.task = 0
        
        self._interop = rospy.Subscriber()
        self._sub_waypoints = rospy.Subscriber('approved_path', NED_list, self.update_path_callback, queue_size=5)
        self._sub_mission = rospy.Subscriber('task_command', task, self.update_task, queue_size=5)
        
        self._pub_task = rospy.Publisher('current_task', task, queue_size=5)
        self._pub_waypoints = rospy.Publisher('desired_waypoints', NED_list, queue_size=5)
        
        #Load the values that identify the various objectives
        #This needs to match what is being used in the GUI
        self.SEARCH_PLANNER = rospy.get_param("SEARCH")
        self.PAYLOAD_PLANNER = rospy.get_param("PAYLOAD")
        self.LOITER_PLANNER = rospy.get_param("LOITER")
        self.OBJECTIVE_PLANNER = rospy.get_param("OBJECTIVE")
        
    def update_path_callback(self, msg):
        """
        This function is called when waypoints are approved by the GUI and adds the waypoints to the approved path.
        The approved waypoints are sent from the GUI to the path manner via the approve_path
        message and topic.
        """
        
        self.waypoints.append(msg.waypoints)
        
    def update_task(self, msg):
        """
        This function is called when the desired task is changed by the GUI. The proper mission is then called.
        """
        
        self.task = msg.task
        
        #Each task_planner class function should return a NED_list msg
        if(self.task == self.SEARCH_PLANNER):
            rospy.loginfo('PATH PLANNER TASK BEING PLANNED')
            planned_points = task_planner.plan_search()
            
        elif(self.task == self.PAYLOAD_PLANNER):
            rospy.loginfo('PAYLOAD PLANNER TASK BEING PLANNED')
            planned_points = task_planner.plan_payload()
            
        elif(self.task == self.LOITER_PLANNER):
            rospy.loginfo('LOITER PLANNER TASK BEING PLANNED')
            planned_points = task_planner.plan_loiter()
            
        elif(self.task == self.OBJECTIVE_PLANNER):
            rospy.loginfo('OBJECTIVE PLANNER TASK BEING PLANNED')
            planned_points = task_planner.plan_objective()
            
        else:
            rospy.logfatal('TASK ASSIGNED BY GUI DOES NOT HAVE ASSOCIATED PLANNER')
            
        
        

