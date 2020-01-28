# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

from __future__ import print_function
import logging
import rospy
from uav_msgs.msg import JudgeMission, State
from rosplane_msgs.msg import Waypoint
from uav_msgs.srv import (
    PlanMissionPoints,
    UploadPath,
    NewWaypoints,
    UpdateSearchParams,
)

import numpy as np

import metis.ros.utils as utils
from metis import Mission
from metis.manager import MissionManager
from metis.messages import msg_ned

log = logging.getLogger('METIS')


class MissionPlanner(object):
    """Handles the passing of information for the competition

    This class handles passing information between the interop server, the GUI,
    the path planner, and the various mission planners
    """

    def __init__(self, Va=17):
        """Creates a new MissionPlanner class with planner objectives

        This initializes a new MissionPlanner. The reference latitude, longitude,
        and altitude are taken from the .launch files. A service call is made 
        to get the stationary obstacles, boundaries, and payload drop location.
        The various mission planners are initilized with the obstacles and 
        boundaries. The payload planer additionally is initialized with the drop
        location known.

        Attributes
        ----------
        mission : metis.core.Mission
            The main mission object used to define targets, obstacles, and
            objectives.
        manager : metis.manager.MissionManager
            The manager class used to perform all planning functions and 
            autopilot state management.
        """
        rospy.logwarn("Is your home location set correctly (see ref_params in .launch file)?")

        self.mission = utils.get_mission()
        self.manager = MissionManager(self.mission)
        self.manager.target_height = rospy.get_param("target_h")

        # Provide services and publishers
        self._services = []
        self._services.append(rospy.Service("approved_path", UploadPath, self.update_path_callback))
        self._services.append(rospy.Service("clear_wpts", UploadPath, self.clear_waypoints))
        self._services.append(rospy.Service("plan_path", PlanMissionPoints, self.update_task_callback))
        self._services.append(rospy.Service("update_search_params", UpdateSearchParams, self.update_search_params))

        # self._pub_task = rospy.Publisher("current_task", JudgeMission, queue_size=5)
        self.wp_pub = rospy.Publisher("/fixedwing/waypoint_path", Waypoint, queue_size=10)

        self.plan = None
        print(self.mission)
        self.Va = Va

    def update_path_callback(self, req):
        """
        Adds waypoints to the approved path and publishes to the `waypoint_path` topic.

        This function is called when waypoints are approved by the GUI in 
        `ros_groundstation` and adds the waypoints to the approved path.
        The approved waypoints are sent from the GUI to the path manner via 
        the `approve_path` message and topic.

        Parameters
        ----------
        req : UploadPath
            The `UploadPath` ROS service (from `uav_msgs`).
        """
        msg = NewWaypoints()

        waypoints = []

        for point in self.plan.waypoints:
            new_point = Waypoint()
            new_point.w = [point.n, point.e, point.d]
            new_point.Va_d = self.Va  # airspeed (m/s)
            new_point.set_current = False  # erases list, sets this as current waypoint
            new_point.clear_wp_list = False  # removes all waypoints, returns to origin

            new_point.chi_d = 0 # Deseired course at this waypoint (rad)
            new_point.chi_valid = False # Desired course valid (dubin or fillet paths)

            waypoints.append(new_point)

        for point in waypoints:
            self.wp_pub.publish(point)
        rospy.loginfo("Waypoints sent.")
        return True

    def clear_waypoints(self, req):
        new_point = Waypoint()
        new_point.w = self.manager.default_pos.to_array(radius=False)
        new_point.clear_wp_list = True

        self.wp_pub.publish(new_point)
        rospy.loginfo("Waypoints cleared.")
        return True


    def update_search_params(self, req):
        """
        This function is called to change desired search parameters.

        Parameters
        ----------
        req : uav_msgs.srv.UpdateSearchParams
        """
        self.manager.set_search_params(req.height, req.waypoint_distance)
        return True


    def update_task_callback(self, req):
        """
        This function is called when the desired task is changed by the GUI. The proper mission is then called.

        Parameters
        ----------
        req : uav_msgs.srv.PlanMissionPoints
        """

        TASK = req.mission_type

        if TASK == JudgeMission.MISSION_TYPE_SEARCH:
            rospy.loginfo("SEARCH TASK BEING PLANNED")
            plan = self.manager.plan("search")

        elif TASK == JudgeMission.MISSION_TYPE_DROP:
            rospy.loginfo("PAYLOAD TASK BEING PLANNED")
            try:
                state_msg = rospy.wait_for_message("/state", State, timeout=10)
                wind = np.array([state_msg.wn, state_msg.we, 0.0])
            except rospy.ROSException as e:
                wind = np.array([0.0, 0.0, 0.0])
                rospy.logerr("PAYLOAD - No state message received. Setting wind to {}".format(wind))

            plan = self.manager.plan("payload", wind=wind)
            # rospy.set_param("DROP_LOCATION", drop_location)

        elif TASK == JudgeMission.MISSION_TYPE_LOITER:
            rospy.loginfo("LOITER PLANNER TASK BEING PLANNED")
            try:
                pos_msg = rospy.wait_for_message("/state", State, timeout=1)
                current_pos = msg_ned(pos_msg.position[0], pos_msg.position[1], pos_msg.position[2])
            except rospy.ROSException as e:
                current_pos = self.manager.default_pos
                rospy.logerr("LOITER - No state message received. Setting current position to the default position.")
            plan = self.manager.plan("loiter", current_pos=current_pos)

        elif TASK == JudgeMission.MISSION_TYPE_WAYPOINT:
            rospy.loginfo("OBJECTIVE PLANNER TASK BEING PLANNED")
            plan = self.manager.plan("objective")

        elif TASK == JudgeMission.MISSION_TYPE_OFFAXIS:
            rospy.loginfo("OFFAXIS PLANNER TASK BEING PLANNED")
            plan = self.manager.plan("offaxis")

        elif TASK == JudgeMission.MISSION_TYPE_LAND:
            rospy.loginfo("LANDING PATH BEING PLANNED")
            landing_msg = req.landing_waypoints
            if len(landing_msg.waypoint_list) == 2:
                try:
                    pos_msg = rospy.wait_for_message("/state", State, timeout=1)
                    curr_altitude = pos_msg.position[2]
                except rospy.ROSException as e:
                    curr_altitude = 0.0
                    rospy.logerr("LANDING - No state message received. Setting current altitude to {}".format(curr_altitude))

                landing_wypts = utils.msg2wypts(landing_msg)
                plan = self.manager.plan("landing", landing_wypts, curr_altitude)
            else:
                raise RuntimeError("Error in provided landing waypoints! {} waypoints specified.".format(len(landing_msg.waypoint_list)))

        # I believe the emergent object is just within the normal search boundaries
        elif TASK == JudgeMission.MISSION_TYPE_EMERGENT:
            pass

        else:
            rospy.logfatal("TASK ASSIGNED BY GUI DOES NOT HAVE ASSOCIATED PLANNER")

        wypts_msg = utils.wypts2msg(plan.waypoints, TASK)
        plan.plot()

        self.plan = plan

        print(wypts_msg)
        return wypts_msg
