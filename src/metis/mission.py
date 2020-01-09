# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

from __future__ import print_function
import rospy
from uav_msgs.msg import JudgeMission, State #, Waypoint
from rosplane_msgs.msg import Waypoint
from uav_msgs.srv import (
    GetMissionWithId,
    PlanMissionPoints,
    UploadPath,
    NewWaypoints,
    UpdateSearchParams,
)

import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Point

import metis.ros.utils as utils
from metis import tools
from metis.messages import msg_ned
from metis.planners import (
    LoiterPlanner,
    LandingPlanner,
    ObjectivePointsPlanner,
    OffaxisPlanner,
    PayloadPlanner,
    SearchPlanner,
)
from metis.plotter import MissionPlotter
from metis.rrt import RRT


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
        target_h : float
            Target altitude in <units>.
        ref_pos : list of floats
            A list of floats, size 3, in the form [latitude, longitude, height].
        DEFAULT_POS : tuple of floats
            A tuple of floats, length 3, equal to [0, 0, -target_height]
        task : int
            The task currently being executed.
        planners : dict
            A dictionary of planner names to planner objects.
        mission_type : int
        obstacles : list
        boundary_list : list
        boundary_poly : list
        drop_location : point
        """

        # Get reference latitude and longitude from the launch file 
        # (it's a <param> in the XML file).
        ref_lat = rospy.get_param("ref_lat")
        ref_lon = rospy.get_param("ref_lon")
        ref_h = rospy.get_param("ref_h")
        self.target_h = rospy.get_param("target_h")
        self.ref_pos = [ref_lat, ref_lon, ref_h]
        self.DEFAULT_POS = (0.0, 0.0, -1*self.target_h)

        # Keep track of what task is currently being executed
        self.task = 0

        # Provide services and publishers
        self._ser_waypoints = rospy.Service("approved_path", UploadPath, self.update_path_callback)
        self._ser_clear = rospy.Service("clear_wpts", UploadPath, self.clear_waypoints)
        self._pub_task = rospy.Publisher("current_task", JudgeMission, queue_size=5)
        self._plan_server = rospy.Service("plan_path", PlanMissionPoints, self.update_task_callback)
        self._ser_search_params = rospy.Service("update_search_params", UpdateSearchParams, self.update_search_params)

        self.wp_pub = rospy.Publisher("waypoint_path", Waypoint, queue_size=10)

        # Get the obstacles, boundaries, and drop location in order to initialize the planner classes
        (
            self.mission_type,
            self.obstacles,
            self.boundary_list,
            self.boundary_poly,
            self.drop_location,
        ) = tools.get_server_data(JudgeMission.MISSION_TYPE_DROP, self.ref_pos)

        # Initiate the planner classes
        self.planners = {
            "landing": LandingPlanner(
                self.boundary_list, self.obstacles, boundary_poly=self.boundary_poly
            ),
            "loiter": LoiterPlanner(
                self.boundary_list, self.obstacles, boundary_poly=self.boundary_poly
            ),
            "objective": ObjectivePointsPlanner(
                self.boundary_list, self.obstacles, boundary_poly=self.boundary_poly
            ),
            "offaxis": OffaxisPlanner(
                self.boundary_list, self.obstacles, boundary_poly=self.boundary_poly
            ),
            "payload": PayloadPlanner(
                self.drop_location[0],
                self.boundary_list,
                self.obstacles,
                boundary_poly=self.boundary_poly,
            ),
            "search": SearchPlanner(
                self.boundary_list, self.obstacles, boundary_poly=self.boundary_poly
            ),
        }

        self.landing = False
        self.last_exists = False
        self.last_waypoint = msg_ned(*self.DEFAULT_POS)

        # Other arguments are available but have been given default values in the RRT constructor
        self.rrt = RRT(self.obstacles, self.boundary_list, animate=False)

        # -----START DEBUG----
        # This code is just used to visually check that everything worked ok. Can be removed anytime.
        print("Obstacles")
        for obstacle in self.obstacles:
            print(obstacle.n, obstacle.e, obstacle.d, obstacle.r)
        print("Boundaries")
        for boundary in self.boundary_list:
            print(boundary.n, boundary.e)
        print("Drop")
        for drop in self.drop_location:
            print(drop.n, drop.e, drop.d)

        _, _, _, _, objective_waypts = tools.get_server_data(
            JudgeMission.MISSION_TYPE_WAYPOINT, self.ref_pos
        )
        _, _, _, _, search_boundary = tools.get_server_data(
            JudgeMission.MISSION_TYPE_SEARCH, self.ref_pos
        )
        # self.obstacles = obstacles
        # self.drop_location = drop_location
        self.objective_waypts = objective_waypts
        # V V V Except it would seem this line is necessary right now. V V V
        self.search_boundary = search_boundary
        # self.boundary_list = boundary_list

        # -----END DEBUG----

        self.Va = Va

    # TODO replace lists of lists with NED msg type
    # TODO remove obstacle D position

    def init_mission_plotter(self):
        mp = MissionPlotter()
        mp.addRegion(self.search_boundary, "Search Region", color="orange")
        mp.addRegion(self.boundary_list, "Boundary")
        mp.addObstacles(self.obstacles, "Obstacles")
        mp.addWaypoints(
            self.drop_location, "Drop Target", color="green", size=25, marker="X"
        )
        mp.addWaypoints(
            self.objective_waypts,
            "Objective Waypoints",
            color="blue",
            size=12,
            marker="o",
        )

        self.mp = mp

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

        Returns
        -------
        val : bool
            Returns true no matter what, for some inexplicable reason.
        """
        msg = NewWaypoints()

        waypoints = []

        for point in self.planned_waypoints:
            new_point = Waypoint()
            new_point.w = [point.n, point.e, point.d]
            new_point.Va_d = self.Va    # airspeed (m/s)
            new_point.set_current = False # erases list, sets this as current waypoint
            new_point.clear_wp_list = False # removes all waypoints, returns to origin

            # # From the old Waypoint message
            # new_point.drop_bomb = False
            # new_point.landing = False
            # new_point.loiter_point = False
            # new_point.priority = 1

            # # Things we don't have that are in the new Waypoint message
            # new_point.chi_d = 0 # Deseired course at this waypoint (rad)
            # new_point.chi_valid = True # Desired course valid (dubin or fillet paths)

            waypoints.append(new_point)

        # waypoints[-1].loiter_point = True
        # waypoints[-1].priority = 0

        self.last_exists = True
        self.last_waypoint = self.planned_waypoints[-1]

        if self.landing == True:
        #     waypoints[-1].loiter_point = False
        #     waypoints[-1].priority = 1
        #     waypoints[-1].landing = True

            self.last_exists = False

        # Send the service call with the desired mission type number
        # resp = waypoint_update(waypoints)
        for point in waypoints:
            self.wp_pub.publish(point)

        print("Waypoints sent")

        return True

    def clear_waypoints(self, req):
        new_point = Waypoint()
        new_point.w = [self.DEFAULT_POS[0], self.DEFAULT_POS[1], self.DEFAULT_POS[2]]
        new_point.clear_wp_list = True

        # Send the service call with the desired mission type number
        # resp = waypoint_update(waypoints)
        self.wp_pub.publish(new_point)

        # set the last waypoint to the origin
        self.last_waypoint = msg_ned(*self.DEFAULT_POS)
        print("Waypoints cleared")

        return True

    def update_search_params(self, req):
        """
        This function is called to change desired search params

        Parameters
        ----------
        req : uav_msgs.srv.UpdateSearchParams
        """
        self.planners["search"].height = req.height
        self.planners["search"].waypoint_distance = req.waypoint_distance

        return True

    def update_task_callback(self, req):
        """
        This function is called when the desired task is changed by the GUI. The proper mission is then called.

        Parameters
        ----------
        req : uav_msgs.srv.PlanMissionPoints
        """

        self.task = req.mission_type
        self.landing = False

        # change this to determine how tight of turns the path will create
        # 15*pi/16 seems reasonable for most tasks - large planning tasks will
        # take an excessive amount of time if this is too low.
        self.rrt.maxRelChi = 15 * np.pi / 16

        # interop server doesn't provide landing info
        if self.task != JudgeMission.MISSION_TYPE_LAND:
            (
                mission_type,
                obstacles,
                boundary_list,
                boundary_poly,
                waypoints,
            ) = tools.get_server_data(self.task, self.ref_pos)

        # Each task_planner class function should return a NED_list msg
        # These classes can be switched out depending on the desired functionality
        connect = False

        if self.task == JudgeMission.MISSION_TYPE_SEARCH:
            rospy.loginfo("SEARCH TASK BEING PLANNED")
            planned_points = self.planners["search"].plan(waypoints)

        elif self.task == JudgeMission.MISSION_TYPE_DROP:
            rospy.loginfo("PAYLOAD TASK BEING PLANNED")
            # be more picky about tight turns while performing the payload drop
            self.rrt.maxRelChi = 10 * np.pi / 16
            try:
                state_msg = rospy.wait_for_message("/state", State, timeout=10)
                wind = np.array([state_msg.wn, state_msg.we, 0.0])
            except rospy.ROSException as e:
                print("no state message received")
                wind = np.array([0.0, 0.0, 0.0])

            planned_points, drop_location = self.planners["payload"].plan(wind)
            rospy.set_param("DROP_LOCATION", drop_location)

        elif self.task == JudgeMission.MISSION_TYPE_LOITER:
            rospy.loginfo("LOITER PLANNER TASK BEING PLANNED")
            try:
                pos_msg = rospy.wait_for_message("/state", State, timeout=1)
                current_pos = msg_ned(
                    pos_msg.position[0], pos_msg.position[1], pos_msg.position[2]
                )
            except rospy.ROSException as e:
                print("Loiter - No State msg recieved")
                current_pos = msg_ned(*self.DEFAULT_POS)
            planned_points = self.planners["loiter"].plan(current_pos)

        # This is the task that deals with flying the mission waypoints. We call it objective to avoid confusion with the waypoints that are used to define the drop flight path or search flight path
        elif self.task == JudgeMission.MISSION_TYPE_WAYPOINT:
            rospy.loginfo("OBJECTIVE PLANNER TASK BEING PLANNED")
            planned_points = self.planners["objective"].plan(waypoints)
            connect = True

        elif self.task == JudgeMission.MISSION_TYPE_OFFAXIS:
            rospy.loginfo("OFFAXIS PLANNER TASK BEING PLANNED")
            planned_points = self.planners["offaxis"].plan(waypoints)

        elif self.task == JudgeMission.MISSION_TYPE_LAND:
            rospy.loginfo("LANDING PATH BEING PLANNED")
            landing_msg = req.landing_waypoints
            self.rrt.maxRelChi = 10 * np.pi / 16
            if len(landing_msg.waypoint_list) == 2:
                try:
                    pos_msg = rospy.wait_for_message("/state", State, timeout=1)
                    curr_altitude = pos_msg.position[2]
                except rospy.ROSException as e:
                    print("Landing - No State msg recieved")
                    curr_altitude = 0.0

                landing_wypts = utils.msg2wypts(landing_msg)
                planned_points = self.planners["landing"].plan(
                    landing_wypts, curr_altitude
                )
            else:
                planned_points = [msg_ned(*self.DEFAULT_POS)]
                print("No landing waypoints specified")
                print(len(landing_msg.waypoint_list))
            self.landing = True

        # I believe the emergent object is just within the normal search boundaries
        elif self.task == JudgeMission.MISSION_TYPE_EMERGENT:
            pass

        else:
            rospy.logfatal("TASK ASSIGNED BY GUI DOES NOT HAVE ASSOCIATED PLANNER")

        if self.last_exists == True:
            print("Planning from last waypoint of previous path")
            current_pos = self.last_waypoint
        else:
            print("Planning from origin")
            current_pos = msg_ned(*self.DEFAULT_POS)

        planned_points.insert(0, current_pos)

        print("Starting RRT to find full path")
        final_path = self.rrt.findFullPath(planned_points, connect=connect)

        # Convert python NED class to rospy ned msg
        wypts_msg = utils.wypts2msg(final_path, self.task)
        self.planned_waypoints = final_path

        self.init_mission_plotter()
        self.mp.addPathway(final_path, "Planned pathway")
        self.mp.show(block=True)

        print(wypts_msg)
        return wypts_msg


# Run the main planner
if __name__ == "__main__":

    rospy.init_node("main_planner", anonymous=True)
    test_planner = MissionPlanner()
    while not rospy.is_shutdown():
        rospy.spin()
