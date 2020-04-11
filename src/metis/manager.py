# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

import functools
import logging
import warnings

import numpy as np

from metis import Plan
from metis.location import Waypoint, BoundaryPoint, CircularObstacle
from metis.planners import (
    LoiterPlanner,
    LandingPlanner,
    ObjectivePointsPlanner,
    OffaxisPlanner,
    PayloadPlanner,
    SearchPlanner,
)
from metis.rrt import get_rrt, Config

_module_logger = logging.getLogger(__name__)

class MissionManager(object):
    """
    A manager class that centralizes all path planning processes.

    Attributes
    ----------
    target_height : float, optional
        A default height for the aircraft when not flying waypoints.
        It is a positive floating point value in meters.
        It is always the same as the height value in `default_pos`;
        modifying `default_pos` will affect `target_height`.
    default_pos : metis.messages.msg_ned
        A default fallback position for the aircraft to loiter about when no \
        mission is being performed.
    current_pos : metis.messages.msg_ned
        The current position of the aircraft.
    """
    _logger = _module_logger.getChild('RRT')

    def __init__(self, mission, target_height=35.):
        """
        Initializes the MissionManager.

        Parameters
        ----------
        mission : metis.core.Mission
            A Mission object, representing a complete AUVSI mission.
        target_height : float, optional
            A default height for the aircraft when not flying waypoints.
            It is a positive floating point value in meters (default 35).
        """
        self.mission = mission
        self.default_pos = Waypoint(0., 0., -target_height)
        self.planners = {}
        if self.mission:
            self._init_planners(self.mission)
        else:
            warnings.warn("MissionManager created without mission object; no planners are initialized.", RuntimeWarning)
        self.plans = []
        self.current_pos = Waypoint(0., 0., -target_height)

    @property
    def target_height(self):
        """
        A target default height for the plane when not flying specific,
        pre-planned waypoints, in meters.
        """
        return -self.default_pos.d

    @target_height.setter
    def target_height(self, height):
        """
        A target default height for the plane when not flying specific,
        pre-planned waypoints, in meters.
        """
        if height < 0:
            raise ValueError("Target height is less than 0!")
        else:
            self.default_pos.d = float(-height)

    def _approve_plan(self, plan):
        self._logger.info("Plan approved.")
        self.plans.append(plan)

    def _apply_rrt(self, planned_points, config=None, connect=False, rrt_type='fillet'):
        """
        Private function that applies the RRT algorithm to generated paths.

        Parameters
        ----------
        planned_points : list of metis.messages.msg_ned
            The points of the mission that must be hit.
        config : metis.rrt.rrt_base.Config
            Configuration object that contains, among other thigns, the
            tightness of turns that the RRT algorithm should plan for
            (default (15/16)*pi).
        connect : bool, optional
            Whether RRT should use the `connect` flag.

        Returns
        -------
        plan : metis.core.Plan
            Returns a Plan object representing the final plan.
        """
        # RRT = get_rrt('straight')
        # RRT = get_rrt('fillet')
        # RRT = get_rrt('dubins')
        RRT = get_rrt(rrt_type)
        if config:
            rrt = RRT(self.mission, config=config)
        else:
            rrt = RRT(self.mission)

        if self.plans:
            self._logger.info("Planning from last waypoint of previous path")
            current_pos = self.plans[-1].last_waypoint
        else:
            self._logger.info("Planning from origin")
            current_pos = self.default_pos
            # FIXME: Can we update to get the actual position of the plane? And
            # get the current heading, too.

        planned_points.insert(0, current_pos)
        final_path = rrt.find_full_path(planned_points, connect=connect)
        plan = Plan(self.mission, waypoints=final_path, callback=self._approve_plan)
        return plan

    def plan(self, task, **kwargs):
        """
        Performs the planning actions associated with a certain task.

        Parameters
        ----------
        task : str
            The task to be performed. Valid values are:
                `objective` - Waypoint Mission
                `payload` - Payload Drop
                `search` - Search Mission
                `None` - Other
                `landing` - Landing
                `None` - Emergent
                `offaxis` - Offaxis Detection
                `loiter` - Loiter
            Ensure these values stay current with those in JudgeMission.

        Returns
        -------
        plan : metis.core.Plan
            A plan object representing the planned path.
        """
        method_name = 'plan_' + str(task)
        method = getattr(self, method_name, self.plan_invalid)
        return method(**kwargs)

    def plan_invalid(self):
        raise RuntimeError("No planners exist for the specified task.")

    def plan_landing(self, waypoints=None, altitude=0.0):
        """
        Plans the landing.

        Parameters
        ----------
        waypoints : list of metis.messages.msg_ned, optional
            The set of waypoints defining the landing direction. Raises an
            error if not provided.
        altitude : float, optional
            The current altitude of the plane in meters; the altitude to plan
            the approach from (default 0).
        """
        if not waypoints:
            raise ValueError("No waypoints provided to the landing planner.")
        if altitude <= 0.0:
            warnings.warn("Current altitude is 0 meters or less. Are you sure this is accurate?", RuntimeWarning)
        planned_points = self.planners["landing"].plan(waypoints, altitude)
        # TODO: Planner needs to plan points from current position to start of
        # approach, and not do an RRT on final.
        # max_rel_chi=10*np.pi/16
        plan = self._apply_rrt(planned_points)
        return plan

    def plan_loiter(self, current_pos=None):
        """
        Plans the loiter mission.

        Parameters
        ----------
        current_pos : metis.messages.msg_ned, optional
            The current position of the aircraft to plan a path from. If none
            is provided, planning is performed from the MissionManager's set
            default position.
        """
        # if current_pos is None:
        #     current_pos = self.default_pos
        # planned_points = self.planners["loiter"].plan(current_pos)
        # plan = self._apply_rrt(planned_points)
        # return plan
        raise NotImplementedError

    def plan_objective(self):
        planned_points = self.planners["objective"].plan()
        plan = self._apply_rrt(planned_points, connect=True, rrt_type='dubins')
        return plan

    def plan_offaxis(self):
        # planned_points = self.planners["offaxis"].plan()
        # plan = self._apply_rrt(planned_points)
        # return plan
        raise NotImplementedError

    def plan_payload(self, wind=np.array([0.0, 0.0, 0.0])):
        """
        Plans the payload drop portion of the mission.

        Parameters
        ----------
        wind : numpy.array
            An array of length 3 containing the wind as NED.
        """
        # wind = np.array([0.0, 0.0, 0.0])
        self._logger.info('plan_payload called')
        planned_points, drop_location = self.planners["payload"].plan(wind)
        # Be more picky about tight turns while performing the payload drop
        # max_rel_chi=10*np.pi/16
        # config = Config(max_rel_chi=np.radians(90))
        plan = self._apply_rrt([planned_points[0]], rrt_type='dubins')
        plan.waypoints += planned_points[1:]
        plan.params["drop_location"] = drop_location
        return plan

    def plan_search(self):
        planned_points = self.planners["search"].plan()
        config = Config(max_rel_chi=np.radians(110), iterations=3)
        plan = self._apply_rrt(planned_points, config, rrt_type='fillet')
        return plan


    def _init_planners(self, mission):
        """
        Initializes the planner classes for the MissionManager.

        Available planner keywords are:
            `landing`
            `loiter`
            `objective`
            `offaxis`
            `payload`
            `search`

        >>> mm = MissionManager(mission)
        >>> waypoints = mm.planners["objective"].plan()
        """
        self.planners = {
            "landing": LandingPlanner(mission),
            "loiter": LoiterPlanner(mission),
            "objective": ObjectivePointsPlanner(mission),
            "offaxis": OffaxisPlanner(mission),
            "payload": PayloadPlanner(mission),
            "search": SearchPlanner(mission),
        }

    def set_search_params(self, height, waypoint_distance):
        self.planners["search"].height = height
        self.planners["search"].waypoint_distance = waypoint_distance
