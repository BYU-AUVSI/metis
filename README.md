# Installation

To run Metis as a standalone package, just pull the latest code from the repository.

# Operation

For testing, Metis currently has a fake_interop node that supplies waypoints that would normally be supplied by the interop server. 

Metis can be run with `roslaunch metis fake_interop.launch`, which will run the fake interop node, the mission planner, node, and the path planner node. At the time of this writing, the path planner node is still not fully functional.

The various missions can be called with the `/plan_mission` rosservice call. The only argument for this service call is the mission that should be run. 

The plan_mission service call will return a list of the major waypoints that the aircraft needs to fly thorough. These may be the drop location and points leading to the drop location, points that create a lawnmower path over the search area, points that allow detection of objects outside the mission area, or objective waypoints given by the interop server.

## Planner Commands

`rosservice call /plan_mission 0`: Waypoint Mission
`rosservice call /plan_mission 1`: Payload Mission
`rosservice call /plan_mission 2`: Search Mission
`rosservice call /plan_mission 6`: Offaxis Detection
`rosservice call /plan_mission 7`: Loiter Mission

Note that the Loiter mission currently (as of 5/17/19) only passes the objective waypoints through. The desired behavior of that mission is still being determined.

## Path Planning

The path needed to fly any mission can be called with the `/plan_path` call. Syntax is identical to the planner commands (i.e. `rosservice call /plan_path <num>` where `<num>` is the desired mission. The path planner internally calls the `/plan_mission` service call to get the main waypoints and then uses RRT to find the best path that connects all the waypoints in the specified order.

This functionality is currently not working (as of 5/17/19)
