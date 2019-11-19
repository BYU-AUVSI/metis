# Installation

To run Metis as a standalone package, just pull the latest code from the repository.

# State of the Package

We reverted rosplane to "Vanilla," meaning the base MAGICC lab code. Previous teams had  modified rosplane itself, building in functionality such as "path_manager" and "path_planner". These are now gone, so a substitute must be created in order for this package to begin working again.

To get the skeleton running (but without functionality), you need at least the following repositories:

- rosplane
- ros_groundstation (AUVSI-SUAS-2019 branch)
- metis

To run, execute the following commands (each will need its own terminal):

```
roslaunch rosplane_sim fixedwing.launch             # Launches the simulation. Make sure to hit "play" in Gazebo.
roslaunch ros_groundstation gs_fixedwing.launch     # Launches groundstation
roslaunch metis fake_interop.launch                 # Provides a mock interop server for testing
```

# Operation

For testing, Metis currently has a fake_interop node that supplies waypoints that would normally be supplied by the interop server. 

Metis can be run with `roslaunch metis fake_interop.launch`, which will run the fake interop node, the mission planner, node, and the path planner node. At the time of this writing, the path planner node is still not fully functional.

The various missions can be called with the `/plan_path` rosservice call. The arguments for this service call are the mission that should be run and the arguments associated with the PlanMissionPath service call. 

The plan_mission service call will return a list of the major waypoints that the aircraft needs to fly thorough. These may be the drop location and points leading to the drop location, points that create a lawnmower path over the search area, points that allow detection of objects outside the mission area, or objective waypoints given by the interop server.

## Planner Commands

`rosservice call /plan_path 0`: Waypoint Mission

`rosservice call /plan_path 1`: Payload Mission

`rosservice call /plan_path 2`: Search Mission

`rosservice call /plan_path 6`: Offaxis Detection

`rosservice call /plan_path 7`: Loiter Mission

Note that the Loiter mission currently (as of 5/17/19) only passes the objective waypoints through. The desired behavior of that mission is still being determined.

Once the waypoints are planned, use the ```approved_path``` command to pass the waypoints to the plane.
