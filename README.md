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

# Notes

Current behavior is as follows
```
user:~$ rosservice call /plan_path 0
ERROR: Incompatible arguments to call service:
Not enough arguments:
 * Given: [0]
 * Expected: ['mission_type', 'landing_waypoints']
Provided arguments are:
 * 0 (type int)

Service arguments are: [mission_type landing_waypoints.waypoint_list]
```

`landing_waypoints` is a set of ROS messages (type `NED_list`, comprised of `NED_pt`)from `uav_msg`.

The `NED_pt` message is as follows:

`NED_pt.msg`
```
#General points, used by the mission planner and GUI

float64 N
float64 E
float64 D
uint16 task #Determines the task associated with the waypoint
```

where `uint16 task` is an integer as defined by the `JudgeMission` message.

`JudgeMission.msg`
```
# Representation of mission data received from the judges.

uint8 mission_type # An integer to tell the UAV which mission this message represents. Types described by constants below.
uint8 MISSION_TYPE_WAYPOINT = 0
uint8 MISSION_TYPE_DROP = 1
uint8 MISSION_TYPE_SEARCH = 2
uint8 MISSION_TYPE_OTHER = 3
uint8 MISSION_TYPE_LAND = 4
uint8 MISSION_TYPE_EMERGENT = 5
uint8 MISSION_TYPE_OFFAXIS = 6
uint8 MISSION_TYPE_LOITER = 7 #Custom to give a loiter mission

bool now # Whether the plane should begin executing this mission immediately (true), or wait for confirmation (false)

OrderedPoint[] waypoints # These are the primary mission waypoints.
OrderedPoint[] boundaries # An array of GPS points that denote the flight boundaries of the competition.
StationaryObstacle[] stationary_obstacles # The static obstacles that will be placed throughout the competition field.
```

To see the proper syntax for `rosservice call /plan_path <args>`, simply enter `rosservice call /plan_path` into the terminal and tab complete until it autofills a template for what the message looks like. Doing so will provide the following:

```
user:~$ rosservice call /plan_path "mission_type: 0
landing_waypoints:
  waypoint_list:
  - N: 0.0
    E: 0.0
    D: 0.0
    task: 0" 
```

To see the mission specified by `FakeInteropElberta.py`, you can run that file (`python FakeInteropElberta.py`) and then in a separate terminal call 

```
user:~/ros/uas_ws/src/metis/testingCode$ rosservice call /get_mission_with_id "mission_type: 0" 
```
This returns:
```
mission: 
  mission_type: 0
  now: False
  waypoints: 
    - 
      point: 
        latitude: 39.983031
        longitude: -111.991051
        altitude: 30.0
        chi: 0.0
      ordinal: 1
    - 
      point: 
        latitude: 39.983449
        longitude: -111.99216
        altitude: 45.0
        chi: 0.0
      ordinal: 2
  boundaries: 
    - 
      point: 
        latitude: 39.985229
        longitude: -111.993796
        altitude: 0.0
        chi: 0.0
      ordinal: 1
    - 
      point: 
        latitude: 39.981026
        longitude: -111.986903
        altitude: 0.0
        chi: 0.0
      ordinal: 2
    - 
      point: 
        latitude: 39.977468
        longitude: -111.993858
        altitude: 0.0
        chi: 0.0
      ordinal: 3
    - 
      point: 
        latitude: 39.983239
        longitude: -112.000138
        altitude: 0.0
        chi: 0.0
      ordinal: 4
  stationary_obstacles: 
    - 
      point: 
        latitude: 39.983258
        longitude: -111.994228
        altitude: 0.0
        chi: 0.0
      cylinder_height: 60.0
      cylinder_radius: 20.0
    - 
      point: 
        latitude: 39.981556
        longitude: -111.993697
        altitude: 0.0
        chi: 0.0
      cylinder_height: 40.0
      cylinder_radius: 10.0
```

Still the wrong format for providing to `rosservice call /plan_path` because the message is not a NED_list, but this is progress, people!