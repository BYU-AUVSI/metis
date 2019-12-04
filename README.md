# Installation

To run Metis as a standalone package, just pull the latest code from the repository.

# API

## Available ROS Services

`/clear_wpts`  
`/plan_path`  
`/update_search_params`  

## Available ROS Topics

`/current_task`

## Required ROS Services

`/waypoint_path`  
`/get_mission_with_id`  

## Required ROS Topics

# State of the Package

We reverted rosplane to "Vanilla," meaning the base MAGICC lab code. 

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

The plan_mission service call will return a list of the major waypoints that the aircraft needs to fly through. These may be the drop location and points leading to the drop location, points that create a lawnmower path over the search area, points that allow detection of objects outside the mission area, or objective waypoints given by the interop server.

## Planner Commands

`rosservice call /plan_path 0`: Waypoint Mission

`rosservice call /plan_path 1`: Payload Mission

`rosservice call /plan_path 2`: Search Mission

`rosservice call /plan_path 6`: Offaxis Detection

`rosservice call /plan_path 7`: Loiter Mission

Note that the Loiter mission currently (as of 5/17/19) only passes the objective waypoints through. The desired behavior of that mission is still being determined.

Once the waypoints are planned, use the ```approved_path``` command to pass the waypoints to the plane.

---

### Planner Notes

# Mission Planner

`mainPlanner.py` This is a description about the mission planner


## Objective Points Planner
This is a description about this planner

## Search Planner
This is a description about this planner

## Loiter Planner
This is a description about this planner

## Payload Planner

The payload path planner is calculated using two distinct regions. The final path plan is shown below
with the boundaries and obstacles in blue, payload path in red, and calculated plane waypoints
in green.

![server overview](docs/source/img/payloadplan.png)

### Commanded Release to Parachute Open
The first region is between when the command to release is given to the servo to when
the parachute is fully open.
**Assumptions:**
1. Time delay between the command release to the servo and the bay door opening is constant and known
2. Time delay between the bay door opening is constant and known (through experimentation)
3. Height difference is calculated between when the bay door opens and when the parachute opens
4. Wind is steady state (no gusts)
5. The only force acting on the payload is gravity (no aerodynamic drag, etc.)

### Parachute Open to Target
The second region is between when the parachute opens to when the payload hits the ground target
**Assumptions:**
1. Payload descends down at a constant rate which is known (through experimentation)
2. Payload moves in the North and East directions at the speed of the wind
3. No aerodynamic drag, acceleration

### Supporting Waypoints
The final step of  the payload planner is to create supporting waypoints (green triangles) so that the plane is flying in a straight line when it drops the payload.  
The planner first tries to fly directly into the wind. If that commanded chi angle
would hit an obstacle or go out of bounds, it iterates on the command chi angle by adding 15 degrees until it finds a successful waypoint path.

## Current Task
This is a description about this planner

---

# Path Planner
`pathPlannerBase.py` is a ROS Node for planning paths between a list of waypoints. 
The planner can use either `rrt.py` or `rrtstar.py`. 

## RRT
`rrt.py` uses the RRT algorithm with small modifications for UAV flight. Adding new
nodes takes into account the relative chi angle and incline angle in order for the 
paths to be flyable. 

### Changing Altitude Between Waypoints
Most RRT algorithms that solve for paths in 3D space add points anywhere in the flyable space.
This method does find paths, but takes many iterations because most added points are in spaces 
that would logically never be flown. By assuming the vehicle will ascend and descend at the beginning
of its path, the space where points are added can be simplified back to essentially a 2D plane.
This allows for shorter solve times, and for more paths to be tested. 

---

# Coding Standard

This package adheres to the numpydoc style of documentation using Python docstrings.
Additionally, the code is formatted with the `black` code formatter provided by the 
Python Software Foundation.

# Notes

Scripts to start nodes ought to be placed in /bin. Regardless of the language they are
written in, be sure to include the shebang at the top of the file; for example, in Python:

```
#! /usr/bin/env python
```

The file also needs to be marked as executable.

```
chmod u+x <filename>
```

---

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