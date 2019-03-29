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