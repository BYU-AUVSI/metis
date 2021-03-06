
class ObjectivePointsPlanner():
    
    def __init__(self, obstacles):
        pass
    
    def plan(self, waypoints):
        """ Plans the path to hit the designated flight points

        Since our architecture is such that all mission tasks (payload, search) take in parameters and then determine points they want to fly through, this just passes the mission waypoints straight through.
        A flyable path is constructed later using a path planning algorithm.

        Parameters
        ----------
        waypoints : list of NED class
            The objective points that interop wants us to fly through
        
        Returns
        -------
        waypoints : list of NED class
            The objective points that we want to fly through.

        """
        return(waypoints)