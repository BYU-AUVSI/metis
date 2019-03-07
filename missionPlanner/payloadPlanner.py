



class PayloadPlanner():
    """
    inputs: drop gps location, wind, obstacles
    outputs: X number of waypoints with a single drop waypoint
    """
    def __init__(self,dropLocation,wind,obstacles):
        """
        initializes global variables
        """
        self.dropLocation = dropLocation            # gps location of where on the ground we want to hit
        self.wind = wind                            # current wind vector
        self.obstacles = obstacles                  # competition obstacles
        self.command_time = 0.15                    # seconds between command to drop and UGV leaves plane
        self.NED_open_parachute                     # NED location we want the parachute to be fully open
        self.NED_command                            # NED location we want the command to happen

    def plan():
        """
        function called by mainplanner that returns bombdrop waypoint
        """
        open = openParachute()
        self.NED_command =


    def openParachute():
        """
        calculates desired location for the parachute to open
        """
        self.NED_open_parachute =

    def closedParachute():
        return NED_


    def supportingPoints():
        pass
