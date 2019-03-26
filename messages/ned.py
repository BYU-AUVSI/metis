class msg_ned:
    def __init__(self, north=0., east=0., down=0., radius=0.):
        self.n = north  # North position
        self.e = east  # East position
        self.d = down  # Down position. Height for obstacles
        self.r = radius  # Radius for obstacles. Not sure if there is another message type for this
