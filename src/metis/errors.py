# -*- coding: utf-8 -*-
# Copyright 2018-2019 John Akagi and Jacob Willis
# Copyright 2019-2020 Sequoia Ploeg

"""
The exceptions and errors of the metis package.
"""

class Error(Exception):
    pass

class InvalidAltitudeError(Error):
    """
    This error is raised when in invalid altitude value is encountered (for
    example, a negative altitude).
    """
    pass

class WaypointError(Error):
    """
    This error is raised when invalid or missing waypoints are encountered.
    """
    pass

class InvalidCallbackError(Error):
    """
    This error is raised when a callback is called that has not been initialized.
    """
    pass