import pytest
import sys
sys.path.append('../src')

import numpy as np

from metis.location import GPSWaypoint

class TestTools:
    def test_FakeInteropElberta(self):
        """
        GPS positions and data are taken from the sample mission, 
        FakeInteropElberta.
        """
        # The reference position
        ref = GPSWaypoint(39.98304881111, -111.9903776, 0)

        # Waypoints
        pt1 = GPSWaypoint(39.983031, -111.991051, 30)
        pt2 = GPSWaypoint(39.983449, -111.992160, 45)
        pt3 = GPSWaypoint(39.982648, -111.993338, 45)
        pt4 = GPSWaypoint(39.981005, -111.992602, 45)
        pt5 = GPSWaypoint(39.981912, -111.989530, 45)
        pt6 = GPSWaypoint(40.266946, -111.632552, 0)

        # Boundaries
        bnd1 = GPSWaypoint(39.985229, -111.993796, 0)
        bnd2 = GPSWaypoint(39.981026, -111.986903, 0)
        bnd3 = GPSWaypoint(39.977468, -111.993858, 0)
        bnd4 = GPSWaypoint(39.983239, -112.000138, 0)

        # The obstacles
        ob1 = GPSWaypoint(39.983258, -111.994228, 0)
        ob2 = GPSWaypoint(39.981556, -111.993697, 0)

        # Test waypoints
        assert np.array_equal(pt1.ned_from(ref).ned, np.array([[-1.9774270580937272, -57.51845492956573, -30.0]]))
        assert np.array_equal(pt2.ned_from(ref).ned, np.array([[44.436219158444004, -152.24275217249115, -45.0]]))
        assert np.array_equal(pt3.ned_from(ref).ned, np.array([[-44.49958466049114, -252.86395114645126, -45.0]]))
        assert np.array_equal(pt4.ned_from(ref).ned, np.array([[-226.93073988523403, -190.0027176375646, -45.0]]))
        assert np.array_equal(pt5.ned_from(ref).ned, np.array([[-126.22467709169122, 72.39892740438708, -45.0]]))

        # Test boundaries
        assert np.array_equal(bnd1.ned_from(ref).ned, np.array([[242.08140457437887, -291.9732487769884, 0.0]]))
        assert np.array_equal(bnd2.ned_from(ref).ned, np.array([[-224.59560770270807, 296.7916024552834, 0.0]]))
        assert np.array_equal(bnd3.ned_from(ref).ned, np.array([[-619.6553920373733, -297.30244304492277, 0.0]]))
        assert np.array_equal(bnd4.ned_from(ref).ned, np.array([[21.163119796055742, -833.6819483428043, 0.0]]))

        # Test obstacles
        assert np.array_equal(ob1.ned_from(ref).ned, np.array([[23.234244610397234, -328.88079281265095, 0.0]]))
        assert np.array_equal(ob2.ned_from(ref).ned, np.array([[-165.74794803299733, -283.5326177308099, 0.0]]))


        