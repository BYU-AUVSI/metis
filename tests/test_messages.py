import pytest
from metis.messages import msg_ned

class TestMsgNed:
    def test_msgned(self):
        n = -104.671952645
        e = 73.1074556701
        d = -113.44
        r = 0.0
        a = msg_ned(n, e, d, r)
        assert n == a.n
        assert e == a.e
        assert d == a.d
        assert r == a.r
        c = msg_ned(n, e, d, r)
        assert a == c
        
        b = msg_ned(511.06542072, 18.5627842528, -113.44, 0.0)
