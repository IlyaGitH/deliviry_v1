import math
import numpy as np
import pytest
from atsd_drive.gps_gate import GpsGate
from atsd_drive.road_logic import PoseFilter


def test_requires_time_and_several_fixes():
    g = GpsGate()
    for t in range(5):
        assert not g.push(0.,0.,2.,float(t))
    assert g.push(0.,0.,2.,5.)
    g = GpsGate()
    assert not g.push(0,0,2,0)
    assert not g.push(0,0,2,6)  # gap restarts qualification


@pytest.mark.parametrize('x,sigma,reason', [(float('nan'),2,'poor_quality'),
    (0,20,'poor_quality'),(0,float('inf'),'poor_quality'),(100,2,'jump')])
def test_invalid_or_distant_fix_never_qualifies(x,sigma,reason):
    g = GpsGate()
    for t in range(10):
        assert not g.push(x,0,sigma,t,estimate=(0,0,0))
    assert g.reason == reason


def test_jump_and_loss_require_new_stable_series():
    g = GpsGate()
    for t in range(6):
        g.push(0,0,1,t)
    assert not g.push(20,0,1,6)
    assert g.reason == 'jump'
    for t in range(7,12):
        assert not g.push(0,0,1,t)
    assert g.push(0,0,1,12)
    assert not g.push(0,0,1,20)


def test_motion_consistency_uses_odometry_without_assuming_north_alignment():
    g = GpsGate()
    for t in range(20):
        accepted = g.push(t*.8,0,1,t,odom=(0,t*.8,0))
    assert accepted


def test_qualified_gps_releases_manual_gradually_then_dead_reckons():
    f = PoseFilter(); f.set_manual(0,0,0)
    f.qualified_gps(10,0,1)
    assert not f.manual and f.gnss == 'ok'
    assert 0 < f.X[0] <= .25
    for _ in range(100):
        before = f.X.copy()
        f.qualified_gps(10,0,1)
        assert math.dist(f.X[:2],before[:2]) <= .25000001
        assert np.linalg.eigvalsh(f.P).min() >= -1e-10
    assert abs(f.X[0]-10) < .01
    f.gnss_lost(); before=f.X.copy(); f.predict(1,0)
    assert f.gnss == 'none' and math.isclose(f.X[0]-before[0],1.)
