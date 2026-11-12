import math
import numpy as np
import pytest
from atsd_sensors.lidar_odometry import (match, transform, inverse, ScanOdometry,
                                        PoseHistory, SlipMonitor, cloud)


def scene():
    x=np.linspace(-2.,3.,110);y=np.linspace(-2.,2.,95)
    return np.concatenate((np.column_stack((x,np.full_like(x,2.))),
                           np.column_stack((np.full_like(y,3.),y)),
                           np.column_stack((x,np.full_like(x,-2.))),
                           np.column_stack((np.full_like(y,-2.),y))))


@pytest.mark.parametrize('motion',[(.08,.02,.03),(-.06,0.,-.04),(0.,0.,.08),(0.,0.,0.)])
def test_recovers_actual_motion_without_wheel_translation_prior(motion):
    reference=scene();current=transform(reference,inverse(motion))
    result=match(current,reference,yaw_hint=motion[2],budget_s=2.)
    assert result.valid, result
    assert np.linalg.norm(np.array(result.pose[:2])-motion[:2])<.015
    assert abs(result.pose[2]-motion[2])<.01


def test_parallel_corridor_is_degenerate_not_proof_of_stationary_robot():
    x=np.linspace(-4,4,140)
    walls=np.concatenate([np.column_stack((x,np.full_like(x,y))) for y in (-1.,1.)])
    r=match(walls,walls,budget_s=2.)
    assert not r.valid and r.reason=='degenerate'


def test_empty_invalid_and_over_budget_are_not_usable():
    assert not match(np.empty((0,2)),scene()).valid
    assert not match(scene(),scene(),budget_s=-1.).valid
    assert len(cloud([float('inf')]*455,-math.pi,2*math.pi/455,.05,12.))==0
    assert len(cloud([float('nan')]*455,-math.pi,2*math.pi/455,.05,12.))==0


def test_rolling_map_preserves_incremental_motion_and_resets_after_gap():
    engine=ScanOdometry(budget_s=2.)
    for i in range(12):
        pose=(i*.04,0.,i*.01)
        r=engine.update(transform(scene(),inverse(pose)),10+i*.2,.01)
        if i==0:assert not r.valid
        else:
            assert r.valid, r
            assert abs(math.hypot(*r.pose[:2])-.04)<.015
            assert abs(r.pose[2]-.01)<.01
    assert not engine.update(scene(),20.).valid


def test_history_interpolates_at_scan_time_and_does_not_extrapolate():
    h=PoseHistory();h.add(1.,(0,0,math.radians(179)));h.add(1.2,(.2,0,math.radians(-179)))
    p=h.at(1.1)
    assert abs(p[0]-.1)<1e-8 and abs(abs(p[2])-math.pi)<.001
    assert h.at(1.3) is None


def test_slip_requires_multiple_quality_measurements():
    s=SlipMonitor()
    assert not s.update(1.,(.04,0,0),(0,0,0))
    assert not s.update(1.2,(.04,0,0),(0,0,0))
    assert s.update(1.4,(.04,0,0),(0,0,0))
    s.reset()
    for i in range(10):
        assert not s.update(i*.2,(.04,0,.02),(.04,0,.02))


def test_moving_outliers_do_not_dominate_static_geometry():
    rng=np.random.default_rng(4);motion=(.06,0.,.02)
    current=transform(scene(),inverse(motion))
    current=np.concatenate((current,rng.uniform(-1,1,(30,2))))
    r=match(current,scene(),yaw_hint=.02,budget_s=2.)
    assert r.valid and abs(r.pose[0]-.06)<.02
