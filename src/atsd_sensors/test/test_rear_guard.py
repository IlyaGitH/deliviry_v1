import math
from atsd_sensors.rear_guard import RearGuard


def feed(g, t, ranges=(.3,.3), moving=True):
    return g.update(ranges, math.pi-.01, .02, .02, 12., t, moving)


def test_requires_one_second_and_motion_and_rearms():
    g = RearGuard()
    for i in range(10):
        assert not feed(g, i/10)
    assert feed(g, 1.)
    assert not feed(g, 1.1)
    assert not feed(g, 1.2, moving=False)
    for i in range(10):
        assert not feed(g, 2+i/10)
    assert feed(g, 3.)


def test_gaps_unknown_and_outside_body_do_not_trigger():
    for ranges in ((math.inf,math.inf), (.6,.6), (float('nan'),.3), (.3,)):
        g = RearGuard()
        for i in range(20):
            assert not feed(g, i/10, ranges)
    g = RearGuard()
    assert not feed(g, 0)
    assert not feed(g, 2)  # missing scans never count as continuous visibility
    assert not feed(g, 2.1)


def test_forward_objects_are_not_rear_intrusion():
    g = RearGuard()
    for i in range(20):
        assert not g.update([.3,.3], 0, .01, .02, 12, i/10, True)
