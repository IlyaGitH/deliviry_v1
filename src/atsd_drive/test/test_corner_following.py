"""Corner following through the real local checker and motor acceleration model."""
import math
import numpy as np
import pytest
from atsd_drive.road_logic import Follower, FollowCfg, rounded_route
from atsd_drive.local_planner import LocalCfg, LocalPlanner, ScanSpace, advance


@pytest.mark.parametrize('side', [1., -1.])
def test_right_angle_enters_early_and_exits_on_road_without_stop_start(side):
    f = Follower(FollowCfg(turn_radius=.5))
    f.start([(0.,0.), (4.,0.), (4.,side*4.)])
    cfg = LocalCfg(radius=.5, front=.64, rear=.26, half_width=.2, budget_s=.2)
    planner = LocalPlanner(cfg)
    pose = (0.,0.,0.)
    v = w = 0.
    trace = []
    for i in range(600):
        preferred_v, preferred_w, done = f.step(pose, True, None, True)
        if done:
            break
        dx, dy = f._lookahead(pose[:2], 4.5)-np.asarray(pose[:2])
        cs, sn = math.cos(pose[2]), math.sin(pose[2])
        goal = (cs*dx+sn*dy, -sn*dx+cs*dy)
        space = ScanSpace(np.full(455,np.inf), -math.pi, 2*math.pi/455, .05, 12., cfg)
        d = planner.decide(space, (0.,0.,0.), goal, i*.1, .1, v, w, 0., pose,
                           (preferred_v, preferred_w), short_reverse_allowed=f.i==len(f.path)-2)
        old_v = v
        v += float(np.clip(d.v-v, -.035, .035))
        k = d.w/d.v if abs(d.v)>.001 else w/old_v if abs(old_v)>.001 else 0.
        w = v*k
        pose = advance(pose, v*.1, k)
        trace.append((*pose, v, w, d.state))
    assert done
    assert all(t[5] == 'following' for t in trace)
    assert max(t[0] for t in trace) < 4.15
    assert max(abs(t[0]-4.) for t in trace if side*t[1]>1.2) < .15
    first_turn = next(t for t in trace if abs(t[4])>.05)
    assert first_turn[0] < 3.3
    assert max(t[3] for t in trace) > .75
    assert all(t[3] > .1 for t in trace if 2.5 < t[0] and side*t[1] < 2.)


def test_short_corner_does_not_force_radius_below_robot_limit():
    path = [(0.,0.),(.3,0.),(.3,.3)]
    assert rounded_route(path, .8) == path
