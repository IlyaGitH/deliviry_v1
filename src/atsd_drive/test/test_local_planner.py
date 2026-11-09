"""Safety and geometry tests of the production planner; no ROS dependencies."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

from atsd_drive.local_planner import LocalCfg, LocalPlanner, ScanSpace, advance, relative_pose
from atsd_drive.road_logic import RoadNav, Roads
from atsd_drive.protocol import nav_command_fresh

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from simulate_avoidance import collision, raycast, rectangle, run, scenarios


def space(segments=(), cfg=None, ranges=None):
    cfg = cfg or LocalCfg()
    if ranges is None:
        ranges = raycast((0., 0., 0.), segments)
    return ScanSpace(ranges, -math.pi, 2*math.pi/455, .05, 12., cfg)


def command(planner, scan, **kw):
    args = dict(pose=(0.,0.,0.), goal=(4.,0.), now=10., scan_age=.1)
    args.update(kw)
    return planner.decide(scan, **args)


def test_positive_inf_is_free_nan_and_negative_inf_unknown():
    c=LocalCfg()
    for value, clear in ((np.inf,True),(np.nan,False),(-np.inf,False),(0.,False)):
        s=space(cfg=c,ranges=np.full(455,value))
        assert (s.arc((0,0,0),1.,0.) is not None) == clear
    # +inf authorizes only the sensor range, not infinite travel.
    s=ScanSpace(np.full(455,np.inf),-math.pi,2*math.pi/455,.05,1.,c)
    assert s.arc((0,0,0),2.,0.) is None


def test_occluded_space_behind_wall_is_not_free():
    s=space(rectangle(2.,-1.,2.1,1.))
    assert s.clear([(0.,0.,0.)])
    assert not s.clear([(3.,0.,0.)])


def test_partial_front_scan_never_authorizes_reverse():
    c=LocalCfg()
    s=ScanSpace(np.full(180,np.inf),-math.pi/2,math.pi/180,.05,12.,c)
    assert s.arc((0,0,0),-.4,0.) is None


@pytest.mark.parametrize('changes,state', [({'scan_age':.46},'stale_scan'),
    ({'scan_age':-.01},'stale_scan'),({'odom_age':.36},'stale_odometry'),
    ({'measured_v':float('nan')},'invalid_input')])
def test_stale_or_invalid_sensor_stops(changes,state):
    d=command(LocalPlanner(),space(),**changes)
    assert (d.v,d.w,d.state)==(0.,0.,state)


def test_new_obstacle_revokes_cached_arc_and_brakes():
    p=LocalPlanner()
    d=command(p,space(),preferred=(.25,0.))
    assert d.v>0
    blocked=space(rectangle(.8,-1.,1.,1.))
    d=command(p,blocked,now=10.1,measured_v=.25)
    assert d.v<=0  # no continuing forward on an old trajectory
    # Already inside the stopping envelope: only a stop command is allowed.
    d=command(p,space(rectangle(.55,-1.,.6,1.)),now=10.2,measured_v=.25)
    assert (d.v,d.w,d.state)==(0.,0.,'braking_blocked')


def test_reverse_requires_free_rear_and_obeys_disable_switch():
    walls=rectangle(.9,-1.,1.,1.)
    cfg=LocalCfg(budget_s=2.,max_nodes=80)
    d=command(LocalPlanner(cfg),space(walls,cfg))
    assert d.v<0
    cfg.allow_reverse=False
    d=command(LocalPlanner(cfg),space(walls,cfg))
    assert d.v==d.w==0
    cfg.allow_reverse=True
    d=command(LocalPlanner(cfg),space(walls+rectangle(-.9,-1.,-.8,1.),cfg))
    assert d.v==d.w==0


def test_gear_change_waits_for_actual_stop():
    p=LocalPlanner(LocalCfg(budget_s=2.,max_nodes=80))
    command(p,space(),preferred=(.25,0.))
    wall=space(rectangle(.95,-1.,1.,1.))
    d=command(p,wall,now=10.1,measured_v=.10)
    assert (d.v,d.w,d.state)==(0.,0.,'gear_stop')
    d=command(p,wall,now=10.2,measured_v=0.)
    assert d.v<0


def test_stall_latches_and_never_starts_blind_reverse():
    p=LocalPlanner()
    command(p,space(),preferred=(.25,0.))
    d=command(p,space(),now=14.,preferred=(.25,0.))
    assert (d.v,d.w,d.state)==(0.,0.,'stalled')
    assert command(p,space(),now=15.).state=='stalled'
    p.reset()
    assert command(p,space(),now=16.,preferred=(.25,0.)).v>0


def test_reverse_distance_limit_and_no_pivot():
    cfg=LocalCfg(budget_s=2.,max_nodes=80)
    p=LocalPlanner(cfg)
    wall=space(rectangle(.9,-1.,1.,1.))
    d=command(p,wall)
    assert d.v<0 and abs(d.w)<=abs(d.v)/cfg.radius+1e-9
    p.reverse_distance=cfg.max_reverse_m
    p.commit=None
    d=command(p,wall,now=10.1)
    assert d.v==d.w==0


def test_full_swept_body_not_just_endpoint_or_center():
    cfg=LocalCfg()
    wall=rectangle(1.,-.2,1.05,.2)
    s=space(wall,cfg)
    assert s.arc((0,0,0),2.,0.) is None
    # Independently verify one safe arc has no body collision along its sweep.
    s=space(rectangle(2.,-.4,2.5,.4),cfg)
    endpoint=s.arc((0,0,0),1.,1.,planning=True)
    assert endpoint is not None
    for dist in np.linspace(0,1,101):
        assert not collision(advance((0,0,0),dist,1.), rectangle(2.,-.4,2.5,.4),cfg)


def test_strict_arrival_does_not_finish_beside_obstacle():
    nav=RoadNav(Roads(allow_direct=True));nav.filter.set_manual(0.,0.,0.);nav.plan((4.,0.))
    nav.filter.X[:]=[4.,2.,0.]
    _,_,done=nav.tick(None,strict_arrival=True)
    assert not done and nav.active


def test_detour_disables_lane_pose_snap_and_old_lane_cache():
    nav=RoadNav(Roads(allow_direct=True));nav.filter.reset(0.,1.,0.);nav.plan((10.,0.))
    before=nav.filter.X.copy()
    nav.last_lane={'offset':-1.,'angle':0.,'width':3.,'lines':2}
    nav.tick(nav.last_lane,correct_lane=False)
    assert np.array_equal(before,nav.filter.X)
    assert nav.last_lane is None


def test_frame_transform_and_delayed_command():
    assert np.allclose(relative_pose((1.,3.,math.pi/2),(1.,2.,math.pi/2)),(1.,0.,0.))
    assert nav_command_fresh(.1,-.15,.15)
    for age in (-1.,.251,float('nan')):
        assert not nav_command_fresh(age,-.15,.15)
    assert not nav_command_fresh(.1,float('inf'),0.)


@pytest.mark.parametrize('name', ['box','left_blocked','right_blocked','slalom','dead_end'])
def test_closed_loop_escape_scenarios(name):
    # Fixed node count makes regression tests deterministic across CPU speeds.
    cfg=LocalCfg(max_nodes=100,budget_s=5.)
    result=run(name,scenarios()[name],100.,cfg)
    assert not result['collision'],result
    assert result['reached'],{k:v for k,v in result.items() if k!='trace'}
    if name=='dead_end':
        assert result['reverse_m']>.5
    for t,x,y,yaw,v,w,state in result['trace']:
        assert abs(w)<=abs(v)/cfg.radius+1e-8


@pytest.mark.parametrize('name,expected', [('scan_loss','stale_scan'),('odom_loss','stale_odometry'),
                                         ('invalid_ranges','braking_blocked'),('moving_front','avoid')])
def test_closed_loop_sensor_failures_and_obstacle_appearance(name,expected):
    r=run(name,scenarios()[name],60.,LocalCfg(max_nodes=100,budget_s=5.))
    assert r['reached'] and not r['collision']
    assert r['states'].get(expected,0)>0


def test_attempt_timeout_is_bounded_even_while_moving():
    p=LocalPlanner();p.avoid_started=1.
    d=command(p,space(),now=122.)
    assert d.state=='escape_timeout' and d.v==d.w==0


@pytest.mark.parametrize('metadata', [(float('nan'),.1,.05,12.),(0.,0.,.05,12.),(0.,.1,.05,float('inf'))])
def test_invalid_scan_metadata_cannot_authorize_motion(metadata):
    s=ScanSpace(np.full(455,np.inf),*metadata,LocalCfg())
    assert not s.clear([(0.,0.,0.)])


def test_first_reverse_command_waits_for_existing_forward_motion():
    p=LocalPlanner(LocalCfg(max_nodes=80,budget_s=5.))
    d=command(p,space(rectangle(.95,-1.,1.,1.)),measured_v=.1)
    assert d.state=='gear_stop' and d.v==d.w==0.


@pytest.mark.parametrize('spacing', [.8, .9, 1.0])
def test_real_packet_raster_in_empty_space_never_interrupts_motion(spacing):
    from atsd_sensors.lidar_logic import bins_from_packets

    for phase in np.linspace(0, 12*spacing, 60, endpoint=False):
        starts = np.arange(phase, phase+360, 12*spacing) % 360
        ranges, _, _ = bins_from_packets(
            starts, (starts+11*spacing) % 360, [bytes(36)]*len(starts), 455)
        decision = command(LocalPlanner(), space(ranges=ranges), preferred=(.25, 0.))
        assert decision.state == 'following', (spacing, phase, decision)
        assert decision.v == .25
