"""Exercise production ROS callbacks with only transport/message types stubbed."""
import importlib.util
from pathlib import Path
import sys
import types
from unittest.mock import Mock

import numpy as np
import pytest

from atsd_drive.gps_gate import GpsGate
from atsd_drive.local_planner import Decision, LocalPlanner, ScanSpace
from atsd_drive.road_logic import RoadNav, Roads

NS = types.SimpleNamespace


@pytest.fixture
def adapter(monkeypatch):
    class Message:
        def __init__(self, **kw): self.__dict__.update(kw)
    modules={
        'rclpy': {},'rclpy.node':{'Node':object},'rclpy.qos':{'qos_profile_sensor_data':object()},
        'geometry_msgs.msg':{n:Message for n in ('PoseStamped','PoseWithCovarianceStamped','TransformStamped','Twist','TwistStamped','TwistWithCovarianceStamped')},
        'nav_msgs.msg':{n:Message for n in ('Odometry','Path')},
        'sensor_msgs.msg':{n:Message for n in ('Imu','LaserScan','NavSatFix','NavSatStatus')},
        'std_msgs.msg':{n:Message for n in ('Float32','String')},'tf2_ros':{'TransformBroadcaster':Message}}
    for name,attrs in modules.items():
        mod=types.ModuleType(name);mod.__dict__.update(attrs);monkeypatch.setitem(sys.modules,name,mod)
    spec=importlib.util.spec_from_file_location('tested_nav',Path(__file__).resolve().parents[1]/'atsd_drive/nav_node.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.NavSatStatus.STATUS_FIX = 0
    clock=[10.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    node=module.NavNode.__new__(module.NavNode)
    node.nav=RoadNav(Roads(allow_direct=True));node.nav.filter.set_manual(0.,0.,0.);node.nav.plan((5.,0.))
    node.local=LocalPlanner();node.avoidance=Decision();node.blocked_since=None;node.blocked_timeout=30.
    node.odom=(0.,0.,0.);node.odom_t=10.;node.v_odom=node.w_odom=0.
    node.space=None;node.scan_pose=None;node.scan_t=0.
    node.laser=(.29,0.,0.);node.laser_frame='laser_frame'
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock[0]*1e9)))
    node.leg=('TO_DROP','goal');node.leg_t=0.;node.result=None
    node.gps_gate=GpsGate();node.gps_stamp=None;node.origin=(55.,37.)
    node.wifi_fresh=lambda:None
    node.fix_t=node.lane_t=0.;node.gnss_timeout=2.;node.lane_timeout=.5;node.lane=None
    node.last_plan_pub=10.;node.no_pos_timeout=20.
    node.publish_tf=Mock();node.publish_command=Mock();node.pub_remain=Mock();node.publish_plan=Mock()
    node.publish_result=Mock();node.event=Mock();node.get_logger=Mock(return_value=Mock())
    return module,node,clock


def scan(stamp=9.9,frame='laser_frame'):
    sec=int(stamp)
    return NS(header=NS(stamp=NS(sec=sec,nanosec=int((stamp-sec)*1e9)),frame_id=frame),
              ranges=np.full(455,np.inf),angle_min=-np.pi,angle_increment=2*np.pi/455,range_min=.05,range_max=12.)


def test_scan_rejects_delayed_wrong_frame_and_missing_odom(adapter):
    _,node,_=adapter
    node.on_scan(scan(9.));assert node.space is None
    node.on_scan(scan(frame='wrong'));assert node.space is None
    node.odom=None;node.on_scan(scan());assert node.space is None
    node.odom=(0.,0.,0.);node.on_scan(scan())
    assert isinstance(node.space,ScanSpace) and node.scan_pose==(0.,0.,0.)
    assert abs(node.scan_t-9.9)<1e-6


def test_tick_requires_lidar_and_stops_after_scan_loss(adapter):
    _,node,clock=adapter
    node.tick();node.publish_command.assert_called_with(0.,0.)
    node.on_scan(scan());node.tick()
    assert node.publish_command.call_args.args[0]>0
    clock[0]=10.6;node.odom_t=10.6;node.tick()
    node.publish_command.assert_called_with(0.,0.)
    assert node.avoidance.state=='stale_scan'


def test_expired_during_search_is_not_published_as_fresh_motion(adapter):
    _,node,clock=adapter
    node.on_scan(scan())
    def slow_search(*a,**kw):
        clock[0]=10.5
        return Decision(.25,0.,'avoid')
    node.local.decide=slow_search
    node.tick()
    node.publish_command.assert_called_with(0.,0.)
    assert node.avoidance.state=='stale_after_planning'


def test_blocked_timeout_fails_and_mission_pause_stops(adapter):
    module,node,clock=adapter
    node.tick();clock[0]=41.;node.tick()
    assert node.result==(('TO_DROP','goal'),False) and node.leg is None
    node.publish_command.assert_called_with()
    node.leg=('TO_DROP','goal');node.result=None
    node.local.reverse_distance=2.
    node.on_state(module.String(data='{"state":"TO_DROP","to":"goal","pause":["red_light"]}'))
    assert node.leg is None and not node.nav.active and node.local.reverse_distance==0.


def test_graph_replacement_invalidates_active_path(adapter):
    _,node,_=adapter
    node.local.reverse_distance=2.
    node.invalidate_road_plan()
    assert not node.nav.active
    assert node.local.reverse_distance==0.
    node.publish_command.assert_called_with()
    node.publish_plan.assert_called_once()


def test_delivery_point_coordinates_take_priority_over_same_named_road_node(adapter,tmp_path):
    import json
    _,node,_=adapter
    node.points_file=str(tmp_path/'points.json')
    (tmp_path/'points.json').write_text(json.dumps([{'id':'kpp','x':10,'y':2}]))
    node.nav.roads=Roads({'kpp':[0,0],'home':[5,0]},[['kpp','home']])
    assert node.goal_of('kpp')==(10.,2.)
    assert node.goal_of('home')==(5.,0.)


def test_production_graph_without_roads_fails_instead_of_driving_direct(adapter):
    _,node,_=adapter
    node.nav.roads=Roads()
    node.nav.stop()
    node.load_roads=Mock()
    node.goal_of=Mock(return_value=(5.,0.))
    node.try_plan()
    assert node.result==(('TO_DROP','goal'),False)
    assert not node.nav.active
    node.publish_command.assert_called_with()


def test_recovery_off_road_still_proposes_route_command(adapter):
    _, node, _ = adapter
    node.on_scan(scan())
    node.local.mode = 'avoid'
    node.nav.filter.X[1] = 1.
    node.local.decide = Mock(return_value=Decision(.25,0.,'following'))
    node.tick()
    assert node.local.decide.call_args.kwargs['preferred'] is not None


def test_single_false_lane_cannot_steer_straight_route(adapter):
    _, node, clock = adapter
    node.on_scan(scan())
    node.lane_t = clock[0]
    node.lane = {'lines':1, 'offset':-1.7, 'angle':.1}
    node.local.decide = Mock(return_value=Decision(.25,0.,'following'))
    node.tick()
    assert node.local.decide.call_args.kwargs['preferred'][1] == 0.


def test_reverse_preference_uses_final_distance_not_lookahead(adapter):
    _, node, _ = adapter
    node.nav.plan((20.,0.))
    node.on_scan(scan())
    node.local.decide = Mock(return_value=Decision(.25,0.,'following'))
    node.tick()
    assert not node.local.decide.call_args.kwargs['short_reverse_allowed']


def gps_fix(node, clock, x=10., quality=0, sigma=1.):
    from atsd_drive.road_logic import to_geo
    lat, lon = to_geo(x,0.,node.origin)
    stamp=clock[0]; sec=int(stamp)
    cov=[0.]*9;cov[0]=cov[4]=sigma*sigma
    return NS(header=NS(stamp=NS(sec=sec,nanosec=int((stamp-sec)*1e9))),
              status=NS(status=quality),latitude=lat,longitude=lon,
              position_covariance=cov,position_covariance_type=1)


def test_gps_automatically_takes_over_manual_only_after_qualification(adapter):
    _,node,clock=adapter
    for t in range(10,15):
        clock[0]=float(t);node.on_fix(gps_fix(node,clock))
        assert node.nav.filter.manual
        assert node.nav.filter.X[0] == 0.
    clock[0]=15.;node.on_fix(gps_fix(node,clock))
    assert not node.nav.filter.manual and node.nav.filter.gnss=='ok'
    assert 0 < node.nav.filter.X[0] <= .25
    assert node.gps_gate.reason=='ok'
    clock[0]=16.;node.on_fix(gps_fix(node,clock,quality=-1))
    assert node.nav.filter.gnss=='none'


def test_gps_stale_unknown_accuracy_and_duplicate_cannot_take_over(adapter):
    _,node,clock=adapter
    msg=gps_fix(node,clock)
    node.on_fix(msg)
    for t in range(11,17):
        clock[0]=float(t);node.on_fix(msg)
    assert node.nav.filter.manual
    msg=gps_fix(node,clock);msg.position_covariance_type=0
    node.on_fix(msg)
    assert node.gps_gate.reason=='poor_quality' and node.nav.filter.manual


def test_gps_course_is_fresh_and_reversed_when_backing(adapter):
    _,node,clock=adapter
    node.nav.course=Mock();node.imu_fresh=lambda:False
    msg=NS(header=NS(stamp=NS(sec=10,nanosec=0)),
           twist=NS(twist=NS(linear=NS(x=.5,y=0.))))
    node.on_vel(msg);node.nav.course.assert_not_called()
    node.gps_gate.reason='ok';node.fix_t=10.;node.v_odom=-.5
    node.on_vel(msg)
    assert abs(node.nav.course.call_args.args[0]) == np.pi


def test_stationary_gps_drift_cannot_change_heading(adapter):
    _,node,_=adapter
    node.nav.course=Mock();node.imu_fresh=lambda:False
    node.gps_gate.reason='ok';node.fix_t=10.;node.v_odom=0.
    msg=NS(header=NS(stamp=NS(sec=10,nanosec=0)),
           twist=NS(twist=NS(linear=NS(x=.6,y=0.))))
    node.on_vel(msg)
    node.nav.course.assert_not_called()


def test_explicit_lane_loss_clears_previous_observation(adapter):
    _,node,_=adapter
    node.on_lane(NS(data='0.1 0.05 3.5 2'))
    assert node.lane is not None
    node.on_lane(NS(data='none'))
    assert node.lane is None and node.lane_t == 0.
