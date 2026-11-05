"""Exercise production ROS callbacks with only transport/message types stubbed."""
import importlib.util
from pathlib import Path
import sys
import types
from unittest.mock import Mock

import numpy as np
import pytest

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
    clock=[10.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    node=module.NavNode.__new__(module.NavNode)
    node.nav=RoadNav(Roads());node.nav.filter.set_manual(0.,0.,0.);node.nav.plan((5.,0.))
    node.local=LocalPlanner();node.avoidance=Decision();node.blocked_since=None;node.blocked_timeout=30.
    node.odom=(0.,0.,0.);node.odom_t=10.;node.v_odom=node.w_odom=0.
    node.space=None;node.scan_pose=None;node.scan_t=0.
    node.laser=(.29,0.,0.);node.laser_frame='laser_frame'
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock[0]*1e9)))
    node.leg=('TO_DROP','goal');node.leg_t=0.;node.result=None
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
