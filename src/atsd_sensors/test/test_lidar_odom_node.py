import importlib.util
import json
from pathlib import Path
import sys
import types
from unittest.mock import Mock
import numpy as np
import pytest
from atsd_sensors.lidar_odometry import ScanOdometry, PoseHistory, SlipMonitor

NS=types.SimpleNamespace


@pytest.fixture
def observer(monkeypatch):
    class Message:
        def __init__(self, **kw):
            self.__dict__.update(kw)
            self.pose=NS(pose=NS(position=NS(x=0.,y=0.),orientation=NS(z=0.,w=1.)),covariance=[0.]*36)
    modules={'rclpy':{},'rclpy.node':{'Node':object},'rclpy.qos':{'qos_profile_sensor_data':object()},
             'sensor_msgs.msg':{'LaserScan':Message,'Imu':Message},'nav_msgs.msg':{'Odometry':Message},
             'geometry_msgs.msg':{'PoseWithCovarianceStamped':Message},'std_msgs.msg':{'String':Message}}
    for name,attrs in modules.items():
        m=types.ModuleType(name);m.__dict__.update(attrs);monkeypatch.setitem(sys.modules,name,m)
    path=Path(__file__).resolve().parents[1]/'atsd_sensors/lidar_odom_node.py'
    spec=importlib.util.spec_from_file_location('tested_observer',path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    node=mod.LidarOdomNode.__new__(mod.LidarOdomNode)
    node.engine=ScanOdometry(2.);node.slip=SlipMonitor();node.wheels=PoseHistory();node.imu=PoseHistory()
    node.previous_stamp=node.previous_wheel=node.previous_yaw=None
    node.laser=(.29,0.,0.);node.frame='laser_frame';node.timeout=.45
    node.delta_pub=NS(publish=Mock());node.status_pub=NS(publish=Mock())
    clock=[10.]
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock[0]*1e9)))
    x=np.linspace(-2,3,100);y=np.linspace(-2,2,100)
    points=np.concatenate([np.column_stack((x,np.full_like(x,2.))),np.column_stack((np.full_like(y,3.),y)),
                           np.column_stack((x,np.full_like(x,-2.)))])
    monkeypatch.setattr(mod,'cloud',lambda *args:points)
    return mod,node,clock


def scan(stamp):
    sec=int(stamp)
    return NS(header=NS(stamp=NS(sec=sec,nanosec=round((stamp-sec)*1e9)),frame_id='laser_frame'),
              ranges=[],angle_min=-np.pi,angle_increment=.01,range_min=.05,range_max=12.)


def test_observer_flags_wheel_spin_only_after_good_scan_matches(observer):
    _,node,clock=observer
    for i in range(10):
        t=10+i*.2;clock[0]=t+.1
        node.wheels.add(t-.05,(i*.04-.01,0,0));node.wheels.add(t+.05,(i*.04+.01,0,0))
        node.on_scan(scan(t))
    status=json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['valid'] and status['slip_suspected']
    assert status['mode']=='observe' and np.linalg.norm(status['delta'])<.01
    assert node.delta_pub.publish.call_count>0
    count=node.delta_pub.publish.call_count
    clock[0]=30.;node.on_scan(scan(10.))
    assert node.delta_pub.publish.call_count==count
    status=json.loads(node.status_pub.publish.call_args.args[0].data)
    assert not status['valid'] and not status['slip_suspected']


def test_no_wheel_data_is_not_misreported_as_no_slip_evidence(observer):
    _,node,clock=observer
    for t in (10.,10.2):
        clock[0]=t+.1;node.on_scan(scan(t))
    status=json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['valid'] and not status['wheel_comparison']
