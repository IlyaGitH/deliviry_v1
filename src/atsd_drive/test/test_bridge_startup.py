"""Exercise real bridge reconnect methods with ROS publishers and USB transport replaced."""
import importlib.util
from pathlib import Path
import sys
import threading
import types
from unittest.mock import Mock

import pytest


@pytest.fixture
def bridge_module(monkeypatch):
    class Message:
        def __init__(self, **kw):
            self.__dict__.update(kw)

    exports = {
        'rclpy': {'ok': lambda: True},
        'rclpy.node': {'Node': object},
        'rclpy.qos': {k: Message for k in ('QoSProfile', 'ReliabilityPolicy', 'HistoryPolicy', 'DurabilityPolicy')},
        'geometry_msgs.msg': {k: Message for k in ('Twist', 'TwistStamped', 'TransformStamped', 'Quaternion')},
        'nav_msgs.msg': {'Odometry': Message},
        'sensor_msgs.msg': {'BatteryState': Message},
        'std_msgs.msg': {k: Message for k in ('Bool', 'Empty', 'Float32', 'String')},
        'tf2_ros': {'TransformBroadcaster': Message},
        'serial': {'SerialException': OSError, 'Serial': Mock()},
    }
    for name, attrs in exports.items():
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / 'atsd_drive/v5_bridge_node.py'
    spec = importlib.util.spec_from_file_location('tested_bridge', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_bridge(module):
    node = module.V5BridgeNode.__new__(module.V5BridgeNode)
    node.ser = None
    node.ser_lock = threading.Lock()
    node.port, node.baud = '/dev/v5', 115200
    node.last_rx = node.last_query = node.port_opened = 0.
    node.link_timeout, node.serial_silence, node.reconnect_s = .5, 5., 2.
    node.firmware, node.last_status, node.link_prev = '', None, None
    node.get_logger = Mock(return_value=Mock())
    node.pub_link, node.pub_state = Mock(), Mock()
    return node


def test_pi_starts_before_vex_then_recovers_silent_connection(bridge_module, monkeypatch):
    m = bridge_module
    node = make_bridge(m)
    clock = [10.]
    monkeypatch.setattr(m.time, 'monotonic', lambda: clock[0])
    first, second = Mock(), Mock()
    m.serial.Serial.side_effect = [OSError('device absent'), first, second]
    assert not node.open_port()  # Pi first: keep retrying
    assert node.open_port()      # USB enumerates but firmware is still silent
    node.check_link()
    assert node.pub_link.publish.call_args.args[0].data is False
    assert first.write.call_count >= 2  # initial query + retry without any received data
    clock[0] = 16.
    node.check_link()
    assert node.ser is None
    first.close.assert_called_once()
    assert node.open_port()
    node.last_rx = 16.1  # valid firmware response
    clock[0] = 16.2
    node.check_link()
    assert node.pub_link.publish.call_args.args[0].data is True
    second.close.assert_not_called()


def test_reconnect_discards_old_mission_status(bridge_module):
    node = make_bridge(bridge_module)
    node.ser = Mock()
    node.last_status = Mock(state='TO_DROP')
    node.last_rx = 10
    node.close_port()
    assert node.last_status is None and node.last_rx == 0


def test_bridge_rejects_delayed_checked_commands(bridge_module):
    from types import SimpleNamespace as NS
    node = bridge_module.V5BridgeNode.__new__(bridge_module.V5BridgeNode)
    node.traveling = lambda: True
    node.send = Mock()
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
    msg = NS(header=NS(frame_id='base_link', stamp=NS(sec=9, nanosec=0)),
             twist=NS(linear=NS(x=-.15), angular=NS(z=.15)))
    node.on_nav_cmd(msg)
    node.send.assert_called_with('G 0 0\n')
    msg.header.stamp = NS(sec=9, nanosec=900_000_000)
    node.on_nav_cmd(msg)
    node.send.assert_called_with('G -150 150\n')
    node.traveling = lambda: False
    node.send.reset_mock()
    node.on_nav_cmd(msg)
    node.send.assert_not_called()


def test_ros_logger_severity_is_constant_per_callsite(bridge_module, monkeypatch):
    import inspect
    node=make_bridge(bridge_module)
    seen={}
    class Logger:
        def record(self,level):
            frame=inspect.currentframe().f_back.f_back
            key=(frame.f_code,frame.f_lineno)
            assert seen.setdefault(key,level)==level
        def info(self,*args): self.record('info')
        def warn(self,*args): self.record('warn')
        def error(self,*args): self.record('error')
    node.get_logger=lambda:Logger()
    monkeypatch.setattr(bridge_module.time,'monotonic',lambda:10.)
    for last in (0.,9.9,0.,9.9):
        node.last_rx=last
        node.check_link()
    node.pub_event=Mock()
    for text in ('K LINK_OK','K LINK_LOST','K LINK_OK','K ESTOP'):
        node.handle_line(text)


def test_order_server_loss_stops_brain_heartbeat_and_recovers(bridge_module, monkeypatch):
    m = bridge_module
    node = make_bridge(m)
    clock = [100.]
    monkeypatch.setattr(m.time, 'monotonic', lambda: clock[0])
    node.send = Mock()
    node.pub_event = Mock()
    node.require_order_server = True
    node.server_last = 0.
    node.server_ok_prev = None
    node.require_feeds = ['obstacle']
    node.feed_timeout = 1.
    node.feed_seen = {'obstacle':clock[0]}
    node.feeds_ok_prev = None
    node.last_obst_text = ''
    node.send_heartbeat()
    node.send.assert_not_called()  # protected installation cannot drive before web is alive
    for value in ('nan', 'inf', '101', '95', 'bad'):
        node.on_server_alive(m.String(data=value))
    assert node.server_last == 0
    node.on_server_alive(m.String(data='100'))
    node.send_heartbeat()
    node.send.assert_called_once_with('H\n')
    node.send.reset_mock()
    clock[0] = 103.1
    node.feed_seen['obstacle'] = clock[0]
    node.send_heartbeat()
    node.send.assert_not_called()
    node.on_server_alive(m.String(data='100'))  # queued old heartbeat must not revive the server
    node.send_heartbeat()
    node.send.assert_not_called()
    node.on_server_alive(m.String(data='103.1'))
    node.send_heartbeat()
    node.send.assert_called_once_with('H\n')
    assert [c.args[0].data for c in node.pub_event.publish.call_args_list] == [
        'ORDER_SERVER_LOST', 'ORDER_SERVER_OK', 'ORDER_SERVER_LOST', 'ORDER_SERVER_OK']
