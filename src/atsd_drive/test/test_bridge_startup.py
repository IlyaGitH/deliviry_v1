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
        'geometry_msgs.msg': {k: Message for k in ('Twist', 'TransformStamped', 'Quaternion')},
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
