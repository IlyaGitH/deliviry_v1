#!/usr/bin/env python3
import copy
import json
import math
import threading
import time
from collections import deque

DEFAULT = {
    'connected': False,
    'brain_link': False,
    'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    'speed': 0.0,
    'indoor': True,
    'battery': {
        'drive': {'volts': None, 'percent': None},
        'compute': {'volts': None, 'percent': None},
    },
    'lock': {'closed': True, 'busy': False, 'tamper': False},
    'gnss': {'fix': False, 'lat': None, 'lon': None},
    'estop': False,
    'mission': None,
    'obstacle': '',
    'ble': {'zone': None, 'rssi': None, 'near': False, 'zones': {}},
}


class RosLink:

    def __init__(self, mock=False, logger=None):
        self.mock = mock
        self.log = logger
        self._lock = threading.Lock()
        self._state = copy.deepcopy(DEFAULT)
        self._events = deque(maxlen=60)
        self._event_seq = 0
        self._thread = None
        self._stop = threading.Event()
        self._node = None
        self._mock_cmd = deque()

    def snapshot(self):
        with self._lock:
            snap = copy.deepcopy(self._state)
            snap['events'] = list(self._events)[-25:]
        return snap

    def _set(self, path, value):
        with self._lock:
            node = self._state
            for key in path[:-1]:
                node = node[key]
            node[path[-1]] = value

    def _event(self, text):
        with self._lock:
            self._event_seq += 1
            self._events.append({'seq': self._event_seq, 'ts': time.time(), 'text': text})

    def start(self):
        target = self._run_mock if self.mock else self._run_ros
        self._thread = threading.Thread(target=target, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run_ros(self):
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
        from std_msgs.msg import Bool, Empty, String
        from std_srvs.srv import Trigger

        if not rclpy.ok():
            rclpy.init()

        node = Node('web_bridge')
        self._node = node
        self._trigger_type = Trigger
        self._empty = Empty
        self._string = String

        def on_odom(msg: Odometry):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                self._state['pose'] = {'x': msg.pose.pose.position.x,
                                       'y': msg.pose.pose.position.y, 'yaw': yaw}
                self._state['speed'] = abs(msg.twist.twist.linear.x)
                self._state['connected'] = True

        def batt(msg, lo=None, hi=None):
            pct = msg.percentage * 100.0 if msg.percentage == msg.percentage else None
            volts = msg.voltage if msg.voltage == msg.voltage else None
            if pct is None and volts and lo is not None:
                pct = max(0.0, min(100.0, (volts - lo) / (hi - lo) * 100.0))
            return {'volts': round(volts, 2) if volts else None,
                    'percent': round(pct) if pct is not None else None}

        def on_fix(msg: NavSatFix):
            has_fix = msg.status.status >= NavSatStatus.STATUS_FIX
            with self._lock:
                self._state['gnss'] = {'fix': has_fix,
                                       'lat': msg.latitude if has_fix else None,
                                       'lon': msg.longitude if has_fix else None}
                self._state['indoor'] = not has_fix

        def on_mission(msg: String):
            try:
                data = json.loads(msg.data)
            except ValueError:
                return
            self._set(['mission'], data)

        sub = node.create_subscription
        sub(Odometry, '/drive/odom', on_odom, 10)
        sub(BatteryState, '/drive/battery',
            lambda m: self._set(['battery', 'drive'], batt(m, 11.2, 14.6)), 5)
        sub(BatteryState, '/power/computer', lambda m: self._set(['battery', 'compute'], batt(m)), 5)
        sub(NavSatFix, '/gnss/fix', on_fix, 5)
        sub(Bool, '/lock/closed', lambda m: self._set(['lock', 'closed'], bool(m.data)), 5)
        sub(Bool, '/lock/busy', lambda m: self._set(['lock', 'busy'], bool(m.data)), 5)
        sub(Bool, '/lock/tamper', lambda m: self._set(['lock', 'tamper'], bool(m.data)), 5)
        sub(Bool, '/drive/estop', lambda m: self._set(['estop'], bool(m.data)), 5)
        sub(Bool, '/drive/link', lambda m: self._set(['brain_link'], bool(m.data)), 5)
        sub(String, '/mission/state', on_mission, 10)
        sub(String, '/mission/event', lambda m: self._event(m.data), 50)
        sub(String, '/perception/obstacle_info', lambda m: self._set(['obstacle'], m.data), 5)

        def on_zone(msg: String):
            parts = msg.data.split()
            if len(parts) >= 3:
                val = {'zone': parts[0], 'rssi': int(float(parts[1])), 'near': parts[2] == '1'}
            else:
                val = {'zone': None, 'rssi': None, 'near': False}
            with self._lock:
                self._state['ble'].update(val)

        def on_zones(msg: String):
            try:
                self._set(['ble', 'zones'], json.loads(msg.data))
            except ValueError:
                pass

        sub(String, '/ble/zone', on_zone, 5)
        sub(String, '/ble/zones', on_zones, 5)

        self._pub_start = node.create_publisher(String, '/mission/start', 5)
        self._pub_cargo = node.create_publisher(Empty, '/mission/cargo', 5)
        self._pub_abort = node.create_publisher(Empty, '/mission/abort', 5)
        self._pub_resume = node.create_publisher(Empty, '/mission/resume', 5)
        self._unlock_cli = node.create_client(Trigger, '/lock/unlock')

        if self.log:
            self.log('Мост ROS поднят')
        try:
            while rclpy.ok() and not self._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()

    def _run_mock(self):
        if self.log:
            self.log('Режим имитации: сценарий прошивки без робота')
        x, y, yaw = 0.0, 0.0, 0.0
        st = {'state': 'IDLE', 'from': None, 'to': None, 'pause': [], 'flags': [],
              'remain_m': 0.0, 'mission_s': 0.0, 'link': True, 'wp': 0, 'wp_count': 0}
        target, t0, drive, comp = None, None, 92.0, 88.0
        pts = {}
        with self._lock:
            self._state['connected'] = True
            self._state['brain_link'] = True

        while not self._stop.is_set():
            while self._mock_cmd:
                cmd, arg = self._mock_cmd.popleft()
                if cmd == 'start' and st['state'] in ('IDLE', 'DONE', 'FAULT'):
                    pts = arg['points']
                    st.update(state='TO_PICKUP', **{'from': arg['from'], 'to': arg['to']})
                    target, t0 = pts[arg['from']], time.time()
                    self._event(f'MISSION_START {arg["from"]} {arg["to"]}')
                    self._event('STATE TO_PICKUP')
                elif cmd == 'cargo' and st['state'] == 'WAIT_LOAD':
                    self._event('CARGO_LOADED')
                    st['state'], target = 'TO_DROP', pts[st['to']]
                    self._event('STATE TO_DROP')
                elif cmd == 'cargo' and st['state'] == 'WAIT_UNLOAD':
                    self._event('CARGO_UNLOADED')
                    self._event(f'DELIVERED {time.time() - t0:.1f} s')
                    st['state'], target = 'DONE', None
                    self._event('STATE DONE')
                elif cmd == 'abort':
                    st['state'], target = 'IDLE', None
                    self._event('MISSION_ABORT')

            if target is not None and st['state'] in ('TO_PICKUP', 'TO_DROP'):
                dx, dy = target[0] - x, target[1] - y
                dist = math.hypot(dx, dy)
                step = 0.55 * 0.2
                st['remain_m'] = round(dist, 1)
                if dist < 2.5 and not st.get('dock'):
                    st['dock'] = True
                    self._event('DOCK_SEARCH')
                    self._event('DOCK_SPOT 2400 30')
                if dist < step:
                    x, y = target
                    arrived = st['state'] == 'TO_PICKUP'
                    goal = st['from'] if arrived else st['to']
                    st['dock'] = False
                    self._event(f'DOCKED {goal} 120')
                    self._event(f'ZONE_OK {goal} -58')
                    self._event('ARRIVED_PICKUP' if arrived else 'ARRIVED_DROP')
                    st['state'] = 'WAIT_LOAD' if arrived else 'WAIT_UNLOAD'
                    self._event(f'STATE {st["state"]}')
                else:
                    x, y, yaw = x + dx / dist * step, y + dy / dist * step, math.atan2(dy, dx)
            if t0 and st['state'] not in ('DONE', 'IDLE'):
                st['mission_s'] = round(time.time() - t0, 1)

            drive = max(5.0, drive - 0.01)
            comp = max(5.0, comp - 0.002)
            with self._lock:
                self._state['pose'] = {'x': x, 'y': y, 'yaw': yaw}
                self._state['speed'] = 0.55 if st['state'] in ('TO_PICKUP', 'TO_DROP') else 0.0
                self._state['mission'] = dict(st)
                near_pt = min(pts.items(), key=lambda kv: math.hypot(kv[1][0] - x, kv[1][1] - y)) if pts else None
                if near_pt:
                    d = math.hypot(near_pt[1][0] - x, near_pt[1][1] - y)
                    self._state['ble'] = {'zone': near_pt[0], 'rssi': int(-45 - 25 * math.log10(max(d, 0.3))),
                                          'near': d < 3.0, 'zones': {}}
                self._state['battery']['drive'] = {'volts': round(11.2 + drive / 100 * 3.4, 2),
                                                   'percent': round(drive)}
                self._state['battery']['compute'] = {'volts': round(3.2 + comp / 100, 2),
                                                     'percent': round(comp)}
            time.sleep(0.2)

    def start_mission(self, from_id, to_id, points=None):
        if self.mock:
            self._mock_cmd.append(('start', {'from': from_id, 'to': to_id, 'points': points or {}}))
            return True
        if self._node is None:
            return False
        self._pub_start.publish(self._string(data=f'{from_id} {to_id}'))
        return True

    def cargo(self):
        if self.mock:
            self._mock_cmd.append(('cargo', None))
            return True
        if self._node is None:
            return False
        self._pub_cargo.publish(self._empty())
        return True

    def abort(self):
        if self.mock:
            self._mock_cmd.append(('abort', None))
            return True
        if self._node is None:
            return False
        self._pub_abort.publish(self._empty())
        return True

    def resume(self):
        if self.mock or self._node is None:
            return False
        self._pub_resume.publish(self._empty())
        return True

    def unlock(self, timeout=3.0):
        if self.mock:
            self._set(['lock', 'busy'], True)
            threading.Timer(0.6, lambda: self._set(['lock', 'busy'], False)).start()
            return True, 'Замок открыт (имитация)'
        if self._node is None:
            return False, 'Мост ROS не поднят'
        if not self._unlock_cli.wait_for_service(timeout_sec=1.0):
            return False, 'Сервис /lock/unlock недоступен'
        future = self._unlock_cli.call_async(self._trigger_type.Request())
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, 'Замок не ответил вовремя'
        res = future.result()
        return bool(res.success), res.message
