#!/usr/bin/env python3
import copy
import json
import math
import random
import threading
import time
from collections import deque

TRAVEL = ('TO_PICKUP', 'TO_DROP', 'RETURN')

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
    'plan': [],
    'road': {},
    'gnss_status': '',
    'lane': '',
    'ble': {'zone': None, 'rssi': None, 'near': False, 'zones': {}},
}


def _nmea(body):
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f'${body}*{cs:02X}'


def _mock_frame(tick, speed):
    """Кадр для режима имитации: дорога с двумя линиями, как рисует vision_node."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        return None
    img = np.full((240, 320, 3), 70, np.uint8)
    img[:110] = (150, 120, 90)
    sway = int(12 * math.sin(tick / 10))
    for x0, x1 in ((60, 140), (260, 180)):
        cv2.line(img, (x0 + sway, 239), (x1 + sway, 110), (230, 230, 230), 3)
        cv2.line(img, (x0 + sway, 239), (x1 + sway, 110), (0, 140, 255), 2)
    cv2.line(img, (160 + sway, 239), (160 + sway, 110), (0, 255, 0), 2)
    cv2.putText(img, f'MOCK lane 2 v {speed:.2f}', (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    ok, buf = cv2.imencode('.jpg', img)
    return buf.tobytes() if ok else None


def _mock_nmea(x, y, yaw, speed):
    """Имитация строк BN-880 для отладочной консоли: позиция вокруг Тольятти."""
    lat = 53.5078 + y / 111320.0
    lon = 49.4204 + x / (111320.0 * math.cos(math.radians(53.5078)))
    t = time.strftime('%H%M%S.00', time.gmtime())
    la = f'{int(lat):02d}{(lat % 1) * 60:07.4f}'
    lo = f'{int(lon):03d}{(lon % 1) * 60:07.4f}'
    course = (90 - math.degrees(yaw)) % 360
    return [_nmea(f'GNGGA,{t},{la},N,{lo},E,1,09,1.1,92.4,M,14.2,M,,'),
            _nmea(f'GNRMC,{t},A,{la},N,{lo},E,{speed / 0.514444:.2f},{course:.1f},'
                  f'{time.strftime("%d%m%y", time.gmtime())},,,A')]


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
        self._console = deque(maxlen=600)   # отладочная консоль: NMEA, события
        self._console_seq = 0
        self._jpeg, self._jpeg_seq, self._jpeg_t = None, 0, 0.0

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
        self._log(text, 'event')

    def _log(self, text, kind):
        with self._lock:
            self._console_seq += 1
            self._console.append({'seq': self._console_seq, 'ts': time.time(), 'kind': kind, 'text': text})

    def console_since(self, seq):
        with self._lock:
            return [c for c in self._console if c['seq'] > seq][-200:]

    def frame(self):
        """Последний кадр с отрисовкой распознанного: (номер, jpeg, возраст в с)."""
        with self._lock:
            return self._jpeg_seq, self._jpeg, time.time() - self._jpeg_t

    def _on_jpeg(self, data):
        with self._lock:
            self._jpeg, self._jpeg_seq, self._jpeg_t = bytes(data), self._jpeg_seq + 1, time.time()

    def start(self):
        target = self._run_mock if self.mock else self._run_ros
        self._thread = threading.Thread(target=target, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run_ros(self):
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry, Path
        from sensor_msgs.msg import BatteryState, CompressedImage, NavSatFix, NavSatStatus
        from std_msgs.msg import Bool, Empty, String
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer, TransformException, TransformListener

        if not rclpy.ok():
            rclpy.init()

        node = Node('web_bridge')
        self._node = node
        self._trigger_type = Trigger
        self._empty = Empty
        self._string = String

        tf_buf = Buffer()
        tf_listener = TransformListener(tf_buf, node)  # noqa: F841 — держит подписку на /tf

        def pose_from_tf():
            # Поза в карте лидара; одометрия Brain только для скорости
            try:
                t = tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            except TransformException:
                return
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._set(['pose'], {'x': t.transform.translation.x, 'y': t.transform.translation.y, 'yaw': yaw})

        def on_odom(msg: Odometry):
            with self._lock:
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
            if data.get('state') not in TRAVEL:
                self._set(['plan'], [])

        def on_plan(msg: Path):
            # Маршрут в кадре map; точки через ~10 см — для рисования хватает
            out = []
            for ps in msg.poses:
                p = (round(ps.pose.position.x, 2), round(ps.pose.position.y, 2))
                if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= 0.1:
                    out.append(p)
            if msg.poses:
                last = msg.poses[-1].pose.position
                out.append((round(last.x, 2), round(last.y, 2)))
            self._set(['plan'], out)

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
        sub(Path, '/plan', on_plan, 1)

        def on_road(msg: String):
            try:
                self._set(['road'], json.loads(msg.data))
            except ValueError:
                pass

        sub(String, '/road/status', on_road, 5)
        sub(CompressedImage, '/perception/debug/compressed', lambda m: self._on_jpeg(m.data), 2)
        sub(String, '/gnss/nmea', lambda m: self._log(m.data, 'nmea'), 50)
        sub(String, '/gnss/status', lambda m: self._set(['gnss_status'], m.data), 5)
        sub(String, '/perception/lane', lambda m: self._set(['lane'], m.data), 5)

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
        self._pose_type = PoseWithCovarianceStamped
        self._pub_pose = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 5)

        if self.log:
            self.log('Мост ROS поднят')
        try:
            while rclpy.ok() and not self._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
                pose_from_tf()
        finally:
            node.destroy_node()

    def _run_mock(self):
        if self.log:
            self.log('Режим имитации: сценарий прошивки без робота')
        x, y, yaw = 0.0, 0.0, 0.0
        st = {'state': 'IDLE', 'from': None, 'to': None, 'pause': [], 'flags': [],
              'remain_m': 0.0, 'mission_s': 0.0, 'link': True}
        target, t0, drive, comp = None, None, 92.0, 88.0
        pts = {}
        tick = 0
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
                if dist < step:
                    x, y = target
                    arrived = st['state'] == 'TO_PICKUP'
                    goal = st['from'] if arrived else st['to']
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
                self._state['plan'] = [(x, y), tuple(target)] if target is not None else []
                near_pt = min(pts.items(), key=lambda kv: math.hypot(kv[1][0] - x, kv[1][1] - y)) if pts else None
                if near_pt:
                    d = math.hypot(near_pt[1][0] - x, near_pt[1][1] - y)
                    self._state['ble'] = {'zone': near_pt[0], 'rssi': int(-45 - 25 * math.log10(max(d, 0.3))),
                                          'near': d < 3.0, 'zones': {}}
                self._state['battery']['drive'] = {'volts': round(11.2 + drive / 100 * 3.4, 2),
                                                   'percent': round(drive)}
                self._state['battery']['compute'] = {'volts': round(3.2 + comp / 100, 2),
                                                     'percent': round(comp)}
            tick += 1
            speed = 0.55 if st['state'] in ('TO_PICKUP', 'TO_DROP') else 0.0
            fx, fy = x + random.gauss(0, 0.8), y + random.gauss(0, 0.8)
            with self._lock:
                self._state['road'] = {'gnss': 'ok', 'mode': 'lane' if speed else 'idle', 'lane': 2,
                                       'yaw_known': True, 'ready': True, 'pose': [x, y, yaw],
                                       'fix': [round(fx, 2), round(fy, 2)], 'sigma': 2.5}
                self._state['gnss_status'] = f'sats=9/14 snr=38дБx14 hdop=1.1 qual=1 fix=3D v={speed:.1f}'
                self._state['lane'] = '0.050 -0.0100 3.50 2' if speed else 'none'
            if tick % 2 == 0:
                jpg = _mock_frame(tick, speed)
                if jpg:
                    self._on_jpeg(jpg)
            if tick % 5 == 0:
                for line in _mock_nmea(fx, fy, yaw, speed):
                    self._log(line, 'nmea')
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

    def set_pose(self, x, y, yaw):
        """Сбросить локализацию: робот стоит в (x, y) карты и смотрит под углом yaw."""
        if self.mock:
            self._set(['pose'], {'x': x, 'y': y, 'yaw': yaw})
            return True
        if self._node is None:
            return False
        msg = self._pose_type()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self._pub_pose.publish(msg)
        self._event(f'POSE_SET {x:.2f} {y:.2f} {math.degrees(yaw):.0f}')
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
