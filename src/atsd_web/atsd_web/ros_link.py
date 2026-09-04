#!/usr/bin/env python3
"""
atsd_web · ros_link

Мост между ROS 2 и веб-сервером. Живёт в отдельном потоке,
складывает телеметрию в потокобезопасный снимок, который
сервер отдаёт по WebSocket.

Подписки:
    /drive/odom        положение и скорость
    /drive/battery     аккумулятор привода, V5
    /power/computer    аккумулятор вычислителя, X1202
    /gnss/fix          координаты и наличие фикса
    /lock/closed       крышка закрыта
    /lock/busy         соленоид под током

Публикации и вызовы:
    /cmd_goal          std_msgs/String — целевая точка для верхнего уровня
    /lock/unlock       std_srvs/Trigger — открыть отсек

РЕЖИМ ИМИТАЦИИ
Флаг mock=true подменяет ROS генератором: ровер ездит по точкам,
батареи садятся, крышка открывается. Позволяет разрабатывать
и показывать интерфейс до того, как поедет робот.
"""

import math
import threading
import time

DEFAULT = {
    'connected': False,
    'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    'speed': 0.0,
    'indoor': True,
    'battery': {
        'drive': {'volts': None, 'percent': None},
        'compute': {'volts': None, 'percent': None},
    },
    'lock': {'closed': True, 'busy': False},
    'gnss': {'fix': False, 'lat': None, 'lon': None},
    'estop': False,
}


class RosLink:
    """Общий интерфейс: снимок телеметрии и команды."""

    def __init__(self, mock=False, logger=None):
        self.mock = mock
        self.log = logger
        self._lock = threading.Lock()
        self._state = dict(DEFAULT)
        self._thread = None
        self._stop = threading.Event()
        self._node = None

    # ────────────────────────────────── снимок
    def snapshot(self):
        with self._lock:
            return {
                'connected': self._state['connected'],
                'pose': dict(self._state['pose']),
                'speed': self._state['speed'],
                'indoor': self._state['indoor'],
                'battery': {
                    'drive': dict(self._state['battery']['drive']),
                    'compute': dict(self._state['battery']['compute']),
                },
                'lock': dict(self._state['lock']),
                'gnss': dict(self._state['gnss']),
                'estop': self._state['estop'],
            }

    def _set(self, path, value):
        with self._lock:
            node = self._state
            for key in path[:-1]:
                node = node[key]
            node[path[-1]] = value

    # ────────────────────────────────── запуск
    def start(self):
        target = self._run_mock if self.mock else self._run_ros
        self._thread = threading.Thread(target=target, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    # ────────────────────────────────── ROS
    def _run_ros(self):
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger

        if not rclpy.ok():
            rclpy.init()

        node = Node('web_bridge')
        self._node = node
        self._trigger_type = Trigger

        def on_odom(msg: Odometry):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                self._state['pose'] = {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'yaw': yaw,
                }
                self._state['speed'] = abs(msg.twist.twist.linear.x)
                self._state['connected'] = True

        def on_drive_batt(msg: BatteryState):
            pct = msg.percentage * 100.0 if msg.percentage == msg.percentage else None
            # У батареи V5 гейджа нет, оцениваем по напряжению:
            # 12,8 В номинал LiFePO4, полный заряд около 14,6, пустой 11,2.
            if pct is None and msg.voltage == msg.voltage and msg.voltage > 1.0:
                pct = max(0.0, min(100.0, (msg.voltage - 11.2) / (14.6 - 11.2) * 100.0))
            self._set(['battery', 'drive'],
                      {'volts': round(msg.voltage, 2) if msg.voltage == msg.voltage else None,
                       'percent': round(pct, 0) if pct is not None else None})

        def on_comp_batt(msg: BatteryState):
            pct = msg.percentage * 100.0 if msg.percentage == msg.percentage else None
            self._set(['battery', 'compute'],
                      {'volts': round(msg.voltage, 2) if msg.voltage == msg.voltage else None,
                       'percent': round(pct, 0) if pct is not None else None})

        def on_fix(msg: NavSatFix):
            has_fix = msg.status.status >= NavSatStatus.STATUS_FIX
            with self._lock:
                self._state['gnss'] = {
                    'fix': has_fix,
                    'lat': msg.latitude if has_fix else None,
                    'lon': msg.longitude if has_fix else None,
                }
                # Нет спутников — значит под крышей. Простое правило,
                # которое на практике работает лучше геозон.
                self._state['indoor'] = not has_fix

        node.create_subscription(Odometry, '/drive/odom', on_odom, 10)
        node.create_subscription(BatteryState, '/drive/battery', on_drive_batt, 5)
        node.create_subscription(BatteryState, '/power/computer', on_comp_batt, 5)
        node.create_subscription(NavSatFix, '/gnss/fix', on_fix, 5)
        node.create_subscription(Bool, '/lock/closed',
                                 lambda m: self._set(['lock', 'closed'], bool(m.data)), 5)
        node.create_subscription(Bool, '/lock/busy',
                                 lambda m: self._set(['lock', 'busy'], bool(m.data)), 5)
        node.create_subscription(Bool, '/drive/estop',
                                 lambda m: self._set(['estop'], bool(m.data)), 5)

        self._goal_pub = node.create_publisher(String, '/cmd_goal', 5)
        self._unlock_cli = node.create_client(Trigger, '/lock/unlock')

        if self.log:
            self.log('Мост ROS поднят')

        try:
            while rclpy.ok() and not self._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()

    # ────────────────────────────────── имитация
    def _run_mock(self):
        if self.log:
            self.log('Режим имитации: телеметрия синтетическая')

        route = [(0.0, 0.0), (12.0, 0.0), (12.0, 8.0), (0.0, 8.0)]
        idx, t0 = 0, time.time()
        x, y = route[0]
        drive, comp = 92.0, 88.0

        with self._lock:
            self._state['connected'] = True

        while not self._stop.is_set():
            tx, ty = route[(idx + 1) % len(route)]
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            step = 0.5 * 0.2                       # 0,5 м/с при шаге 200 мс
            if dist < step:
                x, y = tx, ty
                idx = (idx + 1) % len(route)
            else:
                x += dx / dist * step
                y += dy / dist * step

            elapsed = time.time() - t0
            drive = max(5.0, 92.0 - elapsed * 0.05)
            comp = max(5.0, 88.0 - elapsed * 0.01)

            with self._lock:
                self._state['pose'] = {'x': x, 'y': y, 'yaw': math.atan2(dy, dx)}
                self._state['speed'] = 0.5
                self._state['indoor'] = True
                self._state['battery']['drive'] = {
                    'volts': round(11.2 + drive / 100.0 * 3.4, 2),
                    'percent': round(drive)}
                self._state['battery']['compute'] = {
                    'volts': round(3.2 + comp / 100.0 * 1.0, 2),
                    'percent': round(comp)}
            time.sleep(0.2)

    # ────────────────────────────────── команды
    def unlock(self, timeout=3.0):
        """Открыть грузовой отсек. Возвращает (успех, сообщение)."""
        if self.mock:
            self._set(['lock', 'busy'], True)
            threading.Timer(0.6, lambda: self._set(['lock', 'busy'], False)).start()
            self._set(['lock', 'closed'], False)
            return True, 'Замок открыт (имитация)'

        if self._node is None or not hasattr(self, '_unlock_cli'):
            return False, 'Мост ROS не поднят'
        if not self._unlock_cli.wait_for_service(timeout_sec=1.0):
            return False, 'Сервис /lock/unlock недоступен'

        req = self._trigger_type.Request()
        future = self._unlock_cli.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, 'Замок не ответил вовремя'
        res = future.result()
        return bool(res.success), res.message

    def send_goal(self, point_name: str):
        """Передать целевую точку верхнему уровню."""
        if self.mock or self._node is None:
            return True
        from std_msgs.msg import String
        self._goal_pub.publish(String(data=point_name))
        return True
