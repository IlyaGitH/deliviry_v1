#!/usr/bin/env python3
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import serial
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty, Float32, String
from tf2_ros import TransformBroadcaster

from atsd_drive import protocol as proto


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class V5BridgeNode(Node):

    def __init__(self):
        super().__init__('v5_bridge')

        d = self.declare_parameter
        d('port', '/dev/v5')
        d('baud', 115200)
        d('reconnect_s', 2.0)
        d('link_timeout_s', 0.5)
        d('cmd_rate_hz', 20.0)
        d('cmd_timeout_s', 0.5)
        d('heartbeat_hz', 10.0)
        d('obstacle_max_hz', 15.0)
        d('odom_source', 'brain')
        d('wheel_diameter_mm', 101.6)
        d('track_mm', 430.0)
        d('gear_ratio', 1.0)
        d('odom_frame', 'odom')
        d('base_frame', 'base_link')
        d('publish_tf', True)
        d('point_ids', ['kpp', 'admin', 'testing', 'depot'])
        d('use_pose_correction', False)
        d('require_feeds', ['obstacle'])
        d('feed_timeout_s', 1.0)

        p = lambda n: self.get_parameter(n).value
        self.port = p('port')
        self.baud = int(p('baud'))
        self.reconnect_s = float(p('reconnect_s'))
        self.link_timeout = float(p('link_timeout_s'))
        self.cmd_timeout = float(p('cmd_timeout_s'))
        self.obst_period = 1.0 / float(p('obstacle_max_hz'))
        self.odom_source = str(p('odom_source'))
        self.mm_per_deg = math.pi * float(p('wheel_diameter_mm')) / 360.0 / float(p('gear_ratio'))
        self.track_m = float(p('track_mm')) / 1000.0
        self.odom_frame = p('odom_frame')
        self.base_frame = p('base_frame')
        self.publish_tf = bool(p('publish_tf'))
        self.point_ids = [str(x) for x in p('point_ids')]
        self.require_feeds = [str(x) for x in p('require_feeds')]
        self.feed_timeout = float(p('feed_timeout_s'))
        self.feed_seen = {'obstacle': 0.0, 'vision': 0.0}
        self.feeds_ok_prev = None

        self.ser = None
        self.ser_lock = threading.Lock()
        self.last_rx = 0.0
        self.link_prev = None
        self.flags_prev = None
        self.firmware = ''
        self.last_status = None
        self.last_flags = 0

        self.cmd_lin = 0.0
        self.cmd_ang = 0.0
        self.last_cmd = 0.0
        self.zero_sent = 0

        self.last_obst_sent = 0.0
        self.last_spot_sent = 0.0
        self.last_zone_sent = 0.0
        self.last_nudge_sent = 0.0
        self.last_obst_text = ''

        self.ex = self.ey = self.eth = 0.0
        self.prev_deg = None
        self.prev_t = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_odom = self.create_publisher(Odometry, 'drive/odom', 20)
        self.pub_batt = self.create_publisher(BatteryState, 'drive/battery', 5)
        self.pub_estop = self.create_publisher(Bool, 'drive/estop', 5)
        self.pub_link = self.create_publisher(Bool, 'drive/link', 5)
        self.pub_flags = self.create_publisher(String, 'drive/flags', 5)
        self.pub_fw = self.create_publisher(String, 'drive/firmware', latched)
        self.pub_state = self.create_publisher(String, 'mission/state', 10)
        self.pub_event = self.create_publisher(String, 'mission/event', 50)
        self.tf_bc = TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(String, 'mission/start', self.on_mission_start, 10)
        self.create_subscription(Empty, 'mission/cargo', lambda _: self.send(proto.cmd_simple('C')), 10)
        self.create_subscription(Empty, 'mission/abort', lambda _: self.send(proto.cmd_simple('A')), 10)
        self.create_subscription(Empty, 'mission/resume', lambda _: self.send(proto.cmd_simple('U')), 10)
        self.create_subscription(Bool, 'drive/estop_cmd',
                                 lambda m: self.send(proto.cmd_simple('X' if m.data else 'R')), 10)
        self.create_subscription(Float32, 'drive/speed_limit',
                                 lambda m: self.send(proto.cmd_speed_limit(m.data)), 10)
        self.create_subscription(String, 'perception/sign', self.on_sign, 10)
        self.create_subscription(String, 'perception/light', self.on_light, 10)
        self.create_subscription(String, 'perception/obstacle', self.on_obstacle, 10)
        self.create_subscription(Twist, 'cmd_nudge', self.on_nudge, 10)
        self.create_subscription(String, 'perception/spot', self.on_spot, 10)
        self.create_subscription(String, 'ble/zone', self.on_zone, 10)
        if bool(p('use_pose_correction')):
            self.create_subscription(PoseWithCovarianceStamped, 'localization/pose_correction',
                                     self.on_pose_correction, 5)

        self.create_timer(1.0 / float(p('cmd_rate_hz')), self.send_cmd_vel)
        self.create_timer(1.0 / float(p('heartbeat_hz')), self.send_heartbeat)
        self.create_timer(0.2, self.check_link)

        self.running = True
        threading.Thread(target=self.rx_loop, daemon=True).start()
        self.get_logger().info(f'Мост V5: порт {self.port}, одометрия из «{self.odom_source}», '
                               f'точки {self.point_ids}')

    def open_port(self):
        try:
            s = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.1)
            s.reset_input_buffer()
            with self.ser_lock:
                self.ser = s
            self.get_logger().info(f'Порт {self.port} открыт')
            self.send(proto.cmd_simple('?'))
            return True
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'Не открыть {self.port}: {e}. Повтор через {self.reconnect_s} с',
                                   throttle_duration_sec=10.0)
            return False

    def close_port(self):
        with self.ser_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None

    def send(self, line: str):
        with self.ser_lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(line.encode('ascii'))
                return True
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Запись не удалась: {e}')
        self.close_port()
        return False

    def rx_loop(self):
        buf = b''
        while self.running and rclpy.ok():
            if self.ser is None:
                if not self.open_port():
                    time.sleep(self.reconnect_s)
                    continue
                buf = b''
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError, AttributeError) as e:
                self.get_logger().error(f'Порт отвалился: {e}')
                self.close_port()
                time.sleep(self.reconnect_s)
                continue
            if not chunk:
                continue
            buf += chunk
            if len(buf) > 8192:
                buf = buf[-1024:]
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', 'ignore').strip()
                if text:
                    try:
                        self.handle_line(text)
                    except Exception as e:
                        self.get_logger().warn(f'Ошибка разбора «{text}»: {e}',
                                               throttle_duration_sec=5.0)

    def handle_line(self, text: str):
        msg = proto.parse_line(text)
        if msg is None:
            self.get_logger().debug(f'V5: {text}')
            return
        self.last_rx = time.monotonic()

        if isinstance(msg, proto.Encoders):
            self.on_encoders(msg)
        elif isinstance(msg, proto.Pose):
            if self.odom_source == 'brain':
                self.publish_odom(msg.x_mm / 1000.0, msg.y_mm / 1000.0,
                                  math.radians(msg.yaw_mdeg / 1000.0),
                                  msg.v_mm_s / 1000.0, msg.w_mrad_s / 1000.0)
        elif isinstance(msg, proto.Status):
            self.last_status = msg
            self.publish_state(msg)
        elif isinstance(msg, proto.Event):
            self.pub_event.publish(String(data=msg.raw))
            level = self.get_logger().warn if msg.name in (
                'FAULT', 'STALL', 'BUMPER', 'LINK_LOST', 'GYRO_FAULT', 'MOTOR_MISSING',
                'MOTOR_HOT', 'BATT_LOW', 'RECOVERY', 'ESTOP', 'REJECT',
                'DOCK_NOT_FOUND', 'ZONE_MISMATCH', 'ZONE_UNCONFIRMED', 'GYRO_SIGN_FLIPPED') else self.get_logger().info
            level(f'Brain: {msg.raw}')
        elif isinstance(msg, proto.Info):
            if msg.text != self.firmware:
                self.firmware = msg.text
                self.pub_fw.publish(String(data=msg.text))
                self.get_logger().info(f'Прошивка: {msg.text}')
                if not msg.text.startswith('ATSD-1M 2.'):
                    self.get_logger().error('Нужна прошивка ATSD-1M 2.x — старый протокол не поддерживается')

    def on_encoders(self, e: proto.Encoders):
        self.last_flags = e.flags
        stamp = self.get_clock().now().to_msg()

        batt = BatteryState()
        batt.header.stamp = stamp
        batt.voltage = e.millivolts / 1000.0
        batt.percentage = float('nan')
        batt.present = e.millivolts > 0
        batt.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        self.pub_batt.publish(batt)
        self.pub_estop.publish(Bool(data=bool(e.flags & proto.FLAG_ESTOP)))

        if e.flags != self.flags_prev:
            self.flags_prev = e.flags
            self.pub_flags.publish(String(data=' '.join(proto.decode_flags(e.flags))))

        if self.odom_source == 'encoders':
            self.encoder_odom(e)

    def encoder_odom(self, e: proto.Encoders):
        now = time.monotonic()
        if self.prev_deg is None:
            self.prev_deg = (e.deg_l, e.deg_r)
            self.prev_t = now
            return
        dl = (e.deg_l - self.prev_deg[0]) * self.mm_per_deg / 1000.0
        dr = (e.deg_r - self.prev_deg[1]) * self.mm_per_deg / 1000.0
        self.prev_deg = (e.deg_l, e.deg_r)
        dt = max(now - self.prev_t, 1e-3)
        self.prev_t = now
        ds = 0.5 * (dl + dr)
        yaw = math.radians(e.heading_mdeg / 1000.0)
        dth = yaw - self.eth
        self.eth = yaw
        self.ex += ds * math.cos(yaw)
        self.ey += ds * math.sin(yaw)
        self.publish_odom(self.ex, self.ey, yaw, ds / dt, dth / dt)

    def publish_odom(self, x, y, yaw, v, w):
        stamp = self.get_clock().now().to_msg()
        q = yaw_to_quat(yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.02
        self.pub_odom.publish(odom)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation = q
            self.tf_bc.sendTransform(t)

    def point_id(self, idx: int):
        return self.point_ids[idx] if 0 <= idx < len(self.point_ids) else None

    def publish_state(self, s: proto.Status):
        data = {
            'state': s.state,
            'from': self.point_id(s.from_idx),
            'to': self.point_id(s.to_idx),
            'wp': s.wp,
            'wp_count': s.wp_count,
            'remain_m': round(s.remain_mm / 1000.0, 2),
            'pause': proto.decode_pause(s.pause),
            'mission_s': round(s.mission_ds / 10.0, 1),
            'flags': proto.decode_flags(self.last_flags),
            'link': True,
            'firmware': self.firmware,
        }
        self.pub_state.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def check_link(self):
        ok = (time.monotonic() - self.last_rx) < self.link_timeout
        now = time.monotonic()
        if ok and not self.firmware and now - getattr(self, 'last_query', 0.0) > 2.0:
            self.last_query = now
            self.send(proto.cmd_simple('?'))
        if ok != self.link_prev:
            self.link_prev = ok
            (self.get_logger().info if ok else self.get_logger().error)(
                'Brain на связи' if ok else 'Нет данных от Brain')
            if not ok:
                self.pub_state.publish(String(data=json.dumps(
                    {'state': 'UNKNOWN', 'link': False, 'pause': [], 'flags': []})))
        self.pub_link.publish(Bool(data=ok))

    def send_heartbeat(self):
        now = time.monotonic()
        dead = [f for f in self.require_feeds
                if now - self.feed_seen.get(f, 0.0) > self.feed_timeout]
        ok = not dead
        if ok != self.feeds_ok_prev:
            self.feeds_ok_prev = ok
            if ok:
                self.get_logger().info('Все источники восприятия на связи')
            else:
                self.get_logger().error(f'Нет данных от: {", ".join(dead)} — heartbeat остановлен, '
                                        f'Brain встанет на паузу')
        if ok:
            self.send(proto.cmd_heartbeat())

    def on_cmd_vel(self, msg: Twist):
        self.cmd_lin = msg.linear.x
        self.cmd_ang = msg.angular.z
        self.last_cmd = time.monotonic()
        self.zero_sent = 0

    def send_cmd_vel(self):
        if self.last_status is not None and self.last_status.state in (
                'TO_PICKUP', 'WAIT_LOAD', 'TO_DROP', 'WAIT_UNLOAD', 'RETURN'):
            return
        if time.monotonic() - self.last_cmd <= self.cmd_timeout:
            self.send(proto.cmd_velocity(self.cmd_lin, self.cmd_ang))
        elif self.zero_sent < 3 and self.last_cmd > 0:
            self.send(proto.cmd_velocity(0.0, 0.0))
            self.zero_sent += 1

    def resolve_point(self, token: str):
        token = token.strip()
        if token.isdigit():
            return int(token)
        if token in self.point_ids:
            return self.point_ids.index(token)
        return None

    def on_mission_start(self, msg: String):
        parts = msg.data.replace(',', ' ').split()
        if len(parts) != 2:
            self.get_logger().error(f'mission/start: ждали "откуда куда", пришло «{msg.data}»')
            return
        a, b = (self.resolve_point(x) for x in parts)
        if a is None or b is None:
            self.get_logger().error(f'mission/start: неизвестная точка в «{msg.data}»')
            return
        self.get_logger().info(f'Старт миссии {parts[0]} -> {parts[1]} ({a} -> {b})')
        self.send(proto.cmd_mission(a, b))

    def on_sign(self, msg: String):
        self.feed_seen['vision'] = time.monotonic()
        parsed = proto.parse_sign_msg(msg.data)
        if parsed:
            self.send(proto.cmd_sign(*parsed))

    def on_light(self, msg: String):
        self.feed_seen['vision'] = time.monotonic()
        parsed = proto.parse_light_msg(msg.data)
        if parsed:
            self.send(proto.cmd_light(*parsed))

    def on_obstacle(self, msg: String):
        self.feed_seen['obstacle'] = time.monotonic()
        try:
            dist, side, depth, offset = proto.parse_obstacle_msg(msg.data)
        except ValueError:
            return
        line = proto.cmd_obstacle(dist, side, depth, offset)
        now = time.monotonic()
        if line == 'O 0\n' and self.last_obst_text == line and now - self.last_obst_sent < 0.25:
            return
        if line != 'O 0\n' and now - self.last_obst_sent < self.obst_period:
            return
        self.last_obst_sent = now
        self.last_obst_text = line
        self.send(line)

    def on_nudge(self, msg: Twist):
        now = time.monotonic()
        if now - self.last_nudge_sent < 0.05:
            return
        self.last_nudge_sent = now
        if abs(msg.linear.x) < 1e-3 and abs(msg.angular.z) < 1e-3:
            return
        self.send(proto.cmd_nudge(msg.linear.x, msg.angular.z))

    def on_spot(self, msg: String):
        if not self.last_flags & proto.FLAG_DOCKING:
            return
        try:
            spot = proto.parse_spot_msg(msg.data)
        except ValueError:
            return
        now = time.monotonic()
        if spot and now - self.last_spot_sent >= 0.1:
            self.last_spot_sent = now
            self.send(proto.cmd_dock(*spot))

    def on_zone(self, msg: String):
        now = time.monotonic()
        if now - self.last_zone_sent < 0.4:
            return
        self.last_zone_sent = now
        try:
            zone = proto.parse_zone_msg(msg.data)
        except ValueError:
            return
        if zone is None:
            self.send(proto.cmd_zone(-1))
            return
        pid, rssi, near = zone
        idx = self.resolve_point(pid)
        self.send(proto.cmd_zone(idx if idx is not None else -1, rssi, near))

    def on_pose_correction(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        self.send(proto.cmd_pose(pose.position.x, pose.position.y, quat_to_yaw(pose.orientation)))

    def destroy_node(self):
        self.running = False
        self.send(proto.cmd_velocity(0.0, 0.0))
        self.close_port()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = V5BridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
