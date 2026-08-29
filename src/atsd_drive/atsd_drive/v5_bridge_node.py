#!/usr/bin/env python3
"""
atsd_drive · v5_bridge_node

Мост между Raspberry Pi и VEX V5 Brain по USB-serial.

Подписывается:
    /cmd_vel            geometry_msgs/Twist     команды скорости

Публикует:
    /drive/odom         nav_msgs/Odometry       одометрия по энкодерам
    /drive/battery      sensor_msgs/BatteryState напряжение батареи V5
    /drive/estop        std_msgs/Bool           аварийная кнопка нажата
    /drive/link         std_msgs/Bool           связь с брейном жива
    TF: odom -> base_link

Протокол (см. прошивку v5_final_main.cpp):
    Pi -> V5:  V <лин_мм_с> <угл_мрад_с>
    V5 -> Pi:  E <градЛ> <градП> <курс_мград> <мс> <мВ> <флаги>
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class V5BridgeNode(Node):

    def __init__(self):
        super().__init__('v5_bridge')

        # ── параметры ──
        self.declare_parameter('port', '/dev/v5')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_diameter_mm', 101.6)
        self.declare_parameter('track_mm', 410.0)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('cmd_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_brain_heading', False)

        p = self.get_parameter
        self.port = p('port').value
        self.baud = int(p('baud').value)
        self.wheel_d = float(p('wheel_diameter_mm').value)
        self.track = float(p('track_mm').value)
        self.gear = float(p('gear_ratio').value)
        self.cmd_timeout = float(p('cmd_timeout_s').value)
        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value
        self.publish_tf = bool(p('publish_tf').value)
        self.use_brain_heading = bool(p('use_brain_heading').value)

        self.mm_per_deg = math.pi * self.wheel_d / 360.0 / self.gear

        # ── состояние ──
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_deg_l = None
        self.prev_deg_r = None
        self.prev_stamp = None
        self.v_lin = 0.0
        self.v_ang = 0.0

        self.cmd_lin_mm = 0
        self.cmd_ang_mrad = 0
        self.last_cmd_time = self.get_clock().now()
        self.link_ok = False
        self.lock = threading.Lock()

        # ── порт ──
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Не удалось открыть {self.port}: {e}')
            raise

        self.get_logger().info(f'Мост поднят на {self.port} @ {self.baud}')

        # ── интерфейсы ──
        qos_cmd = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, qos_cmd)
        self.pub_odom = self.create_publisher(Odometry, 'drive/odom', 20)
        self.pub_batt = self.create_publisher(BatteryState, 'drive/battery', 5)
        self.pub_estop = self.create_publisher(Bool, 'drive/estop', 5)
        self.pub_link = self.create_publisher(Bool, 'drive/link', 5)
        self.tf_bc = TransformBroadcaster(self)

        period = 1.0 / float(p('cmd_rate_hz').value)
        self.create_timer(period, self.send_cmd)

        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

    # ────────────────────────────────────────────── приём
    def rx_loop(self):
        buf = b''
        while rclpy.ok():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                self.get_logger().error(f'Порт отвалился: {e}')
                return
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self.handle_line(line.decode('ascii', 'ignore').strip())

    def handle_line(self, text: str):
        if not text.startswith('E '):
            if text:
                self.get_logger().debug(f'V5: {text}')
            return

        parts = text.split()
        if len(parts) < 6:
            return
        try:
            deg_l = int(parts[1])
            deg_r = int(parts[2])
            head_mdeg = int(parts[3])
            millivolts = int(parts[5])
            flags = int(parts[6]) if len(parts) >= 7 else 0
        except ValueError:
            return

        now = self.get_clock().now()
        self.publish_state(deg_l, deg_r, head_mdeg, millivolts, flags, now)

    # ────────────────────────────────────────────── одометрия
    def publish_state(self, deg_l, deg_r, head_mdeg, millivolts, flags, now):
        estop = bool(flags & 1)
        self.link_ok = True

        if self.prev_deg_l is None:
            self.prev_deg_l, self.prev_deg_r = deg_l, deg_r
            self.prev_stamp = now
            return

        d_l_m = (deg_l - self.prev_deg_l) * self.mm_per_deg / 1000.0
        d_r_m = (deg_r - self.prev_deg_r) * self.mm_per_deg / 1000.0
        self.prev_deg_l, self.prev_deg_r = deg_l, deg_r

        dt = (now - self.prev_stamp).nanoseconds / 1e9
        self.prev_stamp = now
        if dt <= 0.0:
            return

        ds = (d_l_m + d_r_m) / 2.0
        dth = (d_r_m - d_l_m) / (self.track / 1000.0)

        if self.use_brain_heading:
            self.th = math.radians(head_mdeg / 1000.0)
        else:
            self.th += dth

        self.th = math.atan2(math.sin(self.th), math.cos(self.th))
        self.x += ds * math.cos(self.th)
        self.y += ds * math.sin(self.th)

        self.v_lin = ds / dt
        self.v_ang = dth / dt

        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.th)
        odom.twist.twist.linear.x = self.v_lin
        odom.twist.twist.angular.z = self.v_ang
        # Диагональ ковариации. Значения подобраны эмпирически:
        # колёсная одометрия на танковом шасси врёт в основном по курсу.
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.08
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.08
        self.pub_odom.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = yaw_to_quat(self.th)
            self.tf_bc.sendTransform(t)

        batt = BatteryState()
        batt.header.stamp = stamp
        batt.voltage = millivolts / 1000.0
        batt.present = True
        batt.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        self.pub_batt.publish(batt)

        self.pub_estop.publish(Bool(data=estop))
        self.pub_link.publish(Bool(data=True))

    # ────────────────────────────────────────────── передача
    def on_cmd_vel(self, msg: Twist):
        with self.lock:
            self.cmd_lin_mm = int(msg.linear.x * 1000.0)
            self.cmd_ang_mrad = int(msg.angular.z * 1000.0)
            self.last_cmd_time = self.get_clock().now()

    def send_cmd(self):
        with self.lock:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if age > self.cmd_timeout:
                lin, ang = 0, 0
            else:
                lin, ang = self.cmd_lin_mm, self.cmd_ang_mrad
        try:
            self.ser.write(f'V {lin} {ang}\n'.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Запись в порт не удалась: {e}')

    def destroy_node(self):
        try:
            self.ser.write(b'S\n')
            self.ser.flush()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
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
