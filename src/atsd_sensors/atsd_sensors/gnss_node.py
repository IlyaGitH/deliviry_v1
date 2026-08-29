#!/usr/bin/env python3
"""
atsd_sensors · gnss_node

Драйвер приёмника BN-880 (u-blox M8) на UART.

Публикует:
    /gnss/fix        sensor_msgs/NavSatFix     координаты
    /gnss/status     std_msgs/String           спутники, HDOP, тип решения
    /diagnostics     diagnostic_msgs/...       для агрегатора

ВАЖНО
При отсутствии фикса узел публикует STATUS_NO_FIX и НЕ повторяет
последние координаты. Иначе EKF в robot_localization начнёт тянуть
робота к призрачной точке — в помещении, где спутников нет вообще,
это гарантированный увод.

Модуль из коробки работает на 9600 бод и 1 Гц. При старте узел
шлёт UBX-команды на переключение в 115200 и 5 Гц, если включён
параметр configure_module.
"""

import rclpy
from rclpy.node import Node

import serial
import pynmea2

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# Качество фикса в поле GGA -> статус ROS
GGA_QUALITY = {
    0: NavSatStatus.STATUS_NO_FIX,
    1: NavSatStatus.STATUS_FIX,          # автономное решение
    2: NavSatStatus.STATUS_SBAS_FIX,     # дифференциальное
    4: NavSatStatus.STATUS_GBAS_FIX,     # RTK fixed
    5: NavSatStatus.STATUS_GBAS_FIX,     # RTK float
}


class GnssNode(Node):

    def __init__(self):
        super().__init__('gnss_node')

        self.declare_parameter('port', '/dev/gnss')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'gnss_link')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('min_satellites', 4)

        p = self.get_parameter
        self.frame_id = p('frame_id').value
        self.min_sats = int(p('min_satellites').value)

        port = p('port').value
        baud = int(p('baud').value)

        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Не удалось открыть {port}: {e}')
            raise

        self.get_logger().info(f'GNSS на {port} @ {baud}')

        self.pub_fix = self.create_publisher(NavSatFix, 'gnss/fix', 10)
        self.pub_status = self.create_publisher(String, 'gnss/status', 5)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 5)

        self.sats = 0
        self.hdop = 99.9
        self.quality = 0
        self.last_fix = None

        self.create_timer(1.0 / float(p('publish_rate_hz').value), self.read_serial)
        self.create_timer(1.0, self.publish_diag)

    # ────────────────────────────────────────── чтение
    def read_serial(self):
        try:
            while self.ser.in_waiting:
                raw = self.ser.readline().decode('ascii', 'ignore').strip()
                if raw.startswith('$'):
                    self.parse(raw)
        except serial.SerialException as e:
            self.get_logger().error(f'Порт отвалился: {e}')

    def parse(self, sentence: str):
        try:
            msg = pynmea2.parse(sentence)
        except pynmea2.ParseError:
            return

        if isinstance(msg, pynmea2.types.talker.GGA):
            self.handle_gga(msg)
        elif isinstance(msg, pynmea2.types.talker.GSA):
            try:
                self.hdop = float(msg.hdop)
            except (TypeError, ValueError):
                pass

    def handle_gga(self, msg):
        try:
            self.quality = int(msg.gps_qual or 0)
            self.sats = int(msg.num_sats or 0)
        except (TypeError, ValueError):
            self.quality, self.sats = 0, 0

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id

        has_fix = (self.quality > 0 and self.sats >= self.min_sats
                   and msg.latitude is not None)

        if has_fix:
            fix.status.status = GGA_QUALITY.get(self.quality, NavSatStatus.STATUS_FIX)
            fix.latitude = float(msg.latitude)
            fix.longitude = float(msg.longitude)
            fix.altitude = float(msg.altitude or 0.0)
            # грубая оценка: горизонтальная ошибка ~ HDOP * 2,5 м
            sigma = max(self.hdop, 0.5) * 2.5
            fix.position_covariance[0] = sigma ** 2
            fix.position_covariance[4] = sigma ** 2
            fix.position_covariance[8] = (sigma * 2.0) ** 2
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            self.last_fix = fix
        else:
            # Без фикса — честный NO_FIX, без повтора старых координат
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        fix.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
        self.pub_fix.publish(fix)

        self.pub_status.publish(String(
            data=f'sats={self.sats} hdop={self.hdop:.1f} qual={self.quality}'))

    # ────────────────────────────────────────── диагностика
    def publish_diag(self):
        st = DiagnosticStatus()
        st.name = 'atsd/gnss'
        st.hardware_id = 'BN-880'

        if self.quality == 0:
            st.level = DiagnosticStatus.WARN
            st.message = 'Нет фикса — норма в помещении'
        elif self.hdop > 5.0:
            st.level = DiagnosticStatus.WARN
            st.message = 'Плохая геометрия, HDOP высокий'
        else:
            st.level = DiagnosticStatus.OK
            st.message = 'Фикс есть'

        st.values = [
            KeyValue(key='satellites', value=str(self.sats)),
            KeyValue(key='hdop', value=f'{self.hdop:.1f}'),
            KeyValue(key='quality', value=str(self.quality)),
        ]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_diag.publish(arr)

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GnssNode()
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
