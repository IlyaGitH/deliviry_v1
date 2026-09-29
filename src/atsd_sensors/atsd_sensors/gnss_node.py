#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node

import serial
import pynmea2

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


KNOTS_TO_MS = 0.514444

GGA_QUALITY = {
    0: NavSatStatus.STATUS_NO_FIX,
    1: NavSatStatus.STATUS_FIX,
    2: NavSatStatus.STATUS_SBAS_FIX,
    4: NavSatStatus.STATUS_GBAS_FIX,
    5: NavSatStatus.STATUS_GBAS_FIX,
}

NMEA_ID = {'GGA': 0x00, 'GLL': 0x01, 'GSA': 0x02,
           'GSV': 0x03, 'RMC': 0x04, 'VTG': 0x05, 'ZDA': 0x08}


def ubx(cls_id: int, msg_id: int, payload: bytes = b'') -> bytes:
    body = bytes([cls_id, msg_id]) + len(payload).to_bytes(2, 'little') + payload
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b'\xB5\x62' + body + bytes([ck_a, ck_b])


class GnssNode(Node):

    def __init__(self):
        super().__init__('gnss_node')

        self.declare_parameter('port', '/dev/gnss')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'gnss_link')
        self.declare_parameter('min_satellites', 4)
        self.declare_parameter('configure_module', True)
        self.declare_parameter('nav_rate_hz', 5.0)
        self.declare_parameter('save_config', True)
        self.declare_parameter('fix_timeout_s', 3.0)
        self.declare_parameter('uere_m', 2.5)

        p = self.get_parameter
        self.port = p('port').value
        self.target_baud = int(p('baud').value)
        self.frame_id = p('frame_id').value
        self.min_sats = int(p('min_satellites').value)
        self.do_config = bool(p('configure_module').value)
        self.nav_rate = float(p('nav_rate_hz').value)
        self.save_cfg = bool(p('save_config').value)
        self.fix_timeout = float(p('fix_timeout_s').value)
        self.uere = float(p('uere_m').value)

        self.pub_fix = self.create_publisher(NavSatFix, 'gnss/fix', 10)
        self.pub_vel = self.create_publisher(TwistWithCovarianceStamped,
                                             'gnss/vel', 10)
        self.pub_status = self.create_publisher(String, 'gnss/status', 5)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 5)

        self.sats_used = 0
        self.sats_view = 0
        self.hdop = 99.9
        self.vdop = 99.9
        self.quality = 0
        self.fix_type = 1
        self.speed_ms = 0.0
        self.course_deg = None
        self.last_gga = None
        self.baud = None

        self.ser = None
        self._alive = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

        self.create_timer(0.5, self._watchdog)
        self.create_timer(1.0, self._diag)

    def _probe(self, baud: int, seconds: float = 1.2) -> bool:
        try:
            self.ser = serial.Serial(self.port, baud, timeout=0.2)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Не открывается {self.port}: {e}')
            self.ser = None
            return False
        self.ser.reset_input_buffer()
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                line = self.ser.readline()
            except (serial.SerialException, OSError):
                break
            if line.startswith(b'$') and b'*' in line:
                return True
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        return False

    def _open(self) -> bool:
        candidates = [self.target_baud, 9600, 38400, 115200, 57600]
        seen = []
        for b in candidates:
            if b in seen:
                continue
            seen.append(b)
            if self._probe(b):
                self.baud = b
                self.get_logger().info(f'GNSS найден на {self.port} @ {b}')
                break
        else:
            self.get_logger().warn(
                f'Модуль молчит на {self.port}. Проверьте питание, TX/RX '
                f'и что порт не занят консолью')
            return False

        if self.do_config:
            try:
                self._configure()
            except serial.SerialException as e:
                self.get_logger().error(f'Настройка не прошла: {e}')
        return self.ser is not None

    def _configure(self):
        for name, rate in (('GGA', 1), ('GSA', 1), ('RMC', 1),
                           ('VTG', 0), ('GLL', 0), ('GSV', 5)):
            self.ser.write(ubx(0x06, 0x01, bytes([0xF0, NMEA_ID[name], rate])))
            time.sleep(0.02)

        meas_ms = max(100, int(round(1000.0 / max(self.nav_rate, 0.5))))
        self.ser.write(ubx(0x06, 0x08,
                           meas_ms.to_bytes(2, 'little') +
                           (1).to_bytes(2, 'little') +
                           (1).to_bytes(2, 'little')))
        time.sleep(0.05)

        if self.baud != self.target_baud:
            payload = (bytes([0x01, 0x00]) +
                       (0x0000).to_bytes(2, 'little') +
                       (0x000008D0).to_bytes(4, 'little') +
                       self.target_baud.to_bytes(4, 'little') +
                       (0x0003).to_bytes(2, 'little') +
                       (0x0003).to_bytes(2, 'little') +
                       (0x0000).to_bytes(2, 'little') +
                       (0x0000).to_bytes(2, 'little'))
            self.ser.write(ubx(0x06, 0x00, payload))
            self.ser.flush()
            time.sleep(0.2)
            self.ser.close()
            if self._probe(self.target_baud, seconds=2.0):
                self.baud = self.target_baud
                self.get_logger().info(f'Переключился на {self.baud} бод')
            else:
                self.get_logger().warn(
                    'После смены скорости модуль не отозвался, '
                    'возвращаюсь на прежнюю')
                if not self._probe(self.baud, seconds=2.0):
                    return

        if self.save_cfg:
            self.ser.write(ubx(0x06, 0x09,
                               (0x00000000).to_bytes(4, 'little') +
                               (0x0000061F).to_bytes(4, 'little') +
                               (0x00000000).to_bytes(4, 'little') +
                               bytes([0x01])))
            time.sleep(0.05)

    def _reader(self):
        while self._alive:
            if self.ser is None:
                if not self._open():
                    time.sleep(3.0)
                continue
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Порт отвалился: {e}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                continue
            if not raw:
                continue
            line = raw.decode('ascii', 'ignore').strip()
            if line.startswith('$'):
                self._parse(line)

    def _parse(self, sentence: str):
        try:
            msg = pynmea2.parse(sentence)
        except (pynmea2.ParseError, pynmea2.ChecksumError, ValueError):
            return

        t = msg.sentence_type if hasattr(msg, 'sentence_type') else ''
        if t == 'GGA':
            self._on_gga(msg)
        elif t == 'GSA':
            self._on_gsa(msg)
        elif t == 'RMC':
            self._on_rmc(msg)
        elif t == 'GSV':
            try:
                self.sats_view = int(msg.num_sv_in_view)
            except (TypeError, ValueError, AttributeError):
                pass

    def _on_gsa(self, msg):
        for attr, setter in (('hdop', 'hdop'), ('vdop', 'vdop')):
            try:
                setattr(self, setter, float(getattr(msg, attr)))
            except (TypeError, ValueError, AttributeError):
                pass
        try:
            self.fix_type = int(msg.mode_fix_type)
        except (TypeError, ValueError, AttributeError):
            pass

    def _on_rmc(self, msg):
        try:
            self.speed_ms = float(msg.spd_over_grnd or 0.0) * KNOTS_TO_MS
        except (TypeError, ValueError):
            self.speed_ms = 0.0
        try:
            self.course_deg = float(msg.true_course)
        except (TypeError, ValueError):
            self.course_deg = None

        if msg.status != 'A':
            return

        vel = TwistWithCovarianceStamped()
        vel.header.stamp = self.get_clock().now().to_msg()
        vel.header.frame_id = self.frame_id
        if self.course_deg is not None:
            c = math.radians(self.course_deg)
            vel.twist.twist.linear.x = self.speed_ms * math.sin(c)
            vel.twist.twist.linear.y = self.speed_ms * math.cos(c)
        sigma_v = 0.1 if self.speed_ms > 0.5 else 0.3
        vel.twist.covariance[0] = sigma_v ** 2
        vel.twist.covariance[7] = sigma_v ** 2
        vel.twist.covariance[14] = (sigma_v * 3.0) ** 2
        self.pub_vel.publish(vel)

    def _on_gga(self, msg):
        self.last_gga = self.get_clock().now()
        try:
            self.quality = int(msg.gps_qual or 0)
            self.sats_used = int(msg.num_sats or 0)
        except (TypeError, ValueError):
            self.quality, self.sats_used = 0, 0

        has_coords = bool(getattr(msg, 'lat', '')) and bool(getattr(msg, 'lon', ''))
        has_fix = (self.quality > 0 and self.sats_used >= self.min_sats
                   and has_coords)

        self._publish_fix(has_fix, msg if has_fix else None)

        self.pub_status.publish(String(data=(
            f'sats={self.sats_used}/{self.sats_view} hdop={self.hdop:.1f} '
            f'qual={self.quality} fix={self.fix_type}D v={self.speed_ms:.1f}')))

    def _publish_fix(self, has_fix: bool, msg=None):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id
        fix.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS

        if has_fix and msg is not None:
            fix.status.status = GGA_QUALITY.get(self.quality,
                                                NavSatStatus.STATUS_FIX)
            fix.latitude = float(msg.latitude)
            fix.longitude = float(msg.longitude)
            fix.altitude = float(msg.altitude or 0.0)
            sigma = max(self.hdop, 0.5) * self.uere
            sigma_v = max(self.vdop if self.vdop < 50.0 else self.hdop,
                          0.5) * self.uere
            fix.position_covariance[0] = sigma ** 2
            fix.position_covariance[4] = sigma ** 2
            fix.position_covariance[8] = sigma_v ** 2
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.pub_fix.publish(fix)

    def _watchdog(self):
        if self.last_gga is None:
            return
        silence = (self.get_clock().now() - self.last_gga).nanoseconds * 1e-9
        if silence > self.fix_timeout:
            self.quality = 0
            self.sats_used = 0
            self.fix_type = 1
            self._publish_fix(False)

    def _diag(self):
        st = DiagnosticStatus()
        st.name = 'atsd/gnss'
        st.hardware_id = 'BN-880 (u-blox M8N)'

        silence = (0.0 if self.last_gga is None else
                   (self.get_clock().now() - self.last_gga).nanoseconds * 1e-9)

        if self.ser is None:
            st.level = DiagnosticStatus.ERROR
            st.message = 'Порт не открыт'
        elif self.last_gga is None or silence > self.fix_timeout:
            st.level = DiagnosticStatus.ERROR
            st.message = 'Модуль молчит — проверьте провод и питание'
        elif self.quality == 0:
            st.level = DiagnosticStatus.WARN
            st.message = 'Нет фикса — норма в помещении'
        elif self.hdop > 5.0:
            st.level = DiagnosticStatus.WARN
            st.message = 'Плохая геометрия, HDOP высокий'
        else:
            st.level = DiagnosticStatus.OK
            st.message = f'Фикс {self.fix_type}D, спутников {self.sats_used}'

        st.values = [
            KeyValue(key='satellites_used', value=str(self.sats_used)),
            KeyValue(key='satellites_in_view', value=str(self.sats_view)),
            KeyValue(key='hdop', value=f'{self.hdop:.1f}'),
            KeyValue(key='quality', value=str(self.quality)),
            KeyValue(key='fix_type', value=f'{self.fix_type}D'),
            KeyValue(key='speed_ms', value=f'{self.speed_ms:.2f}'),
            KeyValue(key='baud', value=str(self.baud)),
            KeyValue(key='silence_s', value=f'{silence:.1f}'),
        ]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_diag.publish(arr)

    def destroy_node(self):
        self._alive = False
        try:
            if self.ser is not None:
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
