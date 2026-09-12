#!/usr/bin/env python3
"""
atsd_sensors · lidar_node

Драйвер лазерного дальномера LDROBOT D500 Kit (семейство LD19)
напрямую с аппаратного UART Raspberry Pi 5. Без SDK и без сторонних
ROS-пакетов: протокол LD19 открытый и умещается в один файл.

Публикует:
    /scan            sensor_msgs/LaserScan     кадр развёртки
    /diagnostics     diagnostic_msgs/...       обороты, брак CRC, темп

Почему свой узел, а не ydlidar_ros2_driver
    D500 — это LD19, а не X2L. Протоколы разные, драйвер YDLIDAR его
    не видит вовсе. Официальный ldlidar_stl_ros2 собирается из
    исходников, тянет CMake-пакет и лишнюю сборку на Pi перед сдачей.
    Здесь тот же результат на 200 строках и без зависимостей.

Подключение (ZH1.5T-4P → колодка Pi)
    P5V  → пин 4  (5 В)
    GND  → пин 6
    TX   → пин 10 (GPIO15, RXD0)      лидар только передаёт
    PWM  → пин 9  (GND)               мотор на максимум оборотов
    230400 бод, 8N1, порт /dev/lidar → ttyAMA0

Обязательно: в raspi-config консоль на последовательном порту
выключена, сам порт включён. Иначе getty съедает поток.

Формат кадра, 47 байт
    0x54 | 0x2C | скорость(2) | старт.угол(2) |
    12 × { дальность мм (2) + сила сигнала (1) } |
    кон.угол(2) | метка времени(2) | CRC8(1)

Углы приходят по часовой стрелке, ROS считает против — узел
разворачивает их сам (параметр invert).
"""

import math
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

import serial

from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


HEADER = 0x54          # признак начала кадра
VER_LEN = 0x2C         # версия протокола + 12 точек в кадре
POINTS = 12
PKT_LEN = 47


def _crc_table(poly: int = 0x4D):
    """Таблица CRC8 LDROBOT: полином 0x4D, старшим битом вперёд."""
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return table


CRC_TABLE = _crc_table()


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        self.declare_parameter('port', '/dev/lidar')
        self.declare_parameter('baud', 230400)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('topic', 'scan')
        self.declare_parameter('bins', 455)            # точек в кадре /scan
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('min_intensity', 0)     # отсев слабых засветок
        self.declare_parameter('invert', True)         # по часовой → против
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('check_crc', True)
        # Сектора, которые закрывает сам робот: плоский список пар
        # градусов в системе ROS, например [150.0, 210.0] — корма.
        self.declare_parameter('crop_deg', [0.0, 0.0])

        p = self.get_parameter
        self.port = p('port').value
        self.baud = int(p('baud').value)
        self.frame_id = p('frame_id').value
        self.bins = max(60, int(p('bins').value))
        self.range_min = float(p('range_min').value)
        self.range_max = float(p('range_max').value)
        self.min_intensity = int(p('min_intensity').value)
        self.invert = bool(p('invert').value)
        self.offset = float(p('angle_offset_deg').value)
        self.check_crc = bool(p('check_crc').value)

        crop = [float(x) for x in p('crop_deg').value]
        self.crop = [(crop[i], crop[i + 1]) for i in range(0, len(crop) - 1, 2)
                     if crop[i] != crop[i + 1]]

        self.pub_scan = self.create_publisher(LaserScan, p('topic').value,
                                              qos_profile_sensor_data)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 5)

        # состояние сборки кадра
        self._pts = []
        self._acc_deg = 0.0
        self._last_start = None
        self._scan_start = self.get_clock().now()
        self._queue = deque(maxlen=4)
        self._lock = threading.Lock()

        # счётчики для диагностики
        self.rpm = 0.0
        self.crc_errors = 0
        self.scans = 0
        self.last_pkt = self.get_clock().now()
        self.last_points = 0

        self.ser = None
        self._alive = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

        self.create_timer(0.01, self._drain)      # публикация из основного потока
        self.create_timer(1.0, self._diag)

        self.get_logger().info(
            f'Лидар LD19/D500: {self.port} @ {self.baud}, '
            f'{self.bins} точек в кадре, кадр {self.frame_id}')

    # ───────────────────────────────────────────── поток чтения
    def _open(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.ser.reset_input_buffer()
            return True
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Не открывается {self.port}: {e}')
            self.ser = None
            return False

    def _reader(self):
        buf = bytearray()
        while self._alive:
            if self.ser is None:
                if not self._open():
                    # порт может появиться позже — например, лидар
                    # запитался от отдельной ветки 5 В
                    time.sleep(2.0)
                    continue

            try:
                chunk = self.ser.read(256)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Порт отвалился: {e}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                continue

            if not chunk:
                continue

            buf.extend(chunk)
            if len(buf) > 8192:               # защита от мусора на линии
                del buf[:-2048]

            while len(buf) >= PKT_LEN:
                if buf[0] != HEADER or buf[1] != VER_LEN:
                    del buf[0]
                    continue
                pkt = bytes(buf[:PKT_LEN])
                if self.check_crc and crc8(pkt[:PKT_LEN - 1]) != pkt[PKT_LEN - 1]:
                    self.crc_errors += 1
                    del buf[0]
                    continue
                del buf[:PKT_LEN]
                self._packet(pkt)

    # ───────────────────────────────────────────── разбор кадра
    def _packet(self, pkt: bytes):
        speed = int.from_bytes(pkt[2:4], 'little')          # град/с
        start = int.from_bytes(pkt[4:6], 'little') / 100.0
        end = int.from_bytes(pkt[42:44], 'little') / 100.0

        self.rpm = speed / 6.0
        self.last_pkt = self.get_clock().now()

        span = (end - start) % 360.0
        step = span / (POINTS - 1) if POINTS > 1 else 0.0

        for i in range(POINTS):
            off = 6 + i * 3
            dist_mm = int.from_bytes(pkt[off:off + 2], 'little')
            intensity = pkt[off + 2]
            if dist_mm == 0 or intensity < self.min_intensity:
                continue
            self._pts.append(((start + step * i) % 360.0,
                              dist_mm / 1000.0, float(intensity)))

        # Оборот считаем по началу пакета, а не по его длине: внутри
        # кадра лежат 11 шагов из 12, и сумма длин до 360° не дотягивает.
        if self._last_start is not None:
            self._acc_deg += (start - self._last_start) % 360.0
        wrapped = self._last_start is not None and start < self._last_start
        self._last_start = start

        if (wrapped and self._acc_deg > 180.0) or self._acc_deg > 400.0 \
                or len(self._pts) > 3000:
            self._build_scan()

    def _build_scan(self):
        now = self.get_clock().now()
        scan_time = (now - self._scan_start).nanoseconds * 1e-9
        pts, self._pts = self._pts, []
        self._acc_deg = 0.0
        self._scan_start = now

        # Мусорный или слишком длинный кадр пропускаем: лучше дырка
        # в потоке, чем растянутая по времени развёртка в costmap.
        if not (0.03 < scan_time < 0.5) or len(pts) < 40:
            return

        n = self.bins
        inf = float('inf')
        ranges = [inf] * n
        intens = [0.0] * n

        for raw_ang, dist, q in pts:
            a = -raw_ang if self.invert else raw_ang
            a = (a + self.offset + 180.0) % 360.0 - 180.0
            if self._cropped(a):
                continue
            if dist < self.range_min or dist > self.range_max:
                continue
            idx = int((a + 180.0) / 360.0 * n)
            if idx >= n:
                idx = n - 1
            # в один бин попадает несколько отсчётов — берём ближний:
            # для объезда препятствий безопаснее ошибиться в меньшую
            if dist < ranges[idx]:
                ranges[idx] = dist
                intens[idx] = q

        msg = LaserScan()
        msg.header.stamp = (now - Duration(seconds=scan_time)).to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = -math.pi
        msg.angle_increment = 2.0 * math.pi / n
        msg.angle_max = math.pi - msg.angle_increment
        msg.time_increment = scan_time / n
        msg.scan_time = scan_time
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = intens

        self.scans += 1
        self.last_points = len(pts)
        with self._lock:
            self._queue.append(msg)

    def _cropped(self, ang_deg: float) -> bool:
        for lo, hi in self.crop:
            if lo <= hi:
                if lo <= ang_deg <= hi:
                    return True
            else:                       # сектор через ±180°
                if ang_deg >= lo or ang_deg <= hi:
                    return True
        return False

    # ───────────────────────────────────────────── публикация
    def _drain(self):
        while True:
            with self._lock:
                if not self._queue:
                    return
                msg = self._queue.popleft()
            self.pub_scan.publish(msg)

    # ───────────────────────────────────────────── диагностика
    def _diag(self):
        silence = (self.get_clock().now() - self.last_pkt).nanoseconds * 1e-9

        st = DiagnosticStatus()
        st.name = 'atsd/lidar'
        st.hardware_id = 'LDROBOT D500 (LD19)'

        if self.ser is None or silence > 2.0:
            st.level = DiagnosticStatus.ERROR
            st.message = 'Нет данных с лидара'
        elif not (240.0 <= self.rpm <= 900.0):
            # штатные 10 Гц развёртки = 600 об/мин; поле speed в град/с
            st.level = DiagnosticStatus.WARN
            st.message = f'Нештатные обороты: {self.rpm:.0f} об/мин'
        elif self.crc_errors > 50:
            st.level = DiagnosticStatus.WARN
            st.message = 'Много битых кадров, проверьте шлейф и землю'
        else:
            st.level = DiagnosticStatus.OK
            st.message = 'Развёртка идёт'

        st.values = [
            KeyValue(key='rpm', value=f'{self.rpm:.0f}'),
            KeyValue(key='scans', value=str(self.scans)),
            KeyValue(key='points_last_scan', value=str(self.last_points)),
            KeyValue(key='crc_errors', value=str(self.crc_errors)),
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
        node = LidarNode()
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
