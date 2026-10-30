#!/usr/bin/env python3
# Гироскоп MPU-6050/6500 (GY-521) по I2C: курс робота для навигации -> /imu/data.
# Положение он не даёт (ускорение дважды по времени уплывает на десятки метров в минуту) —
# только направление; путь считают колёса.
import math
import threading
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from atsd_sensors.imu_logic import DLPF, GYRO_RANGE, WHO_AM_I, GyroYaw, be16

try:
    from smbus2 import SMBus
    I2C_OK = True
except Exception:
    I2C_OK = False

AXIS = {'x': 8, 'y': 10, 'z': 12}  # смещение оси гироскопа в 14 байтах с регистра 0x3B


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        d = self.declare_parameter
        d('i2c_bus', 1)
        d('address', 0x68)          # 0x69, если вывод AD0 платы на 3,3 В
        d('rate_hz', 100.0)
        d('gyro_range_dps', 500)    # ±250 по умолчанию у чипа мало: удар о стык плитки зашкаливает
        d('dlpf_hz', 41)            # срезаем тряску; повороты робота медленнее
        d('yaw_axis', 'z')          # ось, смотрящая вверх; плата на боку — x или y
        d('yaw_sign', 1.0)          # плата вверх ногами — -1
        d('yaw_scale', 1.0)         # калибровка: повернуть на 360° по метке, scale = 360 / показано
        d('publish_hz', 50.0)
        p = lambda n: self.get_parameter(n).value
        self.bus_id, self.addr = int(p('i2c_bus')), int(p('address'))
        self.rate = float(p('rate_hz'))
        self.range_dps = int(p('gyro_range_dps'))
        self.dlpf = int(p('dlpf_hz'))
        self.axis = AXIS[str(p('yaw_axis')).lower()]
        self.yaw = GyroYaw(float(p('yaw_sign')), float(p('yaw_scale')))
        self.lsb = GYRO_RANGE[self.range_dps][1]

        self.pub = self.create_publisher(Imu, 'imu/data', 20)
        self.pub_status = self.create_publisher(String, 'imu/status', 5)
        self.create_subscription(Odometry, 'drive/odom', self.on_odom, 20)
        self.odom_still, self.odom_t = False, 0.0
        self.acc = deque(maxlen=int(self.rate // 2))  # полсекунды модуля ускорения — тряска есть?
        self.chip, self.err = None, ''
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self.loop, daemon=True).start()
        self.create_timer(1.0 / float(p('publish_hz')), self.publish)
        self.create_timer(1.0, self.publish_status)

    def on_odom(self, msg: Odometry):
        t = msg.twist.twist
        self.odom_still = abs(t.linear.x) < 0.01 and abs(t.angular.z) < 0.02
        self.odom_t = time.monotonic()

    def setup(self, bus):
        who = bus.read_byte_data(self.addr, 0x75)
        self.chip = WHO_AM_I.get(who, f'неизвестный чип 0x{who:02X}')
        bus.write_byte_data(self.addr, 0x6B, 0x01)   # проснуться, такт от гироскопа X
        time.sleep(0.05)
        bus.write_byte_data(self.addr, 0x1A, DLPF[self.dlpf])
        bus.write_byte_data(self.addr, 0x19, 0)       # делитель частоты: 1 кГц при включённом DLPF
        bus.write_byte_data(self.addr, 0x1B, GYRO_RANGE[self.range_dps][0] << 3)
        bus.write_byte_data(self.addr, 0x1C, 0x08)    # акселерометр ±4 g
        self.get_logger().info(f'IMU: {self.chip} на I2C {self.bus_id}/0x{self.addr:02X}, '
                               f'±{self.range_dps}°/с, фильтр {self.dlpf} Гц')

    def loop(self):
        if not I2C_OK:
            self.err = 'нет smbus2'
            self.get_logger().error('Нет smbus2 — IMU не читается')
            return
        period = 1.0 / self.rate
        while self.running:
            try:
                with SMBus(self.bus_id) as bus:
                    self.setup(bus)
                    self.err = ''
                    last = time.monotonic()
                    while self.running:
                        raw = bus.read_i2c_block_data(self.addr, 0x3B, 14)
                        now = time.monotonic()
                        dt, last = now - last, now
                        ax, ay, az = (be16(raw[i], raw[i + 1]) / 8192.0 for i in (0, 2, 4))
                        g = be16(raw[self.axis], raw[self.axis + 1])
                        self.acc.append(math.sqrt(ax * ax + ay * ay + az * az))
                        with self.lock:
                            self.yaw.update(g / self.lsb, dt, self.still(now), clipped=abs(g) >= 32700)
                        time.sleep(max(0.0, period - (time.monotonic() - now)))
            except OSError as e:
                self.err = str(e)
                self.get_logger().error(f'IMU не отвечает (0x{self.addr:02X}): {e}', throttle_duration_sec=10.0)
                time.sleep(2.0)

    def still(self, now):
        if now - self.odom_t < 0.5:
            return self.odom_still  # колёса знают лучше всех
        # Brain молчит — стоим, если нет тряски: модуль ускорения ровный в пределах 0,02 g
        return len(self.acc) == self.acc.maxlen and max(self.acc) - min(self.acc) < 0.02

    def publish(self):
        with self.lock:
            if not self.yaw.bias_known:
                return  # ноль гироскопа ещё не известен — курс бы уплывал на градусы в секунду
            yaw, rate = self.yaw.yaw, self.yaw.rate
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.z, msg.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.orientation_covariance[8] = math.radians(2.0) ** 2
        msg.angular_velocity.z = rate
        msg.linear_acceleration_covariance[0] = -1.0  # ускорения не отдаём
        self.pub.publish(msg)

    def publish_status(self):
        with self.lock:
            y = self.yaw
            text = (f'imu {self.chip or "?"}: курс {math.degrees(y.yaw):+.1f}° ноль {y.bias:+.2f}°/с '
                    f'{"" if y.bias_known else "(калибруется: робот должен стоять) "}'
                    f'удары {y.clipped}' + (f' ошибка: {self.err}' if self.err else ''))
        self.pub_status.publish(String(data=text))

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
