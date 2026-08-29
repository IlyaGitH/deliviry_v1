#!/usr/bin/env python3
"""
atsd_actuators · lock_node

Управление соленоидным замком грузового отсека.

Сервис:
    /lock/unlock         std_srvs/Trigger    импульс на соленоид

Публикует:
    /lock/busy           std_msgs/Bool       соленоид под током (интерлок света)
    /lock/closed         std_msgs/Bool       крышка закрыта, по геркону
    /lock/state          std_msgs/String     человекочитаемое состояние

ЛОГИКА
Замок нормально закрытый: без питания язычок выдвинут и держит крышку.
Импульс pulse_time втягивает язычок, пружина выталкивает крышку,
после чего питание снимается — держать соленоид под током нельзя,
у него коэффициент включения 10 процентов и он греется.

Пока идёт импульс, публикуется /lock/busy = true, и light_node
гасит свет: шина 12 В не тянет замок и фары одновременно.

БЕЗОПАСНОСТЬ
Повторный вызов во время импульса игнорируется.
При завершении узла GPIO принудительно уводится в ноль.
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

try:
    from gpiozero import DigitalOutputDevice, Button
    GPIO_OK = True
except Exception:                                  # noqa: BLE001
    GPIO_OK = False


class LockNode(Node):

    def __init__(self):
        super().__init__('lock_node')

        self.declare_parameter('lock_pin', 19)
        self.declare_parameter('door_sensor_pin', 27)
        self.declare_parameter('use_door_sensor', False)
        self.declare_parameter('pulse_time_s', 0.4)
        self.declare_parameter('cooldown_s', 3.0)
        self.declare_parameter('dry_run', False)

        p = self.get_parameter
        self.pulse_time = float(p('pulse_time_s').value)
        self.cooldown = float(p('cooldown_s').value)
        self.dry_run = bool(p('dry_run').value)
        self.use_door = bool(p('use_door_sensor').value)

        self.dev = None
        self.door = None
        if not self.dry_run and GPIO_OK:
            self.dev = DigitalOutputDevice(int(p('lock_pin').value),
                                           active_high=True,
                                           initial_value=False)
            if self.use_door:
                # геркон замыкается на землю при закрытой крышке
                self.door = Button(int(p('door_sensor_pin').value),
                                   pull_up=True, bounce_time=0.05)
        elif not self.dry_run:
            self.get_logger().warn(
                'gpiozero недоступен — узел работает вхолостую.')

        self.busy = False
        self.last_pulse = self.get_clock().now()
        self.lock = threading.Lock()

        self.srv = self.create_service(Trigger, 'lock/unlock', self.on_unlock)
        self.pub_busy = self.create_publisher(Bool, 'lock/busy', 5)
        self.pub_closed = self.create_publisher(Bool, 'lock/closed', 5)
        self.pub_state = self.create_publisher(String, 'lock/state', 5)

        self.create_timer(0.2, self.publish_status)
        self.get_logger().info(
            f'Замок: GPIO{p("lock_pin").value}, импульс {self.pulse_time} с, '
            f'датчик крышки {"вкл" if self.use_door else "выкл"}')

    # ────────────────────────────────────────── сервис
    def on_unlock(self, request, response):
        with self.lock:
            if self.busy:
                response.success = False
                response.message = 'Импульс уже идёт'
                return response

            age = (self.get_clock().now() - self.last_pulse).nanoseconds / 1e9
            if age < self.cooldown:
                response.success = False
                response.message = (f'Пауза после прошлого импульса, '
                                    f'осталось {self.cooldown - age:.1f} с')
                return response

            self.busy = True

        self.pub_busy.publish(Bool(data=True))
        self.get_logger().info('Открываю замок')

        # свету нужно время погаснуть до броска тока
        self.sleep(0.1)
        if self.dev is not None:
            self.dev.on()
        self.sleep(self.pulse_time)
        if self.dev is not None:
            self.dev.off()
        self.sleep(0.2)

        with self.lock:
            self.busy = False
            self.last_pulse = self.get_clock().now()
        self.pub_busy.publish(Bool(data=False))

        response.success = True
        response.message = 'Замок открыт'
        return response

    def sleep(self, seconds: float):
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=seconds))

    # ────────────────────────────────────────── статус
    def publish_status(self):
        closed = True
        if self.door is not None:
            closed = self.door.is_pressed
        self.pub_closed.publish(Bool(data=closed))
        self.pub_busy.publish(Bool(data=self.busy))
        self.pub_state.publish(String(
            data=f'busy={int(self.busy)} closed={int(closed)}'))

    def destroy_node(self):
        if self.dev is not None:
            self.dev.off()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LockNode()
        # многопоточный исполнитель: сервис блокирует поток на время импульса
        from rclpy.executors import MultiThreadedExecutor
        ex = MultiThreadedExecutor(num_threads=2)
        ex.add_node(node)
        try:
            ex.spin()
        finally:
            ex.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
