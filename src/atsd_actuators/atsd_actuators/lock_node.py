#!/usr/bin/env python3
import json
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from atsd_actuators.pins import DOOR_SENSOR, LOCK, check_pin, check_unique

try:
    from gpiozero import Button, DigitalOutputDevice
    GPIO_OK = True
except Exception:
    GPIO_OK = False

TRAVEL = ('TO_PICKUP', 'TO_DROP', 'RETURN')


class LockNode(Node):

    def __init__(self):
        super().__init__('lock_node')
        d = self.declare_parameter
        d('lock_pin', LOCK)
        d('door_sensor_pin', DOOR_SENSOR)
        d('use_door_sensor', False)
        d('pulse_time_s', 0.4)
        d('cooldown_s', 3.0)
        d('block_while_moving', True)
        d('zone_policy', 'warn')
        d('dry_run', False)

        p = lambda n: self.get_parameter(n).value
        self.lock_pin = check_pin(p('lock_pin'), 'замок')
        self.door_pin = check_pin(p('door_sensor_pin'), 'геркон крышки')
        check_unique({'замок': self.lock_pin, 'геркон крышки': self.door_pin})
        self.pulse_time = float(p('pulse_time_s'))
        self.cooldown = float(p('cooldown_s'))
        self.block_moving = bool(p('block_while_moving'))
        self.use_door = bool(p('use_door_sensor'))
        self.zone_policy = str(p('zone_policy')).lower()
        self.zone = None
        self.zone_ts = 0.0
        self.mission_from = self.mission_to = None
        dry = bool(p('dry_run'))

        self.dev = None
        self.door = None
        if not dry and GPIO_OK:
            self.dev = DigitalOutputDevice(self.lock_pin, active_high=True, initial_value=False)
            if self.use_door:
                self.door = Button(self.door_pin, pull_up=True, bounce_time=0.05)
        elif not dry:
            self.get_logger().warn('gpiozero недоступен — узел работает вхолостую.')

        self.busy = False
        self.last_pulse = 0.0
        self.mission_state = 'UNKNOWN'
        self.tamper = False
        self.guard = threading.Lock()

        srv_group = MutuallyExclusiveCallbackGroup()
        io_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, 'lock/unlock', self.on_unlock, callback_group=srv_group)
        self.create_subscription(String, 'mission/state', self.on_state, 10, callback_group=io_group)
        self.create_subscription(String, 'ble/zone', self.on_zone, 10, callback_group=io_group)
        self.pub_busy = self.create_publisher(Bool, 'lock/busy', 5)
        self.pub_closed = self.create_publisher(Bool, 'lock/closed', 5)
        self.pub_tamper = self.create_publisher(Bool, 'lock/tamper', 5)
        self.pub_state = self.create_publisher(String, 'lock/state', 5)
        self.pub_event = self.create_publisher(String, 'mission/event', 10)
        self.create_timer(0.2, self.publish_status, callback_group=io_group)

        self.get_logger().info(
            f'Замок: GPIO{self.lock_pin}, импульс {self.pulse_time} с, датчик крышки '
            f'{"GPIO" + str(self.door_pin) if self.use_door else "выкл"}, '
            f'блокировка в пути {"вкл" if self.block_moving else "выкл"}')

    def on_state(self, msg: String):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        state = data.get('state', 'UNKNOWN')
        self.mission_from, self.mission_to = data.get('from'), data.get('to')
        if state != self.mission_state and state not in TRAVEL:
            self.tamper = False
        self.mission_state = state

    def on_zone(self, msg: String):
        parts = msg.data.split()
        self.zone = (parts[0], len(parts) > 2 and parts[2] == '1') if len(parts) >= 2 else None
        self.zone_ts = time.monotonic()

    def zone_confirmed(self):
        expected = {'WAIT_LOAD': self.mission_from, 'WAIT_UNLOAD': self.mission_to}.get(self.mission_state)
        if expected is None:
            return True, ''
        fresh = self.zone is not None and time.monotonic() - self.zone_ts < 3.0
        if fresh and self.zone[1] and self.zone[0] == expected:
            return True, ''
        return False, expected

    def closed(self) -> bool:
        return self.door.is_pressed if self.door is not None else True

    def on_unlock(self, request, response):
        with self.guard:
            if self.block_moving and self.mission_state in TRAVEL:
                response.success = False
                response.message = 'Робот в пути — отсек заблокирован'
                return response
            if self.busy:
                response.success = False
                response.message = 'Импульс уже идёт'
                return response
            if self.zone_policy != 'off':
                ok, expected = self.zone_confirmed()
                if not ok:
                    self.pub_event.publish(String(data=f'ZONE_UNCONFIRMED_UNLOCK {expected}'))
                    if self.zone_policy == 'require':
                        response.success = False
                        response.message = f'Нет BLE-метки точки {expected} рядом с роботом'
                        return response
            age = time.monotonic() - self.last_pulse
            if age < self.cooldown:
                response.success = False
                response.message = f'Пауза после прошлого импульса, осталось {self.cooldown - age:.1f} с'
                return response
            self.busy = True

        self.pub_busy.publish(Bool(data=True))
        self.get_logger().info('Открываю замок')
        try:
            time.sleep(0.1)
            if self.dev is not None:
                self.dev.on()
            time.sleep(self.pulse_time)
        finally:
            if self.dev is not None:
                self.dev.off()
        time.sleep(0.2)

        with self.guard:
            self.busy = False
            self.last_pulse = time.monotonic()
        self.pub_busy.publish(Bool(data=False))
        response.success = True
        response.message = 'Замок открыт'
        return response

    def publish_status(self):
        closed = self.closed()
        if not closed and self.mission_state in TRAVEL and not self.tamper:
            self.tamper = True
            self.get_logger().error('Крышку открыли в пути!')
            self.pub_event.publish(String(data='LOCK_TAMPER'))
        self.pub_closed.publish(Bool(data=closed))
        self.pub_busy.publish(Bool(data=self.busy))
        self.pub_tamper.publish(Bool(data=self.tamper))
        self.pub_state.publish(String(
            data=f'busy={int(self.busy)} closed={int(closed)} tamper={int(self.tamper)} '
                 f'mission={self.mission_state}'))

    def destroy_node(self):
        if self.dev is not None:
            self.dev.off()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    ex = None
    try:
        node = LockNode()
        ex = MultiThreadedExecutor(num_threads=2)
        ex.add_node(node)
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if ex is not None:
            ex.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
