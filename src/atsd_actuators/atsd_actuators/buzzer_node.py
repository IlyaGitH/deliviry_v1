#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from atsd_actuators.pins import BUZZER, check_pin

try:
    from gpiozero import DigitalOutputDevice
    GPIO_OK = True
except Exception:
    GPIO_OK = False

PATTERNS = {
    'off':      ((), False),
    'beep':     ((0.12, 0.0), False),
    'double':   ((0.10, 0.10, 0.10, 0.0), False),
    'start':    ((0.15, 0.10, 0.15, 0.10, 0.40, 0.0), False),
    'arrive':   ((0.50, 0.15, 0.50, 0.0), False),
    'approach': ((0.15, 0.45), True),
    'reverse':  ((0.30, 0.30), True),
    'alarm':    ((0.25, 0.25), True),
    'fault':    ((0.70, 0.30), True),
}


class BuzzerNode(Node):

    def __init__(self):
        super().__init__('buzzer_node')
        self.declare_parameter('buzzer_pin', BUZZER)
        self.declare_parameter('enabled', True)
        self.declare_parameter('max_continuous_s', 2.0)
        self.declare_parameter('dry_run', False)
        p = lambda n: self.get_parameter(n).value

        self.pin = check_pin(p('buzzer_pin'), 'зуммер')
        self.enabled = bool(p('enabled'))
        self.max_on = float(p('max_continuous_s'))
        self.dev = None
        if not bool(p('dry_run')) and GPIO_OK:
            self.dev = DigitalOutputDevice(self.pin, active_high=True, initial_value=False)
        elif not bool(p('dry_run')):
            self.get_logger().warn('gpiozero недоступен — зуммер работает вхолостую')

        self.pattern = 'off'
        self.version = 0
        self.cv = threading.Condition()
        self.running = True
        threading.Thread(target=self.player, daemon=True).start()

        self.create_subscription(String, 'buzzer/pattern', self.on_pattern, 10)
        self.pub_state = self.create_publisher(String, 'buzzer/state', 5)
        self.create_timer(1.0, lambda: self.pub_state.publish(String(data=self.pattern)))
        self.get_logger().info(f'Зуммер: GPIO{self.pin}, {"включён" if self.enabled else "выключен"}')

    def on_pattern(self, msg: String):
        name = msg.data.strip().lower()
        if name not in PATTERNS:
            self.get_logger().warn(f'Неизвестный шаблон зуммера: {msg.data}')
            return
        with self.cv:
            if name == self.pattern and PATTERNS[name][1]:
                return
            self.pattern = name
            self.version += 1
            self.cv.notify_all()

    def out(self, on: bool):
        if self.dev is not None:
            self.dev.value = bool(on and self.enabled)

    def player(self):
        while self.running:
            with self.cv:
                while self.pattern == 'off' and self.running:
                    self.out(False)
                    self.cv.wait(0.5)
                name, ver = self.pattern, self.version
            seq, repeat = PATTERNS[name]
            interrupted = False
            while not interrupted:
                for i, dur in enumerate(seq):
                    on = (i % 2 == 0)
                    self.out(on)
                    end = time.monotonic() + (min(dur, self.max_on) if on else dur)
                    with self.cv:
                        while time.monotonic() < end and self.version == ver and self.running:
                            self.cv.wait(end - time.monotonic())
                        if self.version != ver or not self.running:
                            interrupted = True
                            break
                self.out(False)
                if not repeat:
                    break
            with self.cv:
                if not interrupted and self.version == ver:
                    self.pattern = 'off'

    def destroy_node(self):
        self.running = False
        with self.cv:
            self.cv.notify_all()
        self.out(False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BuzzerNode()
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
