#!/usr/bin/env python3
"""
atsd_actuators · light_node

Управление фарами и лентой через полевые ключи на GPIO.

Подписывается:
    /lights/headlights   std_msgs/Float32   яркость фар 0.0 .. 1.0
    /lights/strip        std_msgs/Float32   яркость ленты 0.0 .. 1.0
    /lights/strip_mode   std_msgs/String    off | solid | blink | beacon
    /lock/busy           std_msgs/Bool      интерлок: замок под током

Публикует:
    /lights/state        std_msgs/String    текущее состояние, для интерфейса

ТОКОВЫЙ ИНТЕРЛОК
Шина 12 В тянет около 1,4 А. Замок один берёт 1,1 А, поэтому пока
он под током, свет принудительно гасится. Иначе просадка уронит
Raspberry посреди заезда.

ПЛАВНЫЙ ПУСК
Резкое включение даёт бросок тока через повышающий модуль.
Яркость меняется рампой за ramp_time, а не скачком.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool

try:
    from gpiozero import PWMLED
    GPIO_OK = True
except Exception:                                  # noqa: BLE001
    GPIO_OK = False


MODES = ('off', 'solid', 'blink', 'beacon')


class Channel:
    """Один ШИМ-канал с рампой и ограничением сверху."""

    def __init__(self, pin, ceiling, ramp_time, freq, dry_run):
        self.ceiling = ceiling
        self.ramp_step = 1.0 / max(ramp_time, 0.001)
        self.target = 0.0
        self.current = 0.0
        self.dev = None
        if not dry_run and GPIO_OK:
            self.dev = PWMLED(pin, frequency=freq)

    def set(self, value):
        self.target = max(0.0, min(float(value), self.ceiling))

    def tick(self, dt):
        step = self.ramp_step * dt
        if self.current < self.target:
            self.current = min(self.current + step, self.target)
        elif self.current > self.target:
            self.current = max(self.current - step, self.target)
        if self.dev is not None:
            self.dev.value = self.current

    def off_now(self):
        self.target = 0.0
        self.current = 0.0
        if self.dev is not None:
            self.dev.value = 0.0


class LightNode(Node):

    def __init__(self):
        super().__init__('light_node')

        self.declare_parameter('headlight_pin', 18)
        self.declare_parameter('strip_pin', 13)
        self.declare_parameter('pwm_freq_hz', 200)
        self.declare_parameter('headlight_ceiling', 0.6)   # бюджет питания
        self.declare_parameter('strip_ceiling', 0.5)
        self.declare_parameter('ramp_time_s', 0.5)
        self.declare_parameter('blink_period_s', 1.0)
        self.declare_parameter('beacon_period_s', 0.35)
        self.declare_parameter('dim_on_lock', True)
        self.declare_parameter('tick_hz', 50.0)
        self.declare_parameter('dry_run', False)

        p = self.get_parameter
        self.dry_run = bool(p('dry_run').value)
        if not GPIO_OK and not self.dry_run:
            self.get_logger().warn(
                'gpiozero недоступен — узел работает вхолостую. '
                'Поставьте python3-gpiozero и python3-lgpio.')

        freq = int(p('pwm_freq_hz').value)
        ramp = float(p('ramp_time_s').value)

        self.head = Channel(int(p('headlight_pin').value),
                            float(p('headlight_ceiling').value),
                            ramp, freq, self.dry_run)
        self.strip = Channel(int(p('strip_pin').value),
                             float(p('strip_ceiling').value),
                             ramp, freq, self.dry_run)

        self.dim_on_lock = bool(p('dim_on_lock').value)
        self.blink_period = float(p('blink_period_s').value)
        self.beacon_period = float(p('beacon_period_s').value)

        self.mode = 'off'
        self.strip_level = 0.0
        self.head_level = 0.0
        self.lock_busy = False
        self.phase = 0.0

        self.create_subscription(Float32, 'lights/headlights', self.on_head, 10)
        self.create_subscription(Float32, 'lights/strip', self.on_strip, 10)
        self.create_subscription(String, 'lights/strip_mode', self.on_mode, 10)
        self.create_subscription(Bool, 'lock/busy', self.on_lock, 10)
        self.pub_state = self.create_publisher(String, 'lights/state', 5)

        self.dt = 1.0 / float(p('tick_hz').value)
        self.create_timer(self.dt, self.tick)
        self.create_timer(1.0, self.publish_state)

        self.get_logger().info(
            f'Свет: фары GPIO{p("headlight_pin").value}, '
            f'лента GPIO{p("strip_pin").value}, потолки '
            f'{self.head.ceiling:.2f} / {self.strip.ceiling:.2f}')

    # ────────────────────────────────────────── подписки
    def on_head(self, msg: Float32):
        self.head_level = max(0.0, min(msg.data, 1.0))

    def on_strip(self, msg: Float32):
        self.strip_level = max(0.0, min(msg.data, 1.0))

    def on_mode(self, msg: String):
        mode = msg.data.strip().lower()
        if mode in MODES:
            self.mode = mode
            self.phase = 0.0
        else:
            self.get_logger().warn(f'Неизвестный режим ленты: {msg.data}')

    def on_lock(self, msg: Bool):
        self.lock_busy = bool(msg.data)

    # ────────────────────────────────────────── такт
    def tick(self):
        self.phase += self.dt

        if self.dim_on_lock and self.lock_busy:
            self.head.set(0.0)
            self.strip.set(0.0)
        else:
            self.head.set(self.head_level)
            self.strip.set(self.strip_value())

        self.head.tick(self.dt)
        self.strip.tick(self.dt)

    def strip_value(self):
        if self.mode == 'off':
            return 0.0
        if self.mode == 'solid':
            return self.strip_level
        if self.mode == 'blink':
            on = (self.phase % self.blink_period) < (self.blink_period * 0.5)
            return self.strip_level if on else 0.0
        if self.mode == 'beacon':
            # короткая яркая вспышка, как у проблескового маячка
            on = (self.phase % self.beacon_period) < (self.beacon_period * 0.25)
            return self.strip_level if on else 0.0
        return 0.0

    def publish_state(self):
        self.pub_state.publish(String(
            data=f'head={self.head.current:.2f} '
                 f'strip={self.strip.current:.2f} '
                 f'mode={self.mode} lock_busy={int(self.lock_busy)}'))

    def destroy_node(self):
        self.head.off_now()
        self.strip.off_now()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LightNode()
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
