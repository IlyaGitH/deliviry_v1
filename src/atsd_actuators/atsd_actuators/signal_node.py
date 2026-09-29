#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from rcl_interfaces.msg import SetParametersResult

from atsd_actuators.signal_logic import Inputs, SignalConfig, SignalLogic


class SignalNode(Node):

    def __init__(self):
        super().__init__('signal_node')
        d = self.declare_parameter
        cfg = SignalConfig()
        for name in cfg.__dataclass_fields__:
            d(name, getattr(cfg, name))
        d('enabled', True)
        d('tick_hz', 5.0)
        p = lambda n: self.get_parameter(n).value
        for name in cfg.__dataclass_fields__:
            setattr(cfg, name, float(p(name)))
        self.enabled = bool(p('enabled'))
        self.add_on_set_parameters_callback(self.on_params)

        self.logic = SignalLogic(cfg)
        self.inp = Inputs()
        self.last_out = None
        self.last_buzz = None
        self.last_obst = 0.0

        self.pub_head = self.create_publisher(Float32, 'lights/headlights', 5)
        self.pub_strip = self.create_publisher(Float32, 'lights/strip', 5)
        self.pub_mode = self.create_publisher(String, 'lights/strip_mode', 5)
        self.pub_buzz = self.create_publisher(String, 'buzzer/pattern', 5)

        self.create_subscription(String, 'mission/state', self.on_state, 10)
        self.create_subscription(String, 'mission/event', self.on_event, 20)
        self.create_subscription(String, 'perception/obstacle', self.on_obstacle, 10)
        self.create_subscription(Bool, 'drive/link', self.on_link, 5)
        self.create_timer(1.0 / float(p('tick_hz')), self.tick)
        self.get_logger().info('Автоматика света и звука ' + ('включена' if self.enabled else 'выключена'))

    def on_params(self, params):
        for prm in params:
            if prm.name == 'enabled':
                self.enabled = bool(prm.value)
                self.last_out = None
                self.last_buzz = None
                self.get_logger().info('Автоматика света и звука ' + ('включена' if self.enabled else 'выключена'))
                if not self.enabled:
                    self.pub_head.publish(Float32(data=0.0))
                    self.pub_strip.publish(Float32(data=0.0))
                    self.pub_buzz.publish(String(data='off'))
        return SetParametersResult(successful=True)

    def on_state(self, msg: String):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.inp.state = s.get('state', 'UNKNOWN')
        self.inp.pause = s.get('pause', []) or []
        self.inp.flags = s.get('flags', []) or []

    def on_link(self, msg: Bool):
        self.inp.link = bool(msg.data)

    def on_obstacle(self, msg: String):
        try:
            self.inp.obstacle_m = float(msg.data.split()[0])
        except (ValueError, IndexError):
            self.inp.obstacle_m = 0.0
        self.last_obst = time.monotonic()

    def on_event(self, msg: String):
        if not self.enabled:
            return
        name = msg.data.split()[0] if msg.data else ''
        pattern = self.logic.on_event(name, time.monotonic())
        if pattern:
            self.pub_buzz.publish(String(data=pattern))
            self.last_buzz = None

    def tick(self):
        if not self.enabled:
            return
        now = time.monotonic()
        if now - self.last_obst > 1.0:
            self.inp.obstacle_m = 0.0
        out = self.logic.decide(self.inp, now)

        key = (round(out.head, 2), round(out.strip, 2), out.mode)
        if key != self.last_out:
            self.last_out = key
            self.pub_head.publish(Float32(data=float(out.head)))
            self.pub_strip.publish(Float32(data=float(out.strip)))
            self.pub_mode.publish(String(data=out.mode))

        if self.logic.buzzer_free(now) and out.buzzer != self.last_buzz:
            self.last_buzz = out.buzzer
            self.pub_buzz.publish(String(data=out.buzzer))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SignalNode()
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
