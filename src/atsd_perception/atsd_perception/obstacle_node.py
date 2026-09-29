#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from atsd_perception.obstacle_logic import CorridorConfig, Debounce, analyze, scan_to_xy


class ObstacleNode(Node):

    def __init__(self):
        super().__init__('obstacle_node')
        d = self.declare_parameter
        d('scan_topic', 'scan')
        d('laser_x', 0.29)
        d('laser_y', 0.0)
        d('laser_yaw_deg', 0.0)
        d('front_x', 0.35)
        d('half_width', 0.25)
        d('margin', 0.10)
        d('look_ahead', 2.5)
        d('detour_offset', 0.65)
        d('min_points', 3)
        d('default_side', 'L')
        d('on_count', 2)
        d('off_count', 3)
        d('report_offset', False)

        p = lambda n: self.get_parameter(n).value
        self.cfg = CorridorConfig(
            front_x=float(p('front_x')), half_width=float(p('half_width')),
            margin=float(p('margin')), look_ahead=float(p('look_ahead')),
            detour_offset=float(p('detour_offset')), min_points=int(p('min_points')),
            default_side=str(p('default_side')).upper()[:1])
        self.laser = (float(p('laser_x')), float(p('laser_y')), math.radians(float(p('laser_yaw_deg'))))
        self.report_offset = bool(p('report_offset'))
        self.deb = Debounce(int(p('on_count')), int(p('off_count')))
        self.last = (None, '?', 0.0)
        self.last_info = 0.0
        self.last_scan = time.monotonic()

        self.pub = self.create_publisher(String, 'perception/obstacle', 10)
        self.pub_info = self.create_publisher(String, 'perception/obstacle_info', 5)
        self.create_subscription(LaserScan, p('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.watchdog)
        self.get_logger().info(f'Коридор: ширина {2 * (self.cfg.half_width + self.cfg.margin):.2f} м, '
                               f'обзор {self.cfg.look_ahead} м, объезд {self.cfg.detour_offset} м')

    def on_scan(self, scan: LaserScan):
        self.last_scan = time.monotonic()
        pts = scan_to_xy(scan.ranges, scan.angle_min, scan.angle_increment,
                         scan.range_min, scan.range_max, *self.laser)
        dist, side, depth = analyze(pts, self.cfg)
        active = self.deb.update(dist is not None)
        if dist is not None:
            self.last = (dist, side, depth)

        if active and self.last[0] is not None:
            d_, s_, dep = self.last if dist is None else (dist, side, depth)
            off = self.cfg.detour_offset if self.report_offset else 0.0
            text = f'{d_:.2f} {s_} {dep:.2f} {off:.2f}'
        else:
            text = '0'
        self.pub.publish(String(data=text))

        now = time.monotonic()
        if now - self.last_info > 0.5:
            self.last_info = now
            self.pub_info.publish(String(
                data=f'препятствие {self.last[0]:.2f} м, объезд {self.last[1]}' if active and self.last[0]
                else 'коридор свободен'))

    def watchdog(self):
        if time.monotonic() - self.last_scan > 1.5:
            self.get_logger().error('Нет /scan больше 1,5 с — лидар молчит', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleNode()
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
