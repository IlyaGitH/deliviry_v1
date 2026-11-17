#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from atsd_sensors.rear_guard import RearGuard


class RearGuardNode(Node):
    def __init__(self):
        super().__init__('rear_guard_node')
        self.declare_parameter('rear_m', .55)
        self.declare_parameter('half_width_m', .20)
        self.declare_parameter('confirm_s', 1.)
        self.guard = RearGuard(self.get_parameter('rear_m').value,
                               self.get_parameter('half_width_m').value,
                               self.get_parameter('confirm_s').value)
        self.odom_t = self.scan_t = 0.
        self.moving = False
        self.pub_event = self.create_publisher(String, 'mission/event', 10)
        self.pub_alarm = self.create_publisher(String, 'security/rear_intrusion', 10)
        self.pub_status = self.create_publisher(String, 'security/rear_status', 5)
        self.create_subscription(Odometry, 'drive/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(.2, self.watchdog)

    def on_odom(self, msg):
        self.odom_t = time.monotonic()
        self.moving = abs(msg.twist.twist.linear.x) > .03 or abs(msg.twist.twist.angular.z) > .05

    def on_scan(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        if msg.header.frame_id != 'laser_frame' or not 0 <= age <= .45 or now-self.odom_t > .35:
            self.guard.reset()
            return
        self.scan_t = now
        if self.guard.update(msg.ranges, msg.angle_min, msg.angle_increment,
                             msg.range_min, msg.range_max, now-age, self.moving):
            text = 'SECURITY_REAR_INTRUSION'
            self.pub_event.publish(String(data=text))
            self.pub_alarm.publish(String(data=text))
            self.get_logger().warn('Объект в заднем габарите более секунды; только оповещение')

    def watchdog(self):
        now = time.monotonic()
        if now-self.scan_t > .45 or now-self.odom_t > .35:
            self.guard.reset()
            status = 'unavailable'
        else:
            status = 'alert' if self.guard.latched else ('watching' if self.moving else 'idle')
        self.pub_status.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = RearGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
