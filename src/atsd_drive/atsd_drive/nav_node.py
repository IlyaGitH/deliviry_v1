#!/usr/bin/env python3
# Ведёт рейс по лидарной карте: состояние миссии с Brain -> цель Nav2 -> «приехал» или «не доехал» обратно.
import json
import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32, String

from atsd_drive.nav_logic import find_point, want_leg


class NavNode(Node):

    def __init__(self):
        super().__init__('nav_node')
        self.declare_parameter('points_file', '/opt/atsd/data/points.json')
        self.declare_parameter('map_frame', 'map')
        self.points_file = self.get_parameter('points_file').value
        self.map_frame = self.get_parameter('map_frame').value

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub_result = self.create_publisher(String, 'nav/result', 10)
        self.pub_remain = self.create_publisher(Float32, 'nav/remain', 10)
        self.pub_event = self.create_publisher(String, 'mission/event', 20)
        self.create_subscription(String, 'mission/state', self.on_state, 10)

        self.leg = None      # плечо, на которое отправлена цель
        self.handle = None
        self.result = None   # (плечо, ok): повторяем Brain, пока он не сменит состояние
        self.get_logger().info(f'Навигация по карте, точки из {self.points_file}')

    def on_state(self, msg: String):
        try:
            leg = want_leg(json.loads(msg.data))
        except (ValueError, AttributeError):
            return
        if self.result and self.result[0] == leg:
            self.publish_result()
            return
        self.result = None
        if leg == self.leg:
            return
        self.cancel()
        if leg:
            self.send_goal(leg)

    def cancel(self):
        if self.handle is not None:
            self.handle.cancel_goal_async()
        self.handle = None
        self.leg = None

    def load_points(self):
        try:
            with open(self.points_file, encoding='utf-8') as f:
                return json.load(f)
        except (OSError, ValueError) as e:
            self.get_logger().error(f'Не прочитать {self.points_file}: {e}', throttle_duration_sec=5.0)
            return []

    def send_goal(self, leg):
        if not self.client.server_is_ready():
            self.get_logger().warn('Nav2 ещё не поднялся, жду', throttle_duration_sec=5.0)
            return
        pt = find_point(self.load_points(), leg[1])
        if pt is None:
            self.get_logger().error(f'Нет координат точки «{leg[1]}»')
            self.finish(leg, False)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pt['x'])
        goal.pose.pose.position.y = float(pt['y'])
        yaw = math.radians(float(pt.get('yaw', 0.0)))
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.leg = leg
        self.get_logger().info(f'{leg[0]}: еду в «{leg[1]}» ({pt["x"]:.2f}, {pt["y"]:.2f})')
        fut = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        fut.add_done_callback(lambda f: self.on_accepted(leg, f))

    def on_accepted(self, leg, fut):
        handle = fut.result()
        if leg != self.leg:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.finish(leg, False)
            return
        self.handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(leg, handle, f))

    def on_feedback(self, fb):
        self.pub_remain.publish(Float32(data=float(fb.feedback.distance_remaining)))

    def on_result(self, leg, handle, fut):
        if handle is not self.handle:
            return
        self.handle = None
        status = fut.result().status
        if status == GoalStatus.STATUS_CANCELED:
            self.leg = None
            return
        self.finish(leg, status == GoalStatus.STATUS_SUCCEEDED)

    def finish(self, leg, ok):
        self.leg = None
        self.result = (leg, ok)
        self.publish_result()
        if not ok:
            self.pub_event.publish(String(data=f'NAV_FAILED {leg[1]}'))
            self.get_logger().error(f'Nav2 не довёл до «{leg[1]}»')

    def publish_result(self):
        leg, ok = self.result
        self.pub_result.publish(String(data=f'{"ok" if ok else "fail"} {leg[0]}'))


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
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
