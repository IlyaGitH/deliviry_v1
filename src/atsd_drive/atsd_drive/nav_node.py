#!/usr/bin/env python3
# Ведёт рейс по дороге: разметка (камера) держит в полосе, GPS и колёса говорят, где робот и куда
# поворачивать. Состояние миссии с Brain -> маршрут по графу дорог -> скорость G -> «приехал» обратно.
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, TwistStamped, \
    TwistWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster

from atsd_drive.nav_logic import find_point, want_leg
from atsd_drive.local_planner import LocalCfg, LocalPlanner, ScanSpace, Decision, relative_pose
from atsd_drive.road_logic import FollowCfg, RoadNav, Roads, to_local, wrap


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_json(path, fallback):
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return fallback


class NavNode(Node):

    def __init__(self):
        super().__init__('nav_node')
        d = self.declare_parameter
        d('points_file', '/opt/atsd/data/points.json')
        d('roads_file', '/opt/atsd/data/roads.json')
        d('map_frame', 'map')
        d('odom_frame', 'odom')
        d('v_max', 0.5)
        d('arrive_m', 0.3)
        d('turn_radius_m', 0.5)
        d('turn_speed_m_s', 0.25)
        d('local_allow_reverse', True)
        d('robot_front_m', 0.35)
        d('robot_rear_m', 0.40)
        d('robot_half_width_m', 0.25)
        d('obstacle_margin_m', 0.12)
        d('avoid_speed_m_s', 0.25)
        d('reverse_speed_m_s', 0.15)
        d('reverse_limit_m', 5.0)
        d('short_reverse_m', 5.0)
        d('blocked_timeout_s', 30.0)
        d('laser_x', 0.29)
        d('laser_y', 0.0)
        d('laser_yaw_deg', 0.0)
        d('laser_frame', 'laser_frame')
        d('lane_timeout_s', 0.5)
        d('gnss_timeout_s', 2.0)
        d('no_position_timeout_s', 20.0)
        p = lambda n: self.get_parameter(n).value
        self.points_file, self.roads_file = p('points_file'), p('roads_file')
        self.map_frame, self.odom_frame = p('map_frame'), p('odom_frame')
        self.lane_timeout, self.gnss_timeout = float(p('lane_timeout_s')), float(p('gnss_timeout_s'))
        self.no_pos_timeout = float(p('no_position_timeout_s'))

        self.roads_mtime, self.origin = None, None
        self.nav = RoadNav(Roads(), FollowCfg(v_max=float(p('v_max')), arrive_m=float(p('arrive_m')),
            turn_radius=max(0.3, float(p('turn_radius_m'))),
            turn_speed=max(0.05, float(p('turn_speed_m_s')))))
        self.local = LocalPlanner(LocalCfg(
            front=float(p('robot_front_m')), rear=float(p('robot_rear_m')),
            half_width=float(p('robot_half_width_m')), margin=float(p('obstacle_margin_m')),
            short_reverse_m=float(p('short_reverse_m')),
            radius=float(p('turn_radius_m')), allow_reverse=bool(p('local_allow_reverse')),
            cruise_speed=float(p('v_max')),
            forward_speed=float(p('avoid_speed_m_s')), reverse_speed=float(p('reverse_speed_m_s')),
            max_reverse_m=float(p('reverse_limit_m'))))
        self.laser = (float(p('laser_x')), float(p('laser_y')), math.radians(float(p('laser_yaw_deg'))))
        self.laser_frame = p('laser_frame')
        self.space, self.scan_pose, self.scan_t = None, None, 0.0
        self.odom_t, self.v_odom = 0.0, 0.0
        self.avoidance = Decision()
        self.blocked_since = None
        self.blocked_timeout = float(p('blocked_timeout_s'))
        self.load_roads()

        self.pub_cmd = self.create_publisher(TwistStamped, 'nav/checked_cmd_vel', 1)
        self.pub_result = self.create_publisher(String, 'nav/result', 10)
        self.pub_remain = self.create_publisher(Float32, 'nav/remain', 10)
        self.pub_event = self.create_publisher(String, 'mission/event', 20)
        self.pub_plan = self.create_publisher(Path, 'plan', 1)
        self.pub_status = self.create_publisher(String, 'road/status', 5)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(String, 'mission/state', self.on_state, 10)
        self.create_subscription(Odometry, 'drive/odom', self.on_odom, 20)
        self.create_subscription(NavSatFix, 'gnss/fix', self.on_fix, 10)
        self.create_subscription(TwistWithCovarianceStamped, 'gnss/vel', self.on_vel, 10)
        self.create_subscription(String, 'perception/lane', self.on_lane, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_initialpose, 5)
        self.create_subscription(NavSatFix, 'wifi/fix', self.on_wifi, 5)
        self.create_subscription(Imu, 'imu/data', self.on_imu, 50)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_status)

        self.leg = None       # плечо рейса, по которому едем или ждём позицию
        self.leg_t = 0.0
        self.result = None    # (плечо, ok): повторяем Brain, пока он не сменит состояние
        self.odom = None      # (x, y, курс) в одометрии Brain
        self.w_odom = 0.0
        self.lane, self.lane_t = None, 0.0
        self.fix_t = 0.0
        self.fix_xy, self.fix_sigma = None, None
        self.wifi, self.wifi_t = None, 0.0  # (x, y, точность) по Wi-Fi, свежая 2 мин
        self.imu_yaw, self.imu_rate, self.imu_t, self.imu_prev = None, 0.0, 0.0, None  # курс гироскопа
        self.last_plan_pub = 0.0
        self.get_logger().info(f'Езда по дороге: граф {self.roads_file}, точки {self.points_file}')

    # ---------- данные ----------
    def load_roads(self):
        try:
            mtime = os.path.getmtime(self.roads_file)
        except OSError:
            if self.roads_mtime is not False:
                self.get_logger().warn(f'Нет {self.roads_file}: '
                                       'нужны дороги на карте; рейс не будет запущен')
            self.invalidate_road_plan()
            self.roads_mtime = False
            self.nav.roads = Roads()
            return
        if mtime == self.roads_mtime:
            return
        data = load_json(self.roads_file, {})
        if not isinstance(data, dict): data = {}
        self.invalidate_road_plan()
        self.roads_mtime = mtime
        try:
            self.nav.roads = Roads.from_data(data)
        except (ValueError, TypeError, KeyError, IndexError):
            self.nav.roads = Roads()
            self.get_logger().error('Некорректный граф дорог — движение по маршруту запрещено')
        self.origin = tuple(data['origin']) if data.get('origin') else None
        self.get_logger().info(f'Граф дорог: {len(self.nav.roads.nodes)} узлов, origin {self.origin}')

    def invalidate_road_plan(self):
        if self.nav.active:
            self.nav.stop()
            self.publish_command()
            self.reset_local()
            self.publish_plan()

    def on_imu(self, msg: Imu):
        self.imu_yaw, self.imu_rate = yaw_of(msg.orientation), msg.angular_velocity.z
        self.imu_t = time.monotonic()

    def wifi_fresh(self):
        # не по одному времени: часы monotonic идут от включения Pi, первые 2 минуты «свежим» был бы None
        return self.wifi if self.wifi is not None and time.monotonic() - self.wifi_t < 120.0 else None

    def imu_fresh(self):
        return self.imu_yaw is not None and time.monotonic() - self.imu_t < 0.3

    def on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        cur = (pos.x, pos.y, yaw_of(msg.pose.pose.orientation))
        self.w_odom = msg.twist.twist.angular.z
        self.v_odom, self.odom_t = msg.twist.twist.linear.x, time.monotonic()
        imu = self.imu_yaw if self.imu_fresh() else None
        if self.odom is not None:
            dx, dy = cur[0] - self.odom[0], cur[1] - self.odom[1]
            ds = dx * math.cos(self.odom[2]) + dy * math.sin(self.odom[2])
            # путь — колёса, поворот — гироскоп, если он есть: колёса в поворотах проскальзывают
            if imu is not None and self.imu_prev is not None:
                dyaw, self.nav.filter.yaw_err = wrap(imu - self.imu_prev), 0.02
            else:
                dyaw, self.nav.filter.yaw_err = wrap(cur[2] - self.odom[2]), 0.05
            self.nav.filter.predict(ds, dyaw)
        self.odom, self.imu_prev = cur, imu

    def on_scan(self, msg: LaserScan):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        # Reject delayed/wrong-frame scans rather than treating receipt as freshness.
        if (self.odom is None or now-self.odom_t > self.local.cfg.odom_timeout or
                msg.header.frame_id != self.laser_frame or not 0 <= age <= self.local.cfg.scan_timeout):
            return
        self.space = ScanSpace(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max, self.local.cfg, self.laser)
        self.scan_pose, self.scan_t = tuple(self.odom), now-age

    def reset_local(self):
        self.local.reset()
        self.blocked_since = None
        self.avoidance = Decision()

    def on_fix(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX or self.origin is None:
            return
        x, y = to_local(msg.latitude, msg.longitude, self.origin)
        sigma = math.sqrt(msg.position_covariance[0]) if msg.position_covariance[0] > 0 else 2.5
        before = self.nav.filter.gnss
        wifi = self.wifi_fresh()
        state = self.nav.filter.gnss_fix(x, y, max(sigma, 1.0), wifi)
        self.fix_t = time.monotonic()
        self.fix_xy, self.fix_sigma = (round(x, 2), round(y, 2)), round(sigma, 1)
        if state != before and state == 'suspect':
            self.event('GNSS_SUSPECT')
            self.get_logger().warn('GPS прыгнул или не сходится с колёсами — не верю ему, еду по колёсам и разметке')
        elif state != before and state == 'ok' and before == 'suspect':
            self.event('GNSS_OK')

    def on_wifi(self, msg: NavSatFix):
        if self.origin is None:
            return
        x, y = to_local(msg.latitude, msg.longitude, self.origin)
        self.wifi = (x, y, math.sqrt(max(msg.position_covariance[0], 1.0)))
        self.wifi_t = time.monotonic()
        was_ready = self.nav.filter.ready
        if self.nav.filter.wifi_fix(*self.wifi) and not was_ready:
            self.get_logger().info(f'Место по Wi-Fi: {x:.0f}, {y:.0f} ±{self.wifi[2]:.0f} м — '
                                   'точнее даст GPS или «Робот здесь»')

    def on_vel(self, msg: TwistWithCovarianceStamped):
        ve, vn = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        w = self.imu_rate if self.imu_fresh() else self.w_odom
        self.nav.course(math.atan2(vn, ve), math.hypot(ve, vn), abs(w) > 0.2)

    def on_lane(self, msg: String):
        parts = msg.data.split()
        if len(parts) >= 4 and parts[0] != 'none':
            self.lane = {'offset': float(parts[0]), 'angle': float(parts[1]),
                         'width': float(parts[2]), 'lines': int(parts[3])}
            self.lane_t = time.monotonic()

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        pos = msg.pose.pose.position
        self.publish_command()
        self.nav.filter.set_manual(pos.x, pos.y, yaw_of(msg.pose.pose.orientation))
        self.reset_local()
        self.space, self.scan_pose = None, None
        self.odom = None  # не применять дельту, накопленную до установки позиции
        self.imu_prev = None
        self.result = None
        self.nav.stop()
        if self.leg:
            self.try_plan()
        self.publish_status()
        self.get_logger().info(f'Поза задана вручную: {pos.x:.1f}, {pos.y:.1f}')

    # ---------- рейс ----------
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
        self.nav.stop()
        self.publish_command()
        self.reset_local()
        self.leg, self.leg_t = leg, time.monotonic()
        if leg:
            self.try_plan()

    def goal_of(self, pid):
        pt = find_point(load_json(self.points_file, []), pid)
        if pt:
            return (float(pt['x']), float(pt['y']))
        return self.nav.roads.nodes.get(pid)

    def try_plan(self):
        self.load_roads()
        goal = self.goal_of(self.leg[1])
        if goal is None:
            self.get_logger().error(f'Нет координат точки «{self.leg[1]}»')
            return self.finish(False, 'no_point')
        if not self.nav.filter.ready:
            return  # ждём первый фикс GPS или «Робот здесь»
        path = self.nav.plan(goal)
        if not path:
            return self.finish(False, 'no_route')
        self.get_logger().info(f'{self.leg[0]}: еду в «{self.leg[1]}», {len(path)} точек маршрута')
        self.publish_plan()

    def tick(self):
        now = time.monotonic()
        if now - self.fix_t > self.gnss_timeout:
            self.nav.filter.gnss_lost()
        self.publish_tf()
        if not self.leg:
            return
        if not self.nav.active:
            if self.nav.filter.ready:
                self.try_plan()
            elif now - self.leg_t > self.no_pos_timeout:
                self.finish(False, 'no_position')
            return
        lane = self.lane if now - self.lane_t < self.lane_timeout else None
        # During a detour lane/map corrections would pull the estimated robot
        # sideways back onto the road. Odometry/IMU continue normally.
        detouring = self.local.mode == 'avoid'
        # A single edge gives no measured lane width; keep it diagnostic only.
        trusted_lane = lane is not None and lane.get('lines', 0) >= 2
        v, w, done = self.nav.tick(lane if trusted_lane else None,
                                  correct_lane=not detouring and trusted_lane, strict_arrival=True)
        if done:
            self.finish(True)
            return
        decision = Decision(state='waiting_scan')
        if self.space is not None and self.scan_pose is not None and self.odom is not None:
            current = relative_pose(self.odom, self.scan_pose)
            # Convert the road goal through CURRENT world pose into the odometry
            # scan frame. GPS/manual map corrections cannot move old lidar cells.
            goal_world = self.nav.follower._lookahead(self.nav.filter.X[:2], 4.5)
            goal_body = relative_pose((*goal_world, 0.), self.nav.filter.X)
            c, sn = math.cos(current[2]), math.sin(current[2])
            goal_scan = (current[0]+c*goal_body[0]-sn*goal_body[1],
                         current[1]+sn*goal_body[0]+c*goal_body[1])
            # Recovery must be allowed to steer TOWARDS the road before the
            # robot is already perfectly aligned with it.
            final_near = (self.nav.follower.i == len(self.nav.follower.path)-2 and
                          math.dist(self.nav.filter.X[:2], self.nav.follower.path[-1])
                          <= self.local.cfg.short_reverse_m)
            decision = self.local.decide(self.space, current, goal_scan, now, now-self.scan_t,
                                         self.v_odom, self.w_odom, now-self.odom_t, self.odom,
                                         preferred=(v, w), short_reverse_allowed=final_near)
        # Planning consumes time too: do not freshly stamp an expired observation.
        checked_now = time.monotonic()
        if decision.v and (checked_now-self.scan_t > self.local.cfg.scan_timeout or
                           checked_now-self.odom_t > self.local.cfg.odom_timeout):
            decision = Decision(state='stale_after_planning')
            self.local.command = (0., 0.)
        self.avoidance = decision
        moving = abs(decision.v) > 0.01
        if moving:
            self.blocked_since = None
        elif self.blocked_since is None:
            self.blocked_since = now
        if decision.state in ('stalled', 'escape_timeout') or (self.blocked_since is not None and
                                         now-self.blocked_since > self.blocked_timeout):
            self.finish(False, decision.state)
            return
        v, w = decision.v, decision.w
        self.publish_command(v, w)
        self.pub_remain.publish(Float32(data=float(self.nav.follower.remain(self.nav.filter.X[:2]))))
        if now - self.last_plan_pub > 2.0:
            self.publish_plan()

    def publish_command(self, v=0., w=0.):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x, cmd.twist.angular.z = float(v), float(w)
        self.pub_cmd.publish(cmd)

    def finish(self, ok, why=''):
        leg = self.leg
        self.nav.stop()
        self.publish_command()
        self.leg = None
        self.result = (leg, ok)
        self.publish_result()
        if not ok:
            self.event(f'NAV_FAILED {leg[1]} {why}')
            self.get_logger().error(f'Не доехать до «{leg[1]}»: {why}')

    def publish_result(self):
        leg, ok = self.result
        self.pub_result.publish(String(data=f'{"ok" if ok else "fail"} {leg[0]}'))

    def event(self, text):
        self.pub_event.publish(String(data=text))

    # ---------- для веба и отладки ----------
    def publish_tf(self):
        f = self.nav.filter
        if not f.ready or self.odom is None:
            return
        # map -> odom так, чтобы map -> base_link была оценкой фильтра
        yaw = f.X[2] - self.odom[2]
        c, s = math.cos(yaw), math.sin(yaw)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = self.map_frame, self.odom_frame
        t.transform.translation.x = f.X[0] - (c * self.odom[0] - s * self.odom[1])
        t.transform.translation.y = f.X[1] - (s * self.odom[0] + c * self.odom[1])
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(t)

    def publish_plan(self):
        self.last_plan_pub = time.monotonic()
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.nav.follower.path[self.nav.follower.i:] if self.nav.active else []:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            msg.poses.append(ps)
        self.pub_plan.publish(msg)

    def publish_status(self):
        self.load_roads()  # граф или origin могли поменять (road_mark, веб) — подхватываем без перезапуска
        f = self.nav.filter
        lane_on = time.monotonic() - self.lane_t < self.lane_timeout
        self.pub_status.publish(String(data=json.dumps({
            'gnss': f.gnss, 'mode': self.nav.follower.mode if self.nav.active else 'idle',
            'avoidance': self.avoidance.state, 'reverse_m': round(self.local.reverse_distance, 2),
            'lane': self.lane['lines'] if lane_on and self.lane else 0,
            'manual_pose': f.manual, 'yaw_known': bool(f.yaw_known), 'ready': bool(f.ready), 'imu': self.imu_fresh(),
            # для отладочной страницы: оценка и сырой GPS в метрах карты
            'pose': [round(float(v), 3) for v in f.X] if f.ready else None,
            'sigma_xy': round(math.sqrt(max(f.P[0, 0], f.P[1, 1])), 1) if f.ready else None,
            'fix': self.fix_xy if time.monotonic() - self.fix_t < self.gnss_timeout else None,
            'sigma': self.fix_sigma,
            'wifi': [round(v, 1) for v in w] if (w := self.wifi_fresh()) else None})))


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
