#!/usr/bin/env python3
"""Observation only: never publishes commands, TF, or navigation corrections."""
import copy
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from atsd_sensors.lidar_odometry import cloud, ScanOdometry, PoseHistory, SlipMonitor, wrap


def stamp_of(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class LidarOdomNode(Node):
    def __init__(self):
        super().__init__('lidar_odom_node')
        for name,value in [('laser_x',.29),('laser_y',0.),('laser_yaw_deg',0.),
                           ('laser_frame','laser_frame'),('budget_s',.08),('scan_timeout_s',.45)]:
            self.declare_parameter(name,value)
        p=lambda n:self.get_parameter(n).value
        self.laser=(float(p('laser_x')),float(p('laser_y')),math.radians(float(p('laser_yaw_deg'))))
        self.frame=p('laser_frame');self.timeout=float(p('scan_timeout_s'))
        self.engine=ScanOdometry(float(p('budget_s')))
        self.wheels=PoseHistory();self.imu=PoseHistory();self.slip=SlipMonitor()
        self.previous_stamp=None;self.previous_wheel=None;self.previous_yaw=None
        self.last_scan=time.monotonic()
        self.delta_pub=self.create_publisher(PoseWithCovarianceStamped,'lidar_odom/delta',1)
        self.status_pub=self.create_publisher(String,'lidar_odom/status',5)
        self.create_subscription(Odometry,'drive/odom',self.on_wheel,20)
        self.create_subscription(Imu,'imu/data',self.on_imu,20)
        scan_qos=copy.deepcopy(qos_profile_sensor_data);scan_qos.depth=1
        self.create_subscription(LaserScan,'scan',self.on_scan,scan_qos)
        self.create_timer(.5,self.watchdog)
        self.get_logger().info('Лидарная одометрия: только наблюдение, управление и позиция не изменяются')

    def on_wheel(self,msg):
        p=msg.pose.pose.position
        self.wheels.add(stamp_of(msg),(p.x,p.y,yaw_of(msg.pose.pose.orientation)))

    def on_imu(self,msg):
        if msg.orientation_covariance[0] != -1:
            self.imu.add(stamp_of(msg),(0.,0.,yaw_of(msg.orientation)))

    def status(self,**data):
        self.status_pub.publish(String(data=json.dumps({'mode':'observe',**data},allow_nan=False)))

    def reset(self):
        self.engine.reset();self.slip.reset()
        self.previous_stamp=self.previous_wheel=self.previous_yaw=None

    def watchdog(self):
        if time.monotonic()-self.last_scan>self.timeout:
            self.reset()
            self.status(valid=False,reason='stale_scan',slip_suspected=False)

    def on_scan(self,msg):
        self.last_scan=time.monotonic()
        stamp=stamp_of(msg)
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if msg.header.frame_id!=self.frame or not 0<=age<=self.timeout:
            self.reset();self.status(valid=False,reason='stale_or_wrong_frame',slip_suspected=False);return
        wheel=self.wheels.at(stamp);imu=self.imu.at(stamp)
        hint=wrap(imu[2]-self.previous_yaw) if imu is not None and self.previous_yaw is not None else 0.
        previous=self.previous_stamp
        points=cloud(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,self.laser)
        started=time.monotonic()
        result=self.engine.update(points,stamp,hint)
        elapsed=time.monotonic()-started
        # A slow computation must not label a delayed result as usable.
        if age+elapsed>self.timeout:
            result.valid=False;result.reason='stale_after_matching';self.engine.reset()
        wheel_delta=None;slipping=False
        if result.valid and wheel is not None and self.previous_wheel is not None:
            wheel_delta=(wheel[0]-self.previous_wheel[0],wheel[1]-self.previous_wheel[1],wrap(wheel[2]-self.previous_wheel[2]))
            slipping=self.slip.update(stamp,wheel_delta,result.pose)
        else:
            self.slip.reset()
        self.previous_stamp=stamp;self.previous_wheel=wheel
        self.previous_yaw=imu[2] if imu is not None else None
        if result.valid:
            out=PoseWithCovarianceStamped();out.header=copy.deepcopy(msg.header);out.header.frame_id='base_link'
            out.pose.pose.position.x,out.pose.pose.position.y=result.pose[:2]
            out.pose.pose.orientation.z=math.sin(result.pose[2]/2)
            out.pose.pose.orientation.w=math.cos(result.pose[2]/2)
            # Accuracy is not calibrated: do not advertise ICP residual as pose error.
            for axis in range(6):
                out.pose.covariance[axis*7]=1e6
            self.delta_pub.publish(out)
        self.status(valid=result.valid,reason=result.reason,stamp=stamp,reference_stamp=previous,
                    delta=list(result.pose) if result.valid else None,
                    wheel_distance=math.hypot(*wheel_delta[:2]) if wheel_delta is not None else None,
                    wheel_comparison=wheel_delta is not None,imu_hint=imu is not None,
                    rmse=result.rmse,overlap=result.overlap,geometry=result.geometry,
                    match_ms=round(elapsed*1000,1),slip_suspected=slipping)


def main(args=None):
    rclpy.init(args=args);node=LidarOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
