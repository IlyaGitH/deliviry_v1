"""Лидар LD19, камера, GNSS, гейдж питания плюс статические трансформы."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('atsd_bringup'), 'config', 'atsd.yaml')

    use_lidar  = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')
    use_gnss   = LaunchConfiguration('use_gnss')

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar',  default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_gnss',   default_value='true'),

        Node(package='atsd_sensors', executable='lidar_node',
             name='lidar_node', parameters=[cfg],
             output='screen', respawn=True, respawn_delay=3.0,
             condition=IfCondition(use_lidar)),

        Node(package='v4l2_camera', executable='v4l2_camera_node',
             name='v4l2_camera', parameters=[cfg],
             output='screen', respawn=True, respawn_delay=3.0,
             remappings=[('/image_raw', '/camera/image_raw'),
                         ('/camera_info', '/camera/camera_info')],
             condition=IfCondition(use_camera)),

        Node(package='atsd_sensors', executable='battery_node',
             name='battery_node', parameters=[cfg],
             output='screen', respawn=True, respawn_delay=5.0),

        Node(package='atsd_sensors', executable='gnss_node',
             name='gnss_node', parameters=[cfg],
             output='screen', respawn=True, respawn_delay=3.0,
             condition=IfCondition(use_gnss)),

        # ── статические трансформы: x y z yaw pitch roll parent child ──
        # Значения по компоновочному чертежу АТСД-1М, в метрах.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_base_laser',
             arguments=['0.29', '0.0', '0.22', '0', '0', '0',
                        'base_link', 'laser_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_base_camera',
             arguments=['0.28', '0.0', '0.385', '0', '0.17', '0',
                        'base_link', 'camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_base_gnss',
             arguments=['0.11', '0.0', '0.36', '0', '0', '0',
                        'base_link', 'gnss_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_footprint',
             arguments=['0', '0', '0', '0', '0', '0',
                        'base_footprint', 'base_link']),
    ])
