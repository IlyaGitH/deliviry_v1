import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Езда по дороге: разметка + GPS + колёса. Лидар — только препятствия (obstacle_node -> Brain)
    cfg = os.path.join(get_package_share_directory('atsd_bringup'), 'config', 'atsd.yaml')
    return LaunchDescription([
        Node(package='atsd_drive', executable='nav_node', name='nav_node', parameters=[cfg],
             output='screen', respawn=True, respawn_delay=2.0),
    ])
