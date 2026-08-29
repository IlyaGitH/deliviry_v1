"""Мост с V5: одометрия, телеметрия, cmd_vel."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('atsd_bringup'), 'config', 'atsd.yaml')
    return LaunchDescription([
        Node(package='atsd_drive', executable='v5_bridge_node',
             name='v5_bridge', parameters=[cfg],
             output='screen', respawn=True, respawn_delay=2.0),
    ])
