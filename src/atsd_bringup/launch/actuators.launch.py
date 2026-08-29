"""Свет, лента, замок."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('atsd_bringup'), 'config', 'atsd.yaml')
    return LaunchDescription([
        Node(package='atsd_actuators', executable='light_node',
             name='light_node', parameters=[cfg],
             output='screen', respawn=True, respawn_delay=2.0),
        Node(package='atsd_actuators', executable='lock_node',
             name='lock_node', parameters=[cfg],
             output='screen', respawn=True, respawn_delay=2.0),
    ])
