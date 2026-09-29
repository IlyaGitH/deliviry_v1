import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('atsd_bringup'), 'config', 'atsd.yaml')
    use_vision = LaunchConfiguration('use_vision')
    return LaunchDescription([
        DeclareLaunchArgument('use_vision', default_value='true'),
        Node(package='atsd_perception', executable='obstacle_node', name='obstacle_node',
             parameters=[cfg], output='screen', respawn=True, respawn_delay=2.0),
        Node(package='atsd_perception', executable='vision_node', name='vision_node',
             parameters=[cfg], output='screen', respawn=True, respawn_delay=5.0,
             condition=IfCondition(use_vision)),
    ])
