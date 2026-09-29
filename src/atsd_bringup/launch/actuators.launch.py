import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('atsd_bringup'), 'config', 'atsd.yaml')
    use_signal = LaunchConfiguration('use_signal')
    node = lambda exe, **kw: Node(package='atsd_actuators', executable=exe, name=exe,
                                  parameters=[cfg], output='screen',
                                  respawn=True, respawn_delay=2.0, **kw)
    return LaunchDescription([
        DeclareLaunchArgument('use_signal', default_value='true',
                              description='Автоматика света и звука по состоянию рейса'),
        node('light_node'),
        node('lock_node'),
        node('buzzer_node'),
        node('signal_node', condition=IfCondition(use_signal)),
    ])
