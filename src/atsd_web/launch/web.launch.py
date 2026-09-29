from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    port = LaunchConfiguration('port')
    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false',
                              description='Телеметрия из имитатора, без робота'),
        DeclareLaunchArgument('port', default_value='8080'),
        Node(package='atsd_web', executable='web_node', name='web_node',
             parameters=[{'mock': mock, 'port': port}],
             output='screen', respawn=True, respawn_delay=3.0),
    ])
