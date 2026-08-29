"""Полный запуск нижней подсистемы."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    share = get_package_share_directory('atsd_bringup')
    inc = lambda n: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', n)))
    return LaunchDescription([
        inc('drive.launch.py'),
        inc('actuators.launch.py'),
        inc('sensors.launch.py'),
    ])
