import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
NAV2_PKG = {'controller_server': 'nav2_controller', 'planner_server': 'nav2_planner',
            'behavior_server': 'nav2_behaviors', 'bt_navigator': 'nav2_bt_navigator'}


def generate_launch_description():
    share = get_package_share_directory('atsd_bringup')
    slam_share = get_package_share_directory('slam_toolbox')
    cfg = os.path.join(share, 'config', 'atsd.yaml')
    nav2 = os.path.join(share, 'config', 'nav2.yaml')
    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    mapping = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    driving = UnlessCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    slam_params = RewrittenYaml(source_file=os.path.join(share, 'config', 'slam.yaml'),
                                param_rewrites={'mode': mode, 'map_file_name': PythonExpression(
                                    ["'' if '", mode, "' == 'mapping' else '", map_file, "'"])},
                                convert_types=True)
    slam = lambda name, cond: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', name)),
        launch_arguments={'slam_params_file': slam_params, 'use_sim_time': 'false'}.items(),
        condition=cond)

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='localization',
                              description='localization — рейсы по карте, mapping — построить карту'),
        DeclareLaunchArgument('map', default_value='/opt/atsd/maps/map',
                              description='Карта slam_toolbox без расширения (.posegraph/.data)'),
        slam('online_async_launch.py', mapping),
        slam('localization_launch.py', driving),
        *[Node(package=NAV2_PKG[n], executable=n, name=n, parameters=[nav2], output='screen',
               remappings=[('cmd_vel', 'nav/cmd_vel')], condition=driving) for n in NAV2_NODES],
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', condition=driving,
             parameters=[{'autostart': True, 'node_names': NAV2_NODES}]),
        Node(package='atsd_drive', executable='nav_node', name='nav_node', parameters=[cfg],
             output='screen', respawn=True, respawn_delay=2.0, condition=driving),
    ])
