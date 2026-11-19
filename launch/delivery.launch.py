"""
The world of the delivery mission: simulated robot, AMCL and Nav2 (day 8).

    /cmd_vel (Nav2) --> drive_sim --tf odom->base_footprint--> amcl --tf map->odom--> Nav2
                            |  |                               ^   ^                   ^
                            |  +--/battery_state--+            |   |                   |
                            +--/sim/true_pose--> laser_sim --/scan  |    goals, one per leaf
                                                 map_server --/map--+    "go to ..."
                                                                                       |
                                        delivery (py_trees, second terminal) ---------+

Nodes started here:
    drive_sim, laser_sim, robot_viz, base_scan_tf   the simulated robot with battery and dock
    map_server, amcl, lifecycle_manager_localization        the map of day 7 and AMCL
    controller_server, planner_server, behavior_server, bt_navigator,
    lifecycle_manager_navigation                            Nav2 (navigation.launch.py)
    rviz2                                                   with config/<world>.rviz

The behaviour tree runs in its own terminal, so its output is easy to read:
    ros2 launch delivery_bt delivery.launch.py                 # loop of day 7, slide 8
    ros2 run delivery_bt delivery                              # second terminal
or in the room of day 6:
    ros2 launch delivery_bt delivery.launch.py world:=room
    ros2 run delivery_bt delivery --ros-args -p world:=room

The robot starts on the dock, which is the origin of the map, so AMCL starts
there too. No 2D Pose Estimate needed.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('delivery_bt')
    world = LaunchConfiguration('world')

    def launch_file(name):
        return PythonLaunchDescriptionSource(os.path.join(share, 'launch', name))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='loop', choices=['loop', 'room'],
                              description='loop: corridor of day 7, room: room of day 6'),
        DeclareLaunchArgument('map',
                              default_value=PathJoinSubstitution(
                                  [share, 'maps', [world, '.yaml']]),
                              description='map yaml written by map_saver_cli'),
        DeclareLaunchArgument('battery', default_value='70.0',
                              description='battery level at the start in %'),
        DeclareLaunchArgument('blocked', default_value='false',
                              description='true: the block closes its corridor from the start'),
        DeclareLaunchArgument('rviz', default_value='true', description='start RViz'),

        # --- the simulated robot with battery and dock ---------------------------
        IncludeLaunchDescription(
            launch_file('sim.launch.py'),
            launch_arguments={'world': world, 'range_max': '3.5', 'noise_std': '0.02',
                              'battery': LaunchConfiguration('battery'),
                              'blocked': LaunchConfiguration('blocked')}.items()),

        # --- the map of day 7 and AMCL ---------------------------------------------
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[os.path.join(share, 'config', 'amcl.yaml')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]),

        # --- Nav2 ----------------------------------------------------------------
        IncludeLaunchDescription(launch_file('navigation.launch.py')),

        # --- RViz ----------------------------------------------------------------
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'config', [world, '.rviz']])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
