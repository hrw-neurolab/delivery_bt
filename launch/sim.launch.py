"""
Part of delivery.launch.py: the simulated robot, no Nav2.

    /cmd_vel --> drive_sim --tf odom->base_footprint, tf map->world (static)
                     |     --/battery_state, service /dock/charge
                     |
                     +--/sim/true_pose--> laser_sim --/scan-->
                                          robot_viz --/viz/markers--> RViz

Nodes started here:
    drive_sim       true motion, drifting odometry, battery and dock (replaces Gazebo)
    laser_sim       simulated 2D lidar from the true pose
    robot_viz       true walls, stations, battery, true and believed robot (only for looking)
    base_scan_tf    static tf base_footprint -> base_scan (lidar in the middle)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    world = LaunchConfiguration('world')
    range_max = ParameterValue(LaunchConfiguration('range_max'), value_type=float)
    noise_std = ParameterValue(LaunchConfiguration('noise_std'), value_type=float)
    battery = ParameterValue(LaunchConfiguration('battery'), value_type=float)
    blocked = ParameterValue(LaunchConfiguration('blocked'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='loop', choices=['loop', 'room'],
                              description='loop: corridor of slide 8, room: room of day 6'),
        DeclareLaunchArgument('range_max', default_value='3.5',
                              description='lidar range in m'),
        DeclareLaunchArgument('noise_std', default_value='0.02',
                              description='lidar noise (standard deviation) in m'),
        DeclareLaunchArgument('battery', default_value='70.0',
                              description='battery level at the start in %'),
        DeclareLaunchArgument('blocked', default_value='false',
                              description='true: the block closes its corridor from the start'),

        Node(package='delivery_bt', executable='drive_sim', output='screen',
             parameters=[{'world': world, 'battery': battery, 'blocked': blocked}]),
        Node(package='delivery_bt', executable='laser_sim', output='screen',
             parameters=[{'world': world, 'range_max': range_max,
                          'noise_std': noise_std}]),
        Node(package='delivery_bt', executable='robot_viz',
             parameters=[{'world': world}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_scan_tf',
             arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'base_scan']),
    ])
