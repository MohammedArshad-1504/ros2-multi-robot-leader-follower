import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'leader_follower'
    pkg_share = get_package_share_directory(package_name)

    params_file = os.path.join(
        pkg_share,
        'config',
        'follower_params.yaml'
    )

    follower_number = LaunchConfiguration('follower_number')
    desired_distance = LaunchConfiguration('desired_distance')
    base_frame = LaunchConfiguration('base_frame')
    leader_frame = LaunchConfiguration('leader_frame')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    map_yaml = LaunchConfiguration('map_yaml')

    return LaunchDescription([
        DeclareLaunchArgument('follower_number', default_value='1'),
        DeclareLaunchArgument('desired_distance', default_value='1.0'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='follower_1_base_link'
        ),
        DeclareLaunchArgument(
            'leader_frame',
            default_value='leader_base_link'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/follower_1/odom'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/follower_1/front_lidar/scan'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/follower_1/cmd_vel'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.expanduser(
                '~/leader_follower_ws/maps/two_rooms_map.yaml'
            )
        ),

        Node(
            package=package_name,
            executable='follower_controller',
            name=['follower_controller_', follower_number],
            parameters=[
                params_file,
                {
                    'follower_number': follower_number,
                    'desired_distance': desired_distance,
                    'base_frame': base_frame,
                    'leader_frame': leader_frame,
                    'odom_topic': odom_topic,
                    'scan_topic': scan_topic,
                    'cmd_vel_topic': cmd_vel_topic,
                    'map_yaml': map_yaml,
                    'use_sim_time': True,
                },
            ],
            output='screen',
        ),
    ])

