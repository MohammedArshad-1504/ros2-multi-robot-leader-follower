import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'leader_follower'

    pkg_share = get_package_share_directory(
        package_name
    )

    # ============================================================
    # FILE PATHS
    # ============================================================

    robot_xacro = os.path.join(
        pkg_share,
        'urdf',
        'robot.xacro'
    )

    world_file = os.path.join(
        pkg_share,
        'worlds',
        'two_rooms.sdf'
    )

    # ============================================================
    # LEADER ROBOT DESCRIPTION
    # ============================================================

    leader_description = ParameterValue(
        Command([
            'xacro ',
            robot_xacro,

            ' prefix:=leader_',

            # User command goes to safety controller
            ' cmd_vel_topic:=/leader/cmd_vel_safe',

            ' odom_topic:=/leader/odom',
            ' lidar_topic:=/leader/front_lidar/scan',
            ' camera_topic:=/leader/front_camera/image_raw',

            ' odom_frame:=leader/odom',
            ' base_frame:=leader/base_link',

            ' tf_topic:=/leader_/tf',
        ]),
        value_type=str
    )

    # ============================================================
    # FOLLOWER ROBOT DESCRIPTION
    # ============================================================

    follower_description = ParameterValue(
        Command([
            'xacro ',
            robot_xacro,

            ' prefix:=follower_',

            ' cmd_vel_topic:=/follower/cmd_vel',

            ' odom_topic:=/follower/odom',
            ' lidar_topic:=/follower/front_lidar/scan',
            ' camera_topic:=/follower/front_camera/image_raw',

            ' odom_frame:=follower/odom',
            ' base_frame:=follower/base_link',

            ' tf_topic:=/follower_/tf',
        ]),
        value_type=str
    )

    # ============================================================
    # GAZEBO
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # ============================================================
    # LEADER STATE PUBLISHER
    # ============================================================

    leader_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',

        namespace='leader',
        name='robot_state_publisher',

        parameters=[
            {
                'robot_description': leader_description,
                'use_sim_time': True,
            }
        ],

        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],

        output='screen'
    )

    # ============================================================
    # FOLLOWER STATE PUBLISHER
    # ============================================================

    follower_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',

        namespace='follower',
        name='robot_state_publisher',

        parameters=[
            {
                'robot_description': follower_description,
                'use_sim_time': True,
            }
        ],

        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],

        output='screen'
    )

    # ============================================================
    # SPAWN LEADER
    # ============================================================

    spawn_leader = Node(
        package='ros_gz_sim',
        executable='create',

        arguments=[
            '-name',
            'leader',

            '-topic',
            '/leader/robot_description',

            '-x',
            '0.5',

            '-y',
            '0.0',

            '-z',
            '0.085',
        ],

        output='screen'
    )

    # ============================================================
    # SPAWN FOLLOWER
    # ============================================================

    spawn_follower = Node(
        package='ros_gz_sim',
        executable='create',

        arguments=[
            '-name',
            'follower',

            '-topic',
            '/follower/robot_description',

            '-x',
            '-1.5',

            '-y',
            '0.0',

            '-z',
            '0.085',
        ],

        output='screen'
    )

    # ============================================================
    # LEADER GAZEBO <-> ROS BRIDGE
    # ============================================================

    leader_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        name='leader_bridge',

        arguments=[

            '/leader/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            '/leader/front_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

            '/leader/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            '/leader/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            '/leader_/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # IMPORTANT:
            # Bridge the SAFE command to Gazebo.
            '/leader/cmd_vel_safe@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],

        remappings=[
            (
                '/leader_/tf',
                '/tf'
            ),
        ],

        output='screen'
    )

    # ============================================================
    # FOLLOWER GAZEBO <-> ROS BRIDGE
    # ============================================================

    follower_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        name='follower_bridge',

        arguments=[

            '/follower/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            '/follower/front_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

            '/follower/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            '/follower/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            '/follower_/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            '/follower/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],

        remappings=[
            (
                '/follower_/tf',
                '/tf'
            ),
        ],

        output='screen'
    )

    # ============================================================
    # CLOCK
    # ============================================================

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        name='clock_bridge',

        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],

        output='screen'
    )

    # ============================================================
    # LEADER SAFETY CONTROLLER
    # ============================================================

    leader_safety_controller = Node(
        package='leader_follower',
        executable='leader_safety_controller',

        parameters=[
            {
                'use_sim_time': True,

                'obstacle_stop_distance': 0.60,
                'obstacle_slow_distance': 1.00,

                'front_sector_degrees': 30.0,

                'command_timeout': 0.50,
                'scan_timeout': 0.50,
            }
        ],

        output='screen'
    )

    # ============================================================
    # FOLLOWER CONTROLLER
    # ============================================================

    follower_controller = Node(
        package='leader_follower',
        executable='follower_controller',

        parameters=[
            {
                'use_sim_time': True,

                'desired_distance': 1.2,
                'safety_distance': 0.75,

                'obstacle_stop_distance': 0.60,
                'obstacle_slow_distance': 0.90,

                'linear_gain': 0.5,
                'angular_gain': 1.5,

                'max_linear_speed': 0.5,
                'max_angular_speed': 1.0,

                'control_rate': 20.0,
                'data_timeout': 0.5,
            }
        ],

        output='screen'
    )

    # ============================================================
    # START EVERYTHING
    # ============================================================

    return LaunchDescription([

        gazebo,

        leader_state_publisher,
        follower_state_publisher,

        spawn_leader,
        spawn_follower,

        leader_bridge,
        follower_bridge,

        clock_bridge,

        leader_safety_controller,
        follower_controller,
    ])
