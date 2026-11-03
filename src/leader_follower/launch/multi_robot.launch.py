import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def create_robots(context):

    package_name = 'leader_follower'
    pkg_share = get_package_share_directory(package_name)

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

    robot_count = int(
        LaunchConfiguration('robot_count').perform(context)
    )

    follower_spacing = float(
        LaunchConfiguration('follower_spacing').perform(context)
    )

    leader_x = float(
        LaunchConfiguration('leader_x').perform(context)
    )

    leader_y = float(
        LaunchConfiguration('leader_y').perform(context)
    )

    map_yaml = os.path.expanduser(
        '~/leader_follower_ws/maps/two_rooms_map.yaml'
    )

    actions = []
    follower_amcl_names = []

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

    actions.append(gazebo)

    # ============================================================
    # LEADER ROBOT DESCRIPTION
    # ============================================================

    leader_description = ParameterValue(
        Command([
            'xacro ',
            robot_xacro,

            ' prefix:=leader_',

            ' cmd_vel_topic:=/leader/cmd_vel_safe',

            ' odom_topic:=/leader/odom',
            ' lidar_topic:=/leader/front_lidar/scan',
            ' camera_topic:=/leader/front_camera/image_raw',

            ' odom_frame:=leader/odom',
            ' base_frame:=leader_base_link',

            ' tf_topic:=/leader_/tf',
        ]),
        value_type=str
    )

    # ============================================================
    # LEADER STATE PUBLISHER
    # ============================================================

    actions.append(
        Node(
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
    )

    # ============================================================
    # LEADER SPAWN
    # ============================================================

    actions.append(
        Node(
            package='ros_gz_sim',
            executable='create',

            arguments=[
                '-name',
                'leader',

                '-topic',
                '/leader/robot_description',

                '-x',
                str(leader_x),

                '-y',
                str(leader_y),

                '-z',
                '0.085',
            ],

            output='screen'
        )
    )

    # ============================================================
    # LEADER BRIDGE
    # ============================================================

    actions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',

            name='leader_bridge',

            arguments=[
                '/leader/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

                '/leader/front_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

                '/leader/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

                '/leader/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

                '/leader_/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

                '/leader/cmd_vel_safe@geometry_msgs/msg/Twist]gz.msgs.Twist',

                # Gazebo wheel joint states -> ROS JointState
                '/world/test_world/model/leader/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],

            remappings=[
                (
                    '/leader_/tf',
                    '/tf'
                ),
                (
                    '/world/test_world/model/leader/joint_state',
                    '/leader/joint_states'
                ),
            ],

            output='screen'
        )
    )

    # ============================================================
    # LEADER SAFETY CONTROLLER
    # ============================================================

    actions.append(
        Node(
            package='leader_follower',
            executable='leader_safety_controller',

            name='leader_safety_controller',

            parameters=[
                {
                    'use_sim_time': True,

                    'obstacle_stop_distance': 0.45,
                    'obstacle_slow_distance': 0.95,

                    'front_sector_degrees': 30.0,

                    'command_timeout': 0.50,
                    'scan_timeout': 0.50,
                }
            ],

            output='screen'
        )
    )

    # ============================================================
    # CLOCK
    # ============================================================

    actions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',

            name='clock_bridge',

            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],

            output='screen'
        )
    )

    # ============================================================
    # FOLLOWERS
    # robot_count includes the leader.
    #
    # robot_count = 1 -> leader only
    # robot_count = 2 -> leader + follower_1
    # robot_count = 3 -> leader + follower_1 + follower_2
    # ============================================================

    for follower_number in range(1, robot_count):

        follower_name = f'follower_{follower_number}'
        prefix = f'{follower_name}_'
        base_frame = f'{follower_name}_base_link'
        odom_frame = f'{follower_name}/odom'

        lidar_topic = (
            f'/{follower_name}/front_lidar/scan'
        )

        camera_topic = (
            f'/{follower_name}/front_camera/image_raw'
        )

        odom_topic = (
            f'/{follower_name}/odom'
        )

        cmd_vel_topic = (
            f'/{follower_name}/cmd_vel'
        )

        tf_topic = (
            f'/{follower_name}_/tf'
        )

        spawn_x = (
            leader_x
            - follower_number * follower_spacing
        )
        spawn_y = leader_y

        # --------------------------------------------------------
        # Formation slot for this follower.
        # follower_1 -> 1 * spacing
        # follower_2 -> 2 * spacing
        # follower_3 -> 3 * spacing
        # --------------------------------------------------------

        desired_distance = (
            follower_number * follower_spacing
        )

        # ========================================================
        # FOLLOWER ROBOT DESCRIPTION
        # ========================================================

        follower_description = ParameterValue(
            Command([
                'xacro ',
                robot_xacro,

                f' prefix:={prefix}',

                f' cmd_vel_topic:={cmd_vel_topic}',
                f' odom_topic:={odom_topic}',
                f' lidar_topic:={lidar_topic}',
                f' camera_topic:={camera_topic}',

                f' odom_frame:={odom_frame}',
                f' base_frame:={base_frame}',

                f' tf_topic:={tf_topic}',
            ]),
            value_type=str
        )

        # ========================================================
        # FOLLOWER STATE PUBLISHER
        # ========================================================

        actions.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',

                namespace=follower_name,
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
        )

        # ========================================================
        # FOLLOWER SPAWN
        # ========================================================

        actions.append(
            Node(
                package='ros_gz_sim',
                executable='create',

                arguments=[
                    '-name',
                    follower_name,

                    '-topic',
                    f'/{follower_name}/robot_description',

                    '-x',
                    str(spawn_x),

                    '-y',
                    str(spawn_y),

                    '-z',
                    '0.085',
                ],

                output='screen'
            )
        )

        # ========================================================
        # FOLLOWER BRIDGE
        # ========================================================

        actions.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',

                name=f'{follower_name}_bridge',

                arguments=[
                    f'{lidar_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

                    f'{camera_topic}@sensor_msgs/msg/Image[gz.msgs.Image',

                    (
                        f'/{follower_name}/front_camera/camera_info'
                        '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
                    ),

                    (
                        f'{odom_topic}'
                        '@nav_msgs/msg/Odometry[gz.msgs.Odometry'
                    ),

                    (
                        f'{tf_topic}'
                        '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
                    ),

                    (
                        f'{cmd_vel_topic}'
                        '@geometry_msgs/msg/Twist]gz.msgs.Twist'
                    ),

                    # Gazebo wheel joint states -> ROS JointState
                    (
                        f'/world/test_world/model/{follower_name}/joint_state'
                        '@sensor_msgs/msg/JointState[gz.msgs.Model'
                    ),
                ],

                remappings=[
                    (
                        tf_topic,
                        '/tf'
                    ),
                    (
                        f'/world/test_world/model/{follower_name}/joint_state',
                        f'/{follower_name}/joint_states'
                    ),
                ],

                output='screen'
            )
        )

        # ========================================================
        # FOLLOWER AMCL
        # Each follower gets an independent map -> odom transform.
        # ========================================================

        follower_amcl_name = f'{follower_name}_amcl'
        follower_amcl_names.append(follower_amcl_name)

        actions.append(
            Node(
                package='nav2_amcl',
                executable='amcl',
                name=follower_amcl_name,
                parameters=[
                    {
                        'use_sim_time': True,
                        'global_frame_id': 'map',
                        'odom_frame_id': odom_frame,
                        'base_frame_id': base_frame,
                        'scan_topic': lidar_topic,
                        'tf_broadcast': True,
                        'set_initial_pose': False,
                        'laser_min_range': 0.10,
                        'laser_max_range': 12.0,
                        'transform_tolerance': 1.0,
                    }
                ],
                remappings=[
                    ('initialpose', f'/{follower_name}/amcl_initialpose'),
                    ('scan', lidar_topic),
                    # Keep follower AMCL pose outputs separate from the
                    # leader's global /amcl_pose topic so RViz cannot mix
                    # the two robots' pose messages.
                    ('amcl_pose', f'/{follower_name}/amcl_pose'),
                    ('particlecloud', f'/{follower_name}/particlecloud'),
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static'),
                ],
                output='screen',
            )
        )

        # ========================================================
        # FOLLOWER CONTROLLER
        # ========================================================

        actions.append(
            Node(
                package='leader_follower',
                executable='follower_controller',

                name=f'follower_controller_{follower_number}',

                parameters=[
                    {
                        'use_sim_time': True,

                        # Formation slot
                        'desired_distance': desired_distance,
                        'spacing': follower_spacing,
                        'follower_number': follower_number,
                        # Controller gains
                        'linear_gain': 0.5,
                        'angular_gain': 1.5,

                        'max_linear_speed': 0.60,
                        'max_angular_speed': 1.10,

                        'control_rate': 30.0,
                        'data_timeout': 0.5,

                        # Robot-specific frames/topics
                        'base_frame': base_frame,
                        'leader_frame': 'leader_base_link',
                        'scan_topic': lidar_topic,
                        'odom_topic': odom_topic,
                        'cmd_vel_topic': cmd_vel_topic,
                        'map_yaml': map_yaml,
                        'initialpose_topic': f'/{follower_name}/amcl_initialpose',
                        'orientation_tolerance': 0.08,
                        'orientation_gain': 1.5,

                        # Recovery
                        'position_tolerance': 0.10,
                        'rejoin_distance': 2.50,
                        'stuck_timeout': 1.2,
                        'stuck_min_command': 0.08,
                        'stuck_min_motion': 0.02,
                        'recovery_turn_time': 1.5,
                        'recovery_turn_speed': 0.85,
                        'recovery_forward_speed': 0.0,

                        # Local inflated LiDAR planner
                        'local_map_size': 8.0,
                        'local_map_resolution': 0.10,
                        'inflation_radius': 0.40,
                        'path_lookahead_cells': 5,
                    }
                ],

                output='screen'
            )
        )

    # ================================================================
    # FOLLOWER AMCL LIFECYCLE
    # ================================================================

    if follower_amcl_names:
        actions.append(
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_follower_localization',
                parameters=[
                    {
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': follower_amcl_names,
                    }
                ],
                output='screen',
            )
        )

    return actions


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_count',
            default_value='3',
            description=(
                'Total number of robots including the leader'
            )
        ),

        DeclareLaunchArgument(
            'follower_spacing',
            default_value='1.0',
            description=(
                'Desired spacing between consecutive formation positions'
            )
        ),

        DeclareLaunchArgument(
            'leader_x',
            default_value='0.5',
            description='Leader initial X position'
        ),

        DeclareLaunchArgument(
            'leader_y',
            default_value='0.0',
            description='Leader initial Y position'
        ),

        OpaqueFunction(
            function=create_robots
        ),
    ])





