import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    package_name = 'leader_follower'

    pkg_share = get_package_share_directory(
        package_name
    )

    # ============================================================
    # FILES
    # ============================================================

    simulation_launch = os.path.join(
        pkg_share,
        'launch',
        'multi_robot.launch.py'
    )

    nav2_params = os.path.join(
        pkg_share,
        'config',
        'nav2_params.yaml'
    )

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'leader_navigation.rviz'
    )

    map_file = os.path.expanduser(
        '~/leader_follower_ws/maps/two_rooms_map.yaml'
    )

    default_bt_xml = os.path.join(
        get_package_share_directory(
            'nav2_bt_navigator'
        ),
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml'
    )

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    autostart = LaunchConfiguration(
        'autostart'
    )

    params_file = LaunchConfiguration(
        'params_file'
    )

    map_yaml_file = LaunchConfiguration(
        'map'
    )

    default_bt_xml_filename = LaunchConfiguration(
        'default_bt_xml_filename'
    )

    robot_count = LaunchConfiguration(
        'robot_count'
    )

    follower_spacing = LaunchConfiguration(
        'follower_spacing'
    )

    # ============================================================
    # PARAMETER REWRITING
    # ============================================================

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        'default_nav_to_pose_bt_xml':
            default_bt_xml_filename,
        'default_nav_through_poses_bt_xml':
            default_bt_xml_filename,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # ============================================================
    # TF REMAPPING
    # ============================================================

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    # ============================================================
    # MAP SERVER
    # ============================================================

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',

        name='map_server',

        output='screen',

        parameters=[
            configured_params
        ],

        remappings=remappings,
    )

    # ============================================================
    # AMCL
    # ============================================================

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',

        name='amcl',

        output='screen',

        parameters=[
            configured_params
        ],

        remappings=remappings,
    )

    # ============================================================
    # LOCALIZATION LIFECYCLE
    # ============================================================

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',

        name='lifecycle_manager_localization',

        output='screen',

        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,

                'node_names': [
                    'map_server',
                    'amcl',
                ],
            }
        ],
    )

    # ============================================================
    # CONTROLLER SERVER
    # ============================================================

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',

        name='controller_server',

        output='screen',

        parameters=[
            configured_params
        ],

        remappings=[
            *remappings,

            (
                'cmd_vel',
                '/leader/cmd_vel_raw'
            ),
        ],
    )

    # ============================================================
    # PLANNER SERVER
    # ============================================================

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',

        name='planner_server',

        output='screen',

        parameters=[
            configured_params
        ],

        remappings=remappings,
    )

    # ============================================================
    # BEHAVIOR SERVER
    # ============================================================

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',

        name='behavior_server',

        output='screen',

        parameters=[
            configured_params
        ],

        remappings=remappings,
    )

    # ============================================================
    # BT NAVIGATOR
    # ============================================================

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',

        name='bt_navigator',

        output='screen',

        parameters=[
            configured_params
        ],

        remappings=remappings,
    )

    # ============================================================
    # WAYPOINT FOLLOWER
    # ============================================================

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',

        name='waypoint_follower',

        output='screen',

        parameters=[
            configured_params
        ],

        remappings=remappings,
    )

    # ============================================================
    # VELOCITY SMOOTHER
    # ============================================================

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',

        name='velocity_smoother',

        output='screen',

        parameters=[
            configured_params
        ],

        remappings=[
            *remappings,

            (
                'cmd_vel',
                '/leader/cmd_vel_raw'
            ),

            (
                'cmd_vel_smoothed',
                '/leader/cmd_vel'
            ),
        ],
    )

    # ============================================================
    # NAVIGATION LIFECYCLE
    # ============================================================

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',

        name='lifecycle_manager_navigation',

        output='screen',

        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,

                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            }
        ],
    )

    # ============================================================
    # RVIZ
    # ============================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            rviz_config,
        ],
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        },
        output='screen',
    )

    # ============================================================
    # LAUNCH
    # ============================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',

            description='Use Gazebo simulation time'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',

            description=(
                'Automatically activate Nav2 lifecycle nodes'
            )
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params,

            description='Nav2 parameter file'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=map_file,

            description='Saved map YAML file'
        ),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=default_bt_xml,

            description='Default Nav2 behavior tree'
        ),

        DeclareLaunchArgument(
            'robot_count',
            default_value='2',

            description=(
                'Total number of robots including leader'
            )
        ),

        DeclareLaunchArgument(
            'follower_spacing',
            default_value='1.2',

            description=(
                'Initial spacing between robots'
            )
        ),

        # --------------------------------------------------------
        # Multi-robot simulation
        # --------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                simulation_launch
            ),

            launch_arguments={
                'robot_count': robot_count,
                'follower_spacing': follower_spacing,
            }.items()
        ),

        # --------------------------------------------------------
        # Localization
        # --------------------------------------------------------

        map_server,
        amcl,
        lifecycle_manager_localization,

        # --------------------------------------------------------
        # Navigation
        # --------------------------------------------------------

        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_navigation,

        rviz,
    ])

