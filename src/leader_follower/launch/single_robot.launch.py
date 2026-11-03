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

    package_share = get_package_share_directory(package_name)

    robot_xacro = os.path.join(
        package_share,
        'urdf',
        'robot.xacro'
    )

    world_file = os.path.join(
        package_share,
        'worlds',
        'two_rooms.sdf'
    )

    gazebo_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    # ---------------------------------------------------------
    # Robot description
    # ---------------------------------------------------------

    robot_description = ParameterValue(
        Command([
            'xacro ',
            robot_xacro,
            ' cmd_vel_topic:=/cmd_vel',
            ' odom_topic:=/odom',
            ' lidar_topic:=/front_lidar/scan',
            ' camera_topic:=/front_camera/image_raw',
            ' odom_frame:=odom',
            ' base_frame:=base_link',
            ' tf_topic:=/robot/tf',
        ]),
        value_type=str
    )

    # ---------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # ---------------------------------------------------------
    # Robot State Publisher
    # ---------------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Static transform: world -> odom
    # ---------------------------------------------------------


    # ---------------------------------------------------------
    # Spawn robot in Gazebo
    # ---------------------------------------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'leader',
            '-topic',
            '/robot_description',
            '-x',
            '0.5',
            '-y',
            '0.0',
            '-z',
            '0.085',
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Gazebo <-> ROS bridge
    # ---------------------------------------------------------

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='single_robot_bridge',
        arguments=[
            '/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/front_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # Gazebo joint states
            '/world/test_world/model/leader/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            # Gazebo TF -> ROS TF
            ('/robot/tf', '/tf'),

            # Gazebo joint states -> standard ROS joint_states
            (
                '/world/test_world/model/leader/joint_state',
                '/joint_states'
            ),
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Clock bridge
    # ---------------------------------------------------------

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Launch everything
    # ---------------------------------------------------------

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
       
        spawn_robot,
        bridge,
        clock_bridge,
    ])
