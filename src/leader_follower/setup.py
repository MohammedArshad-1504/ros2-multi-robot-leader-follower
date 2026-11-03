from setuptools import find_packages, setup


package_name = 'leader_follower'


setup(
    name=package_name,

    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name
            ]
        ),

        (
            'share/' + package_name,
            [
                'package.xml'
            ]
        ),

        (
            'share/' + package_name + '/urdf',
            [
                'urdf/base.xacro',
                'urdf/camera.xacro',
                'urdf/lidar.xacro',
                'urdf/robot.xacro',
                'urdf/stand.xacro',
                'urdf/wheels.xacro',
            ]
        ),

        (
            'share/' + package_name + '/launch',
            [
                'launch/single_robot.launch.py',
                'launch/gazebo.launch.py',
                'launch/multi_robot.launch.py',
                'launch/navigation.launch.py',
                'launch/follower_bringup.launch.py',
            ]
        ),

        (
            'share/' + package_name + '/worlds',
            [
                'worlds/two_rooms.sdf',
            ]
        ),

        (
            'share/' + package_name + '/config',
            [
                'config/slam_mapping.yaml',
                'config/nav2_params.yaml',
                'config/follower_params.yaml',
            ]
        ),

        (
            'share/' + package_name + '/rviz',
            [
                'rviz/leader_navigation.rviz',
            ]
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='mohammed-arshad-s',

    maintainer_email='mohammed-arshad-s@todo.todo',

    description=(
        'Leader-Follower Navigation using ROS 2'
    ),

    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [

            'follower_controller = '
            'leader_follower.follower_controller:main',

            'leader_safety_controller = '
            'leader_follower.leader_safety_controller:main',
        ],
    },
)

