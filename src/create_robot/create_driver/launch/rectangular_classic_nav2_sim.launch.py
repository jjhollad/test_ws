#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    generic_driver_dir = get_package_share_directory('generic_motor_driver')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz with Nav2 default config',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='If true, do not start Gazebo client',
    )
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Use SLAM (true) or localization with static map (false)',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(generic_driver_dir, 'worlds', 'turtlebot3_world_spacious.world'),
        description='Gazebo Classic world model file',
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Map YAML used when slam:=False',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('generic_motor_driver'),
            'config',
            'nav2_params_sim.yaml',
        ]),
        description='Nav2 params file for simulation',
    )
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='11.9',
        description='Spawn X position (default is hallway center on east side)',
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Spawn Y position',
    )
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.01',
        description='Spawn Z position (kept low to avoid drop-induced drift impulse)',
    )

    urdf_file = PathJoinSubstitution([
        FindPackageShare('create_description'),
        'urdf',
        'rectangular_robot.urdf.xacro',
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
        }],
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            LaunchConfiguration('world'),
        ],
        output='screen',
    )

    gazebo_client = ExecuteProcess(
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('headless')])),
        cmd=['gzclient'],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'rectangular_robot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
        ],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ]),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'True',
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'rviz_launch.py',
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={
            'use_sim_time': 'True',
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(tb3_gazebo_dir, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
        ),
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            os.path.join(tb3_gazebo_dir, 'worlds') + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
        ),
        use_rviz_arg,
        headless_arg,
        slam_arg,
        world_arg,
        map_arg,
        params_file_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        nav2_bringup,
        rviz,
    ])
