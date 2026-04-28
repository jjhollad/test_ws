#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

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
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Map YAML used when slam:=False (empty uses parent default)',
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
        default_value='0.0',
        description='Spawn X position in turtlebot3_world',
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Spawn Y position in turtlebot3_world',
    )
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.01',
        description='Spawn Z position',
    )

    rectangular_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('generic_motor_driver'),
                'launch',
                'rectangular_classic_nav2_sim.launch.py',
            ])
        ]),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'headless': LaunchConfiguration('headless'),
            'slam': LaunchConfiguration('slam'),
            'world': os.path.join(tb3_gazebo_dir, 'worlds', 'turtlebot3_world.world'),
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
        }.items(),
    )

    return LaunchDescription([
        use_rviz_arg,
        headless_arg,
        slam_arg,
        map_arg,
        params_file_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        rectangular_sim,
    ])
