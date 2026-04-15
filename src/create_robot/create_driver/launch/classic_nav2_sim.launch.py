#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('generic_motor_driver'),
            'config',
            'nav2_params_sim.yaml'
        ]),
        description='Nav2 params for simulation'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run without Gazebo GUI'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Run SLAM instead of static map localization'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(nav2_bringup_dir, 'worlds', 'world_only.model'),
        description='Gazebo world model file'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Map YAML when slam:=False'
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'tb3_simulation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': LaunchConfiguration('params_file'),
            'headless': LaunchConfiguration('headless'),
            'slam': LaunchConfiguration('slam'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'world': LaunchConfiguration('world'),
            'map': LaunchConfiguration('map'),
        }.items()
    )

    return LaunchDescription([
        # Required by TB3 simulation launch stack
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        params_file_arg,
        headless_arg,
        slam_arg,
        use_rviz_arg,
        world_arg,
        map_arg,
        sim_launch,
    ])
