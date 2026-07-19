#!/usr/bin/env python3

"""Open the corridor museum in Gazebo Classic without spawning a robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gazebo_share = get_package_share_directory("gazebo_ros")
    demo_share = get_package_share_directory("create2_demo")
    world = os.path.join(demo_share, "worlds", "corridor_museum.world")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "gazebo.launch.py")
            ),
            launch_arguments={"world": world}.items(),
        )
    ])
