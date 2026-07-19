#!/usr/bin/env python3

"""Launch a complete TurtleBot3 Gazebo SLAM and Nav2 demonstration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    nav2_share = get_package_share_directory("nav2_bringup")
    turtlebot_gazebo_share = get_package_share_directory("turtlebot3_gazebo")

    world = os.path.join(
        turtlebot_gazebo_share, "worlds", "turtlebot3_world.world"
    )
    robot_sdf = os.path.join(
        turtlebot_gazebo_share, "models", "turtlebot3_waffle", "model.sdf"
    )
    robot_urdf = os.path.join(nav2_share, "urdf", "turtlebot3_waffle.urdf")
    with open(robot_urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        AppendEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            os.path.join(turtlebot_gazebo_share, "models"),
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="False",
            description="Run Gazebo without its graphical client",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="True",
            description="Start RViz with the Nav2 display configuration",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
            ),
            launch_arguments={"world": world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
            ),
            condition=UnlessCondition(headless),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": True}
            ],
            output="screen",
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "turtlebot3_waffle",
                "-file", robot_sdf,
                "-x", "-2.0",
                "-y", "-0.5",
                "-z", "0.01",
                "-timeout", "120.0",
            ],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "slam": "True",
                "map": os.path.join(
                    nav2_share, "maps", "turtlebot3_world.yaml"
                ),
                "params_file": os.path.join(
                    nav2_share, "params", "nav2_params.yaml"
                ),
                "use_sim_time": "True",
                "autostart": "True",
                "use_composition": "True",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "rviz_launch.py")
            ),
            condition=IfCondition(use_rviz),
            launch_arguments={"use_sim_time": "True"}.items(),
        ),
    ])
