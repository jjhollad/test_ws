#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open('/home/user/test_ws/src/create_robot/create_driver/urdf/create_2.urdf').read()
        }]
    )

    # Joint state publisher (for testing - publishes zero joint states)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Static transform from base_link to base_footprint
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/user/test_ws/src/create_robot/create_driver/rviz/robot_view.rviz']
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        static_transform_node,
        rviz_node,
    ])


