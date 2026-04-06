#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Generic motor driver node
    generic_motor_driver_node = Node(
        package='generic_motor_driver',
        executable='generic_motor_driver',
        name='generic_motor_driver',
        output='screen',
        
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('odom', 'odom'),
            ('joint_states', 'joint_states'),
        ]
    )

    return LaunchDescription([

        generic_motor_driver_node,
    ])

