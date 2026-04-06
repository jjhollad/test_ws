#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value='/dev/ttyACM0',
        description='Serial device path for Arduino DUE'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Serial baud rate'
    )
    
    status_rate_arg = DeclareLaunchArgument(
        'status_publish_rate',
        default_value='1.0',
        description='Status publishing rate (Hz)'
    )

    # Relay controller node
    relay_controller_node = Node(
        package='generic_motor_driver',
        executable='relay_controller',
        name='relay_controller',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('dev'),
            'baud': LaunchConfiguration('baud'),
            'status_publish_rate': LaunchConfiguration('status_publish_rate'),
        }],
        remappings=[
            ('relay1', 'relay1'),
            ('relay2', 'relay2'),
            ('relay3', 'relay3'),
            ('relay4', 'relay4'),
            ('relay_all', 'relay_all'),
            ('relay_command', 'relay_command'),
            ('relay_status', 'relay_status'),
            ('relay_feedback', 'relay_feedback'),
        ]
    )

    return LaunchDescription([
        dev_arg,
        baud_arg,
        status_rate_arg,
        relay_controller_node,
    ])
