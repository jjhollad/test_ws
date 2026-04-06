#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    joy_config_arg = DeclareLaunchArgument(
        'joy_config',
        default_value='xbox360',
        description='Joystick configuration (xbox360, dualshock4, log710, default)'
    )
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.2,
            'autorepeat_rate': 20.0,
        }]
    )

    # Joy teleop node - using inline parameters for simplicity
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[{
            'piloting': {
                'type': 'topic',
                'interface_type': 'geometry_msgs/msg/Twist',
                'topic_name': 'cmd_vel',
                'deadman_buttons': [],
                'axis_mappings': {
                    'linear-x': {
                        'axis': 4,  # Right thumb stick up/down (Xbox360)
                        'scale': 0.4,
                        'offset': 0.0
                    },
                    'angular-z': {
                        'axis': 3,  # Right thumb stick left/right (Xbox360)
                        'scale': 2.5,
                        'offset': 0.0
                    }
                }
            }
        }]
    )

    return LaunchDescription([
        joy_dev_arg,
        joy_config_arg,
        joy_node,
        joy_teleop_node,
    ])
