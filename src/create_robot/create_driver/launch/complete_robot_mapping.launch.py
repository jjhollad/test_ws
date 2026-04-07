#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os

def generate_launch_description():
    # Declare launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    
    relay_dev_arg = DeclareLaunchArgument(
        'relay_dev',
        default_value='/dev/ttyACM0',
        description='Serial device path for Arduino relay controller'
    )
    
    relay_baud_arg = DeclareLaunchArgument(
        'relay_baud',
        default_value='115200',
        description='Serial baud rate for Arduino relay controller'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('generic_motor_driver'),
            'rviz',
            'mapping.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    lidar_dev_arg = DeclareLaunchArgument(
        'lidar_dev',
        default_value='/dev/lidar',
        description='Serial device path for lidar'
    )
    
    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_baud',
        default_value='115200',
        description='Serial baud rate for lidar'
    )
    
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='laser_frame',
        description='Frame ID for lidar scan messages'
    )
    
    # SLAM Toolbox arguments
    slam_toolbox_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('generic_motor_driver'),
            'config',
            'mapper_params_online_async.yaml'
        ]),
        description='Full path to the SLAM parameters file to use'
    )
    
    # Nav2 arguments
    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('generic_motor_driver'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Full path to the Nav2 parameters file'
    )
    
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Enable Nav2 navigation stack'
    )

    # Get URDF file path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('create_description'),
        'urdf',
        'rectangular_robot.urdf.xacro'
    ])
    
    # Robot state publisher with xacro processing
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
        }]
    )

    # Generic motor driver
    motor_driver_node = Node(
        package='generic_motor_driver',
        executable='generic_motor_driver',
        name='generic_motor_driver',
        output='screen',
    )

    # Relay controller
    relay_controller_node = Node(
        package='generic_motor_driver',
        executable='relay_controller',
        name='relay_controller',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('relay_dev'),
            'baud': LaunchConfiguration('relay_baud'),
        }]
    )

    # Joystick node (same behavior as rectangular_robot launch)
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

    # Teleop twist node for cmd_vel driving
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('teleop_twist_joy'),
                'config',
                'xbox.config.yaml'
            ])
        ],
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    # Optional relay button mapping node (requires joy_teleop package).
    relay_button_node = None
    try:
        get_package_share_directory('joy_teleop')
        relay_button_node = Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='relay_button_teleop',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('generic_motor_driver'),
                    'config',
                    'xbox_relay_buttons.yaml'
                ])
            ]
        )
    except PackageNotFoundError:
        relay_button_node = None

    # RPLidar node
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_dev'),
            'serial_baudrate': LaunchConfiguration('lidar_baud'),
            'frame_id': LaunchConfiguration('lidar_frame'),
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[
            ('scan', 'scan'),
        ]
    )

    # SLAM Toolbox node (for mapping)
    slam_toolbox_node = Node(
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
            ('/scan', 'scan'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
        ]
    )

    # Nav2 navigation stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_nav2')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
        }.items()
    )

    # RViz (configured for fullscreen mapping view)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    launch_items = [
        # Launch arguments
        joy_dev_arg,
        relay_dev_arg,
        relay_baud_arg,
        use_sim_time_arg,
        rviz_arg,
        rviz_config_arg,
        lidar_dev_arg,
        lidar_baud_arg,
        lidar_frame_arg,
        slam_toolbox_params_arg,
        nav2_params_file_arg,
        use_nav2_arg,
        
        # Nodes
        robot_state_publisher_node,
        motor_driver_node,
        relay_controller_node,
        joy_node,
        teleop_twist_joy_node,
        lidar_node,
        slam_toolbox_node,
        nav2_launch,
        rviz_node,
    ]

    if relay_button_node is not None:
        launch_items.append(relay_button_node)

    return LaunchDescription(launch_items)

