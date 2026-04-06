#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Declare launch arguments
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value='/dev/motor_controller',
        description='Serial device path for motor controller'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Serial baud rate for motor controller'
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
    
    relay_status_rate_arg = DeclareLaunchArgument(
        'relay_status_publish_rate',
        default_value='1.0',
        description='Relay status publishing rate (Hz)'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.57',
        description='Distance between left and right wheels (meters)'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.12',
        description='Wheel radius (meters)'
    )
    
    motor_gear_ratio_arg = DeclareLaunchArgument(
        'motor_gear_ratio',
        default_value='90.0',
        description='Motor internal gear ratio'
    )
    
    belt_drive_ratio_arg = DeclareLaunchArgument(
        'belt_drive_ratio',
        default_value='6.4',
        description='Belt drive ratio'
    )
    
    apply_gear_reduction_arg = DeclareLaunchArgument(
        'apply_gear_reduction',
        default_value='true',
        description='If true, divide encoder by gear ratio (encoder = motor counts). If false, encoder already in wheel rotations.'
    )
    
    encoder_reduction_factor_arg = DeclareLaunchArgument(
        'encoder_reduction_factor',
        default_value='68.17',
        description='Additional reduction factor to multiply the gear reduction.'
    )
    
    loop_hz_arg = DeclareLaunchArgument(
        'loop_hz',
        default_value='20.0',
        description='Update loop frequency (Hz)'
    )
    
    max_motor_speed_arg = DeclareLaunchArgument(
        'max_motor_speed',
        default_value='1000.0',
        description='Maximum motor speed'
    )
    
    invert_left_encoder_arg = DeclareLaunchArgument(
        'invert_left_encoder',
        default_value='false',
        description='Invert left encoder direction'
    )
    
    invert_right_encoder_arg = DeclareLaunchArgument(
        'invert_right_encoder',
        default_value='true',
        description='Invert right encoder direction'
    )
    
    invert_left_motor_arg = DeclareLaunchArgument(
        'invert_left_motor',
        default_value='true',
        description='Invert left motor direction'
    )
    
    invert_right_motor_arg = DeclareLaunchArgument(
        'invert_right_motor',
        default_value='true',
        description='Invert right motor direction'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Base frame ID'
    )
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
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
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'
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
        parameters=[{
            'dev': LaunchConfiguration('dev'),
            'baud': LaunchConfiguration('baud'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'motor_gear_ratio': LaunchConfiguration('motor_gear_ratio'),
            'belt_drive_ratio': LaunchConfiguration('belt_drive_ratio'),
            'apply_gear_reduction': LaunchConfiguration('apply_gear_reduction'),
            'encoder_reduction_factor': LaunchConfiguration('encoder_reduction_factor'),
            'loop_hz': LaunchConfiguration('loop_hz'),
            'max_motor_speed': LaunchConfiguration('max_motor_speed'),
            'invert_left_encoder': LaunchConfiguration('invert_left_encoder'),
            'invert_right_encoder': LaunchConfiguration('invert_right_encoder'),
            'invert_left_motor': LaunchConfiguration('invert_left_motor'),
            'invert_right_motor': LaunchConfiguration('invert_right_motor'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'publish_tf': True,
            'joint_names': ['left_rear_wheel_joint', 'right_rear_wheel_joint'],
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
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
            'status_publish_rate': LaunchConfiguration('relay_status_publish_rate'),
        }]
    )

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

    # Map server (load saved map)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('map')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map'),
        }]
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
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
        }.items()
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        # Launch arguments
        dev_arg,
        baud_arg,
        relay_dev_arg,
        relay_baud_arg,
        relay_status_rate_arg,
        wheel_base_arg,
        wheel_radius_arg,
        motor_gear_ratio_arg,
        belt_drive_ratio_arg,
        apply_gear_reduction_arg,
        encoder_reduction_factor_arg,
        loop_hz_arg,
        max_motor_speed_arg,
        invert_left_encoder_arg,
        invert_right_encoder_arg,
        invert_left_motor_arg,
        invert_right_motor_arg,
        base_frame_arg,
        odom_frame_arg,
        use_sim_time_arg,
        rviz_arg,
        rviz_config_arg,
        lidar_dev_arg,
        lidar_baud_arg,
        lidar_frame_arg,
        nav2_params_file_arg,
        map_file_arg,
        
        # Nodes
        robot_state_publisher_node,
        motor_driver_node,
        relay_controller_node,
        lidar_node,
        map_server_node,
        nav2_launch,
        rviz_node,
    ])

