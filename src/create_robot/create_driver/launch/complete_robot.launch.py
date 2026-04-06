#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value='/dev/ttyUSB0',
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
    
    relay_status_rate_arg = DeclareLaunchArgument(
        'relay_status_publish_rate',
        default_value='1.0',
        description='Relay status publishing rate (Hz)'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.3',
        description='Distance between left and right wheels (meters)'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.05',
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
    
    loop_hz_arg = DeclareLaunchArgument(
        'loop_hz',
        default_value='20.0',
        description='Update loop frequency (Hz) - higher = lower latency but more CPU'
    )
    
    max_motor_speed_arg = DeclareLaunchArgument(
        'max_motor_speed',
        default_value='1000.0',
        description='Maximum motor speed'
    )
    
    invert_left_encoder_arg = DeclareLaunchArgument(
        'invert_left_encoder',
        default_value='false',
        description='Invert left encoder direction (fix spinning in circles)'
    )
    
    invert_right_encoder_arg = DeclareLaunchArgument(
        'invert_right_encoder',
        default_value='true',
        description='Invert right encoder direction (fix spinning in circles)'
    )
    
    invert_left_motor_arg = DeclareLaunchArgument(
        'invert_left_motor',
        default_value='true',
        description='Invert left motor direction (fix backwards motors)'
    )
    
    invert_right_motor_arg = DeclareLaunchArgument(
        'invert_right_motor',
        default_value='true',
        description='Invert right motor direction (fix backwards motors)'
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

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': open('/home/user/test_ws/src/create_robot/create_driver/urdf/create_2.urdf').read()
        }]
    )

    # Note: joint_state_publisher is not needed since generic_motor_driver publishes joint states
    # Joint state publisher (for manual testing - disabled to avoid conflicts)
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': LaunchConfiguration('use_sim_time')
    #     }]
    # )

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
            'loop_hz': LaunchConfiguration('loop_hz'),
            'max_motor_speed': LaunchConfiguration('max_motor_speed'),
            'invert_left_encoder': LaunchConfiguration('invert_left_encoder'),
            'invert_right_encoder': LaunchConfiguration('invert_right_encoder'),
            'invert_left_motor': LaunchConfiguration('invert_left_motor'),
            'invert_right_motor': LaunchConfiguration('invert_right_motor'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
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
            'baud': LaunchConfiguration('baud'),
            'status_publish_rate': LaunchConfiguration('relay_status_publish_rate'),
        }]
    )

    # Note: Static transform from base_footprint to base_link is NOT needed
    # because the URDF already defines this relationship via base_footprint_joint
    # (base_link is 0.017m above base_footprint as defined in the URDF)
    # robot_state_publisher will automatically publish this transform from the URDF

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/user/test_ws/src/create_robot/create_driver/rviz/robot_view.rviz']
    )

    # Optional: Navigation stack (uncomment if you want to add navigation)
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('nav2_bringup'),
    #             'launch',
    #             'navigation_launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'params_file': PathJoinSubstitution([
    #             FindPackageShare('generic_motor_driver'),
    #             'config',
    #             'nav2_params.yaml'
    #         ])
    #     }.items()
    # )

    return LaunchDescription([
        # Launch arguments
        dev_arg,
        baud_arg,
        relay_dev_arg,
        relay_status_rate_arg,
        wheel_base_arg,
        wheel_radius_arg,
        motor_gear_ratio_arg,
        belt_drive_ratio_arg,
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
        
        # Nodes
        robot_state_publisher_node,
        # joint_state_publisher_node,  # Disabled - motor driver publishes joint states
        motor_driver_node,
        relay_controller_node,
        # static_transform_node,  # Not needed - URDF defines base_footprint to base_link
        rviz_node,
        
        # Optional navigation
        # nav2_launch,
    ])

