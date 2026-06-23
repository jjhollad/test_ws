#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

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
    
    relay_status_rate_arg = DeclareLaunchArgument(
        'relay_status_publish_rate',
        default_value='1.0',
        description='Relay status publishing rate (Hz)'
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
        description='Full path to the saved map yaml file to load'
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
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            # Must match URDF joint names so robot_state_publisher can publish wheel TF.
            'joint_names': ['left_rear_wheel_joint', 'right_rear_wheel_joint'],
            'swap_motors': True,
            'linear_command_sign': -1.0,
            'linear_odom_sign': -1.0,
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

    # Joystick teleop goes through twist_mux so manual control can override Nav2.
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

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('generic_motor_driver'),
                'config',
                'xbox_twist_mux.config.yaml',
            ])
        ],
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('twist_mux'),
                'config',
                'twist_mux_locks.yaml',
            ]),
            PathJoinSubstitution([
                FindPackageShare('generic_motor_driver'),
                'config',
                'twist_mux_topics.yaml',
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
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

    # Saved-map localization brings up map_server, AMCL, and their lifecycle manager.
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'map': LaunchConfiguration('map'),
            'autostart': 'True',
            'use_composition': 'False',
        }.items()
    )

    # Nav2 navigation servers with command topics remapped for twist_mux.
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('generic_motor_driver'),
                'launch',
                'navigation_launch_mux.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': 'True',
            'use_composition': 'False',
        }.items()
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        # Launch arguments
        joy_dev_arg,
        relay_dev_arg,
        relay_baud_arg,
        relay_status_rate_arg,
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
        joy_node,
        teleop_twist_joy_node,
        twist_mux_node,
        lidar_node,
        nav2_localization_launch,
        nav2_navigation_launch,
        rviz_node,
    ] + ([relay_button_node] if relay_button_node is not None else []))
