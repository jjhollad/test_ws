#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

def generate_launch_description():
    # Declare launch arguments
    
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
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

    relay_dev_arg = DeclareLaunchArgument(
        'relay_dev',
        default_value='/dev/ttyACM0',
        description='Serial device path for relay controller'
    )
    
    # Get the URDF file path
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

    # Generic motor driver with rectangular robot parameters
    motor_driver_node = Node(
        package='generic_motor_driver',
        executable='generic_motor_driver',
        name='generic_motor_driver',
        output='screen',
        parameters=[{
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'joint_names': ['right_rear_wheel_joint', 'left_rear_wheel_joint'],  # M1/M2 mapping is physically swapped
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Relay controller node
    relay_controller_node = Node(
        package='generic_motor_driver',
        executable='relay_controller',
        name='relay_controller',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('relay_dev'),
            'baud': 115200,
            'status_publish_rate': 1.0,
        }]
    )

    # Joystick node
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

    # Note: base_footprint to base_link transform is defined in URDF, no need for static transform

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/user/test_ws/src/create_robot/create_driver/rviz/rectangular_robot.rviz']
    )

    launch_items = [
        # Launch arguments
        joy_dev_arg,
        use_sim_time_arg,
        rviz_arg,
        relay_dev_arg,
        
        # Nodes
        robot_state_publisher_node,
        motor_driver_node,
        relay_controller_node,
        joy_node,
        teleop_twist_joy_node,
        rviz_node,
    ]

    if relay_button_node is not None:
        launch_items.append(relay_button_node)

    return LaunchDescription(launch_items)
