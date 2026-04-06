#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    
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

    # Note: base_footprint to base_link transform is defined in URDF, no need for static transform

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/user/test_ws/src/create_robot/create_driver/rviz/rectangular_robot.rviz']
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        rviz_arg,
        
        # Nodes
        robot_state_publisher_node,
        motor_driver_node,
        rviz_node,
    ])
