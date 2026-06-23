#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


DEFAULT_MAP_NAME = 'EERC_SB_LOOP_rot6cw'
DEFAULT_MAP_PATH = '/home/user/EERC_SB_LOOP_rot6cw.yaml'
DEFAULT_START_X = '2.5693857669830322'
DEFAULT_START_Y = '-7.290492057800293'
DEFAULT_START_YAW = '0.0'


def generate_launch_description():
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value=DEFAULT_MAP_NAME,
        description='Saved map basename, without .yaml',
    )
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=DEFAULT_MAP_PATH,
        description='Full path to the saved map yaml file to load',
    )
    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value=DEFAULT_START_X,
        description='Known starting X position in the map frame',
    )
    start_y_arg = DeclareLaunchArgument(
        'start_y',
        default_value=DEFAULT_START_Y,
        description='Known starting Y position in the map frame',
    )
    start_yaw_arg = DeclareLaunchArgument(
        'start_yaw',
        default_value=DEFAULT_START_YAW,
        description='Known starting yaw in radians in the map frame',
    )
    start_cleaning_arg = DeclareLaunchArgument(
        'start_cleaning',
        default_value='true',
        description='Automatically send generated cleaning waypoints to Nav2',
    )
    align_sweeps_to_cell_axis_arg = DeclareLaunchArgument(
        'align_sweeps_to_cell_axis',
        default_value='true',
        description='Fit Boustrophedon sweep direction to each cell long axis',
    )
    snap_sweeps_to_wall_axes_arg = DeclareLaunchArgument(
        'snap_sweeps_to_wall_axes',
        default_value='true',
        description='Snap sweep direction to the dominant rotated wall axes',
    )
    wall_axis_override_deg_arg = DeclareLaunchArgument(
        'wall_axis_override_deg',
        default_value='999.0',
        description='Manual wall axis angle in degrees; leave 999.0 for auto-detect',
    )
    preview_only_arg = DeclareLaunchArgument(
        'preview_only',
        default_value='false',
        description='Publish /coverage_path without sending waypoints to Nav2',
    )
    line_spacing_arg = DeclareLaunchArgument(
        'line_spacing',
        default_value='0.45',
        description='Meters between lawnmower coverage rows',
    )
    waypoint_spacing_arg = DeclareLaunchArgument(
        'waypoint_spacing',
        default_value='0.75',
        description='Meters between waypoints along each coverage row',
    )
    wall_clearance_arg = DeclareLaunchArgument(
        'wall_clearance',
        default_value='0.30',
        description='Meters to stay away from occupied or unknown cells',
    )
    edge_passes_arg = DeclareLaunchArgument(
        'edge_passes',
        default_value='1',
        description='Number of edge-following passes before interior sweeps',
    )
    edge_stepover_arg = DeclareLaunchArgument(
        'edge_stepover',
        default_value='0.80',
        description='Meters between edge-following passes',
    )
    edge_band_arg = DeclareLaunchArgument(
        'edge_band',
        default_value='0.10',
        description='Thickness of each edge-following pass in meters',
    )
    interior_sweep_edge_clearance_arg = DeclareLaunchArgument(
        'interior_sweep_edge_clearance',
        default_value='0.45',
        description='Meters inward from edge pass before interior sweeps may begin',
    )
    max_waypoints_arg = DeclareLaunchArgument(
        'max_waypoints',
        default_value='250',
        description='Maximum waypoints to send; set 0 for no limit',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz from the underlying navigation bringup',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('generic_motor_driver'),
            'rviz',
            'cleaning.rviz',
        ]),
        description='Path to RViz config file',
    )

    robot_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('generic_motor_driver'),
                'launch',
                'complete_robot_navigation.launch.py',
            ])
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    coverage_cleaner = Node(
        package='generic_motor_driver',
        executable='coverage_cleaner.py',
        name='coverage_cleaner',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'auto_start': ParameterValue(LaunchConfiguration('start_cleaning'), value_type=bool),
            'preview_only': ParameterValue(LaunchConfiguration('preview_only'), value_type=bool),
            'align_sweeps_to_cell_axis': ParameterValue(
                LaunchConfiguration('align_sweeps_to_cell_axis'),
                value_type=bool,
            ),
            'snap_sweeps_to_wall_axes': ParameterValue(
                LaunchConfiguration('snap_sweeps_to_wall_axes'),
                value_type=bool,
            ),
            'wall_axis_override_deg': ParameterValue(
                LaunchConfiguration('wall_axis_override_deg'),
                value_type=float,
            ),
            'start_x': ParameterValue(LaunchConfiguration('start_x'), value_type=float),
            'start_y': ParameterValue(LaunchConfiguration('start_y'), value_type=float),
            'start_yaw': ParameterValue(LaunchConfiguration('start_yaw'), value_type=float),
            'line_spacing': ParameterValue(LaunchConfiguration('line_spacing'), value_type=float),
            'waypoint_spacing': ParameterValue(LaunchConfiguration('waypoint_spacing'), value_type=float),
            'wall_clearance': ParameterValue(LaunchConfiguration('wall_clearance'), value_type=float),
            'edge_passes': ParameterValue(LaunchConfiguration('edge_passes'), value_type=int),
            'edge_stepover': ParameterValue(LaunchConfiguration('edge_stepover'), value_type=float),
            'edge_band': ParameterValue(LaunchConfiguration('edge_band'), value_type=float),
            'interior_sweep_edge_clearance': ParameterValue(
                LaunchConfiguration('interior_sweep_edge_clearance'),
                value_type=float,
            ),
            'max_waypoints': ParameterValue(LaunchConfiguration('max_waypoints'), value_type=int),
        }],
    )

    return LaunchDescription([
        map_name_arg,
        map_file_arg,
        start_x_arg,
        start_y_arg,
        start_yaw_arg,
        start_cleaning_arg,
        align_sweeps_to_cell_axis_arg,
        snap_sweeps_to_wall_axes_arg,
        wall_axis_override_deg_arg,
        preview_only_arg,
        line_spacing_arg,
        waypoint_spacing_arg,
        wall_clearance_arg,
        edge_passes_arg,
        edge_stepover_arg,
        edge_band_arg,
        interior_sweep_edge_clearance_arg,
        max_waypoints_arg,
        use_sim_time_arg,
        rviz_arg,
        rviz_config_arg,
        robot_navigation,
        coverage_cleaner,
    ])
