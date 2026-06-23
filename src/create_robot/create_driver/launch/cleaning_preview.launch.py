#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


DEFAULT_MAP_PATH = '/home/user/maps/EERCsB/EERCsBsTRAIT.yaml'
DEFAULT_START_X = '0.22198301553726196'
DEFAULT_START_Y = '-0.06558487564325333'
DEFAULT_START_YAW = '0.0'


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=DEFAULT_MAP_PATH,
        description='Full path to the saved map yaml file to preview',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )
    coverage_algorithm_arg = DeclareLaunchArgument(
        'coverage_algorithm',
        default_value='center_loop',
        description='Coverage algorithm: center_loop, boustrophedon, or lawnmower',
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
    line_spacing_arg = DeclareLaunchArgument(
        'line_spacing',
        default_value='0.75',
        description='Meters between coverage rows',
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
        default_value='0.65',
        description='Meters inward from edge pass before interior sweeps may begin',
    )
    preview_line_width_arg = DeclareLaunchArgument(
        'preview_line_width',
        default_value='0.08',
        description='RViz preview line width in meters',
    )
    preview_dot_size_arg = DeclareLaunchArgument(
        'preview_dot_size',
        default_value='0.08',
        description='RViz preview waypoint dot diameter in meters',
    )
    preview_arrow_stride_arg = DeclareLaunchArgument(
        'preview_arrow_stride',
        default_value='10',
        description='Draw one direction arrow every N path segments',
    )
    preview_label_stride_arg = DeclareLaunchArgument(
        'preview_label_stride',
        default_value='25',
        description='Draw one step number every N waypoints; set 0 to disable',
    )
    preview_text_height_arg = DeclareLaunchArgument(
        'preview_text_height',
        default_value='0.35',
        description='RViz preview step number text height in meters',
    )
    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value=DEFAULT_START_X,
        description='Starting X position used to select the reachable cleaning region',
    )
    start_y_arg = DeclareLaunchArgument(
        'start_y',
        default_value=DEFAULT_START_Y,
        description='Starting Y position used to select the reachable cleaning region',
    )
    start_yaw_arg = DeclareLaunchArgument(
        'start_yaw',
        default_value=DEFAULT_START_YAW,
        description='Starting yaw in radians',
    )
    restrict_to_start_region_arg = DeclareLaunchArgument(
        'restrict_to_start_region',
        default_value='true',
        description='Only preview cells connected to the configured start pose',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'yaml_filename': LaunchConfiguration('map'),
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_preview',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    coverage_cleaner = Node(
        package='generic_motor_driver',
        executable='coverage_cleaner.py',
        name='coverage_cleaner',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'auto_start': True,
            'preview_only': True,
            'coverage_algorithm': LaunchConfiguration('coverage_algorithm'),
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
            'publish_continuous_path': False,
            'preview_republish_period': 2.0,
            'publish_initial_pose': False,
            'startup_delay': 2.0,
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
            'preview_line_width': ParameterValue(
                LaunchConfiguration('preview_line_width'),
                value_type=float,
            ),
            'preview_dot_size': ParameterValue(
                LaunchConfiguration('preview_dot_size'),
                value_type=float,
            ),
            'preview_arrow_stride': ParameterValue(
                LaunchConfiguration('preview_arrow_stride'),
                value_type=int,
            ),
            'preview_label_stride': ParameterValue(
                LaunchConfiguration('preview_label_stride'),
                value_type=int,
            ),
            'preview_text_height': ParameterValue(
                LaunchConfiguration('preview_text_height'),
                value_type=float,
            ),
            'restrict_to_start_region': ParameterValue(
                LaunchConfiguration('restrict_to_start_region'),
                value_type=bool,
            ),
            'max_waypoints': 0,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('generic_motor_driver'),
                'rviz',
                'cleaning.rviz',
            ]),
        ],
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        coverage_algorithm_arg,
        align_sweeps_to_cell_axis_arg,
        snap_sweeps_to_wall_axes_arg,
        wall_axis_override_deg_arg,
        line_spacing_arg,
        waypoint_spacing_arg,
        wall_clearance_arg,
        edge_passes_arg,
        edge_stepover_arg,
        edge_band_arg,
        interior_sweep_edge_clearance_arg,
        preview_line_width_arg,
        preview_dot_size_arg,
        preview_arrow_stride_arg,
        preview_label_stride_arg,
        preview_text_height_arg,
        start_x_arg,
        start_y_arg,
        start_yaw_arg,
        restrict_to_start_region_arg,
        map_server,
        lifecycle_manager,
        coverage_cleaner,
        rviz,
    ])
