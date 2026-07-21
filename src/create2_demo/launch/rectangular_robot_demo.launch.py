#!/usr/bin/env python3

"""Run the rectangular sweeper in Gazebo with SLAM, Nav2, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    demo_share = get_package_share_directory("create2_demo")
    gazebo_share = get_package_share_directory("gazebo_ros")
    nav2_share = get_package_share_directory("nav2_bringup")
    driver_share = get_package_share_directory("generic_motor_driver")
    slam_share = get_package_share_directory("slam_toolbox")
    model = os.path.join(demo_share, "urdf", "rectangular_robot.urdf.xacro")
    default_world = os.path.join(demo_share, "worlds", "corridor_museum.world")
    robot_description = ParameterValue(Command(["xacro ", model]), value_type=str)
    # Keep the stock Humble Nav2 configuration, but replace TurtleBot geometry
    # and dynamics with the measured rectangular sweeper values. The footprint
    # is relative to base_footprint at the rear axle: 1.05 m forward, 0.02 m
    # behind, 0.35 m left, and 0.45 m right (including the side brush).
    # Add SLAM Toolbox's standard mapping parameters to the common Nav2 file.
    # The synthetic no-return rays are 11.95 m; Karto treats readings at or
    # beyond its 11.90 m threshold as clearing rays with no occupied endpoint.
    with open(
        os.path.join(slam_share, "config", "mapper_params_online_sync.yaml"),
        encoding="utf-8",
    ) as slam_file:
        slam_parameters = slam_file.read().replace(
            "max_laser_range: 20.0", "max_laser_range: 11.90"
        ).replace("map_update_interval: 5.0", "map_update_interval: 1.0").replace(
            "scan_topic: /scan", "scan_topic: /scan_clear"
        ).replace(
            "min_laser_range: 0.0", "min_laser_range: 0.12"
        ).replace(
            # SLAM updates map->odom at the 5 Hz scan rate, so an unadjusted
            # transform can already be 200 ms old when Nav2 consumes it. Use
            # half a scan period of lead to keep it within Nav2's 200 ms
            # tolerance. scan_clearer independently bounds corrected scan
            # stamps to the current ROS clock, preventing sensor data itself
            # from being published in the future. Keep the queue short so a
            # timing discontinuity cannot replay obsolete laser data.
            "transform_timeout: 0.2",
            "transform_timeout: 0.10\n    scan_queue_size: 5",
        ).replace(
            # Long loop-closure solves can pause map->odom updates while
            # Gazebo odometry continues.  Retain enough correctly stamped TF
            # history to join the two sides of the chain after that pause.
            "tf_buffer_duration: 30.", "tf_buffer_duration: 120."
        ).replace(
            "map_update_interval: 1.0",
            "map_update_interval: __SLAM_MAP_UPDATE_INTERVAL__",
        ).replace(
            "    resolution: 0.05\n",
            "    resolution: __SLAM_RESOLUTION__\n",
        ).replace(
            "minimum_travel_distance: 0.5",
            "minimum_travel_distance: __SLAM_MINIMUM_TRAVEL_DISTANCE__",
        ).replace(
            "minimum_travel_heading: 0.5",
            "minimum_travel_heading: __SLAM_MINIMUM_TRAVEL_HEADING__",
        ).replace(
            "scan_buffer_size: 10",
            "scan_buffer_size: __SLAM_SCAN_BUFFER_SIZE__",
        ).replace(
            "scan_buffer_maximum_scan_distance: 10.0",
            "scan_buffer_maximum_scan_distance: __SLAM_SCAN_BUFFER_DISTANCE__",
        ).replace(
            "link_match_minimum_response_fine: 0.1",
            "link_match_minimum_response_fine: __SLAM_LINK_MATCH_RESPONSE__",
        ).replace(
            "link_scan_maximum_distance: 1.5",
            "link_scan_maximum_distance: __SLAM_LINK_SCAN_DISTANCE__",
        ).replace(
            "loop_search_maximum_distance: 3.0",
            "loop_search_maximum_distance: __SLAM_LOOP_SEARCH_DISTANCE__",
        ).replace(
            # Long, similar corridors are unusually prone to a geometrically
            # plausible but incorrect loop closure.  Require more supporting
            # scans and stronger coarse/fine agreement before moving the map.
            "loop_match_minimum_chain_size: 10",
            "loop_match_minimum_chain_size: 15",
        ).replace(
            "loop_match_minimum_response_coarse: 0.35",
            "loop_match_minimum_response_coarse: 0.45",
        ).replace(
            "loop_match_minimum_response_fine: 0.45",
            "loop_match_minimum_response_fine: 0.55",
        )

    nav2_params = ReplaceString(
        source_file=os.path.join(nav2_share, "params", "nav2_params.yaml"),
        replacements={
            "robot_base_frame: base_link": "robot_base_frame: base_footprint",
            "robot_radius: 0.22": (
                'footprint: "[[1.05, 0.35], [1.05, -0.45], '
                '[-0.02, -0.45], [-0.02, 0.35]]"\n'
                '      footprint_padding: __NAV2_BODY_CLEARANCE__'
            ),
            # The stock TurtleBot inflation field falls almost to free-space
            # cost before this long rectangular chassis has room to turn.
            # Keep a useful wall gradient for a full metre around obstacles;
            # the footprint padding below remains the hard collision margin.
            "        inflation_radius: 0.55": (
                "        inflation_radius: 1.00"
            ),
            "max_vel_x: 0.26": "max_vel_x: 0.30",
            "bt_loop_duration: 10": "bt_loop_duration: 50",
            "expected_planner_frequency: 20.0": "expected_planner_frequency: 5.0",
            "max_vel_theta: 1.0": "max_vel_theta: 0.60",
            "max_speed_xy: 0.26": "max_speed_xy: 0.30",
            "acc_lim_x: 2.5": "acc_lim_x: 0.30",
            "acc_lim_theta: 3.2": "acc_lim_theta: 0.60",
            "decel_lim_x: -2.5": "decel_lim_x: -0.40",
            "decel_lim_theta: -3.2": "decel_lim_theta: -0.70",
            # The collision-aware two-length backup needs the full recovery
            # trajectory to remain inside the rolling local costmap.
            "      width: 3": "      width: 6",
            "      height: 3": "      height: 6",
            "      track_unknown_space: true": "      track_unknown_space: false",
            "          raytrace_max_range: 3.0": "          raytrace_max_range: 12.0",
            "          obstacle_max_range: 2.5": "          obstacle_max_range: 11.5",
            # Costmap clearing uses the finite-ray stream.  RViz continues to
            # display the genuine /scan stream, where no-returns remain inf.
            "          topic: /scan": "          topic: /scan_clear",
            # Retain the last valid obstacle briefly across a dropped scan,
            # but mark the observation source stale after three missed 10 Hz
            # updates so Nav2 stops rather than driving on old perception.
            '          data_type: "LaserScan"': (
                '          data_type: "LaserScan"\n'
                '          observation_persistence: 0.50\n'
                '          expected_update_rate: 0.45'
            ),
            # This is the final stock Nav2 parameter, so it is a stable point
            # at which to append the separate slam_toolbox node section.
            "    velocity_timeout: 1.0": (
                "    velocity_timeout: 1.0\n\n" + slam_parameters.rstrip()
            ),
            "__SLAM_MAP_UPDATE_INTERVAL__": LaunchConfiguration(
                "slam_map_update_interval"
            ),
            "__SLAM_RESOLUTION__": LaunchConfiguration("slam_resolution"),
            "__SLAM_MINIMUM_TRAVEL_DISTANCE__": LaunchConfiguration(
                "slam_minimum_travel_distance"
            ),
            "__SLAM_MINIMUM_TRAVEL_HEADING__": LaunchConfiguration(
                "slam_minimum_travel_heading"
            ),
            "__SLAM_SCAN_BUFFER_SIZE__": LaunchConfiguration(
                "slam_scan_buffer_size"
            ),
            "__SLAM_SCAN_BUFFER_DISTANCE__": LaunchConfiguration(
                "slam_scan_buffer_distance"
            ),
            "__SLAM_LINK_MATCH_RESPONSE__": LaunchConfiguration(
                "slam_link_match_response"
            ),
            "__SLAM_LINK_SCAN_DISTANCE__": LaunchConfiguration(
                "slam_link_scan_distance"
            ),
            "__SLAM_LOOP_SEARCH_DISTANCE__": LaunchConfiguration(
                "slam_loop_search_distance"
            ),
            "__NAV2_BODY_CLEARANCE__": LaunchConfiguration(
                "nav2_body_clearance"
            ),
            # Exploration goals are positional. Accept any final orientation so
            # DWB does not spin in place trying to match a frontier tangent.
            "      stateful: True": "      stateful: False",
            "      xy_goal_tolerance: 0.25": "      xy_goal_tolerance: 0.35",
            "      yaw_goal_tolerance: 0.25": "      yaw_goal_tolerance: 3.14",
            "max_velocity: [0.26, 0.0, 1.0]": "max_velocity: [0.30, 0.0, 0.60]",
            "min_velocity: [-0.26, 0.0, -1.0]": "min_velocity: [-0.20, 0.0, -0.60]",
            "max_accel: [2.5, 0.0, 3.2]": "max_accel: [0.30, 0.0, 0.60]",
            "max_decel: [-2.5, 0.0, -3.2]": "max_decel: [-0.40, 0.0, -0.70]",
        },
    )
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    wall_distance = LaunchConfiguration("wall_distance")
    inside_corner_clearance = LaunchConfiguration("inside_corner_clearance")
    wall_speed = LaunchConfiguration("wall_speed")
    wall_evaluation = LaunchConfiguration("wall_evaluation")
    wall_cooldown = LaunchConfiguration("wall_cooldown")
    start_wall_follower = LaunchConfiguration("start_wall_follower")
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "rectangular_sweeper", "-topic", "robot_description",
            "-x", "-2.0", "-y", "-0.5", "-z", "0.01",
        ],
        output="screen",
    )
    nav2_bringup = GroupAction(actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "slam_launch.py")
            ),
            launch_arguments={
                "params_file": nav2_params,
                "use_sim_time": "True",
                "autostart": "False",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                driver_share, "launch", "navigation_launch_mux.launch.py"
            )),
            launch_arguments={
                "params_file": nav2_params,
                "use_sim_time": "True",
                "autostart": "False",
                "use_composition": "False",
            }.items(),
        ),
    ])
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_sim_time": "True",
            "rviz_config": os.path.join(
                demo_share, "rviz", "wandering_mapping.rviz"
            ),
        }.items(),
    )
    navigation_activator = Node(
        package="create2_demo",
        executable="activate_navigation.py",
        name="navigation_activator",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="False"),
        # A 0.20 m hard margin keeps the 0.80 m-wide body out of corridors
        # narrower than the separately configured 1.20 m safety threshold.
        DeclareLaunchArgument("nav2_body_clearance", default_value="0.20"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("wall_distance", default_value="1.00"),
        DeclareLaunchArgument("inside_corner_clearance", default_value="1.00"),
        DeclareLaunchArgument("wall_speed", default_value="0.28"),
        DeclareLaunchArgument("wall_evaluation", default_value="25.0"),
        DeclareLaunchArgument("wall_cooldown", default_value="300.0"),
        DeclareLaunchArgument("minimum_corridor_width", default_value="1.20"),
        DeclareLaunchArgument(
            "narrow_corridor_confirmation_time", default_value="0.30"
        ),
        DeclareLaunchArgument("start_wall_follower", default_value="true"),
        DeclareLaunchArgument("slam_map_update_interval", default_value="1.0"),
        DeclareLaunchArgument("slam_resolution", default_value="0.05"),
        DeclareLaunchArgument("slam_minimum_travel_distance", default_value="0.30"),
        DeclareLaunchArgument("slam_minimum_travel_heading", default_value="0.10"),
        DeclareLaunchArgument("slam_scan_buffer_size", default_value="10"),
        DeclareLaunchArgument("slam_scan_buffer_distance", default_value="8.0"),
        DeclareLaunchArgument("slam_link_match_response", default_value="0.20"),
        DeclareLaunchArgument("slam_link_scan_distance", default_value="2.0"),
        DeclareLaunchArgument("slam_loop_search_distance", default_value="3.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "gzserver.launch.py")
            ),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "gzclient.launch.py")
            ),
            condition=UnlessCondition(headless),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="create2_demo",
            executable="scan_clearer.py",
            name="scan_clearer",
            parameters=[{
                "use_sim_time": True,
                "stamp_offset_seconds": 0.05,
            }],
            output="screen",
        ),
        Node(
            package="create2_demo",
            executable="contact_monitor.py",
            name="contact_monitor",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="create2_demo",
            executable="exploration_coordinator.py",
            name="exploration_coordinator",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="create2_demo",
            executable="clockwise_wall_tracer.py",
            name="clockwise_wall_tracer",
            condition=IfCondition(start_wall_follower),
            parameters=[{
                "use_sim_time": True,
                "target_wall_distance": ParameterValue(wall_distance, value_type=float),
                "inside_corner_front_clearance": ParameterValue(
                    inside_corner_clearance, value_type=float
                ),
                "linear_speed": ParameterValue(wall_speed, value_type=float),
                "exploration_evaluation_period": ParameterValue(
                    wall_evaluation, value_type=float
                ),
                "global_planning_cooldown": ParameterValue(
                    wall_cooldown, value_type=float
                ),
                "minimum_corridor_width": ParameterValue(
                    LaunchConfiguration("minimum_corridor_width"),
                    value_type=float,
                ),
                "narrow_corridor_confirmation_time": ParameterValue(
                    LaunchConfiguration("narrow_corridor_confirmation_time"),
                    value_type=float,
                ),
            }],
            output="screen",
        ),
        spawn_robot,
        # Gazebo must publish /clock and the robot must provide odom ->
        # base_footprint before SLAM/Nav2 create their lifecycle nodes.
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    TimerAction(
                        period=1.0,
                        actions=[nav2_bringup],
                    ),
                    TimerAction(
                        period=6.0,
                        actions=[navigation_activator],
                    ),
                ],
            )
        ),
        # RViz's fixed frame is map. Starting it only after lifecycle
        # activation prevents early laser scans from waiting until they are
        # older than TF's cache.
        RegisterEventHandler(
            OnProcessExit(
                target_action=navigation_activator,
                on_exit=[rviz],
            )
        ),
    ])
