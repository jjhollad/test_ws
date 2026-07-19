#!/usr/bin/env python3

"""Run the rectangular sweeper in Gazebo with SLAM, Nav2, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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
        )

    nav2_params = ReplaceString(
        source_file=os.path.join(nav2_share, "params", "nav2_params.yaml"),
        replacements={
            "robot_base_frame: base_link": "robot_base_frame: base_footprint",
            "robot_radius: 0.22": (
                'footprint: "[[1.05, 0.35], [1.05, -0.45], '
                '[-0.02, -0.45], [-0.02, 0.35]]"'
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
    wall_speed = LaunchConfiguration("wall_speed")
    wall_evaluation = LaunchConfiguration("wall_evaluation")
    wall_cooldown = LaunchConfiguration("wall_cooldown")
    start_wall_follower = LaunchConfiguration("start_wall_follower")
    simulation_world = ReplaceString(
        source_file=LaunchConfiguration("world"),
        replacements={
            "<real_time_update_rate>1000</real_time_update_rate>": [
                "<real_time_update_rate>",
                LaunchConfiguration("sim_real_time_update_rate"),
                "</real_time_update_rate>",
            ],
        },
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "rectangular_sweeper", "-topic", "robot_description",
            "-x", "-2.0", "-y", "-0.5", "-z", "0.01",
        ],
        output="screen",
    )
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "slam": "True",
            # Humble requires this argument even when SLAM ignores it.
            "map": os.path.join(nav2_share, "maps", "turtlebot3_world.yaml"),
            "params_file": nav2_params,
            "use_sim_time": "True",
            # SLAM and navigation are activated in order below. Controller
            # configuration fails if it races SLAM's map -> odom transform.
            "autostart": "False",
            "use_composition": "False",
        }.items(),
    )
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

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="False"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("wall_distance", default_value="1.05"),
        DeclareLaunchArgument("wall_speed", default_value="0.28"),
        DeclareLaunchArgument("wall_evaluation", default_value="25.0"),
        DeclareLaunchArgument("wall_cooldown", default_value="300.0"),
        DeclareLaunchArgument("start_wall_follower", default_value="true"),
        DeclareLaunchArgument("sim_real_time_update_rate", default_value="1000"),
        DeclareLaunchArgument("slam_map_update_interval", default_value="0.5"),
        DeclareLaunchArgument("slam_resolution", default_value="0.05"),
        DeclareLaunchArgument("slam_minimum_travel_distance", default_value="0.15"),
        DeclareLaunchArgument("slam_minimum_travel_heading", default_value="0.10"),
        DeclareLaunchArgument("slam_scan_buffer_size", default_value="15"),
        DeclareLaunchArgument("slam_scan_buffer_distance", default_value="12.0"),
        DeclareLaunchArgument("slam_link_match_response", default_value="0.15"),
        DeclareLaunchArgument("slam_link_scan_distance", default_value="2.0"),
        DeclareLaunchArgument("slam_loop_search_distance", default_value="4.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "gzserver.launch.py")
            ),
            launch_arguments={"world": simulation_world}.items(),
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
                "linear_speed": ParameterValue(wall_speed, value_type=float),
                "exploration_evaluation_period": ParameterValue(
                    wall_evaluation, value_type=float
                ),
                "global_planning_cooldown": ParameterValue(
                    wall_cooldown, value_type=float
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
                    nav2_bringup,
                    rviz,
                    TimerAction(
                        period=5.0,
                        actions=[Node(
                            package="create2_demo",
                            executable="activate_navigation.py",
                            name="navigation_activator",
                            output="screen",
                        )],
                    ),
                ],
            )
        ),
    ])
