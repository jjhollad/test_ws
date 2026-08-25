#!/usr/bin/env python3

"""Run the rectangular sweeper in Gazebo with localization, Nav2, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
import launch
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
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


class PersistedReplaceString(launch.Substitution):
    """Replace strings in a YAML file and persist the exact generated file."""

    def __init__(self, source_file, replacements, output_file):
        super().__init__()
        self._source_file = normalize_to_list_of_substitutions(source_file)
        self._output_file = normalize_to_list_of_substitutions(output_file)
        self._replacements = {
            key: normalize_to_list_of_substitutions(value)
            for key, value in replacements.items()
        }

    def describe(self):
        return ""

    def perform(self, context):
        source_path = perform_substitutions(context, self._source_file)
        output_path = perform_substitutions(context, self._output_file)
        replacements = {
            key: perform_substitutions(context, value)
            for key, value in self._replacements.items()
        }
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        temporary_path = f"{output_path}.tmp"
        with open(source_path, "r", encoding="utf-8") as source:
            with open(temporary_path, "w", encoding="utf-8") as output:
                for line in source:
                    for key, value in replacements.items():
                        line = line.replace(key, value)
                    output.write(line)
        os.replace(temporary_path, output_path)
        return output_path


class ConditionalText(launch.Substitution):
    """Return text when a boolean launch configuration is enabled."""

    def __init__(self, launch_configuration, when_true, when_false=""):
        super().__init__()
        self._launch_configuration = launch_configuration
        self._when_true = when_true
        self._when_false = when_false

    def describe(self):
        return ""

    def perform(self, context):
        value = LaunchConfiguration(self._launch_configuration).perform(context)
        if value.lower() in ("1", "true", "yes", "on"):
            return self._when_true
        return self._when_false


def generate_launch_description():
    demo_share = get_package_share_directory("create2_demo")
    gazebo_share = get_package_share_directory("gazebo_ros")
    nav2_share = get_package_share_directory("nav2_bringup")
    driver_share = get_package_share_directory("generic_motor_driver")
    slam_share = get_package_share_directory("slam_toolbox")
    model = os.path.join(demo_share, "urdf", "rectangular_robot.urdf.xacro")
    default_world = os.path.join(demo_share, "worlds", "corridor_museum.world")
    default_map = os.path.join(demo_share, "maps", "Sim_Map.yaml")
    robot_description = ParameterValue(Command(["xacro ", model]), value_type=str)
    # Use the workspace Nav2 configuration as the single tuning source. The
    # launch-time substitutions below are runtime bindings for simulation:
    # enable Gazebo time, use the finite clearing scan for costmaps, substitute
    # the GUI body-clearance value, and append SLAM Toolbox parameters only
    # when an explicit mapping launch requests SLAM.
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
            # Give Nav2 enough map->odom lead to survive controller loop jitter
            # during MPPI turn segments. scan_clearer independently bounds
            # corrected scan stamps to the current ROS clock, preventing sensor
            # data itself from being published in the future. Keep the queue
            # short so a timing discontinuity cannot replay obsolete laser data.
            "transform_timeout: 0.2",
            "transform_timeout: 0.30\n    scan_queue_size: 5",
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

    tuned_nav2_params = os.path.join(
        driver_share, "config", "nav2_params.yaml"
    )
    nav2_params = PersistedReplaceString(
        source_file=tuned_nav2_params,
        output_file=os.environ.get(
            "ROBOT_RUN_MANAGER_ACTIVE_NAV2_PARAMS",
            os.path.join(
                os.path.expanduser("~"),
                ".ros",
                "robot_run_manager",
                "active_nav2_params.yaml",
            ),
        ),
        replacements={
            "use_sim_time: False": "use_sim_time: True",
            "use_sim_time: false": "use_sim_time: true",
            'footprint: "[[1.05, 0.35], [1.05, -0.45], '
            '[-0.02, -0.45], [-0.02, 0.35]]"': (
                'footprint: "[[1.05, 0.35], [1.05, -0.45], '
                '[-0.02, -0.45], [-0.02, 0.35]]"\n'
                '      footprint_padding: __NAV2_BODY_CLEARANCE__'
            ),
            # Costmap clearing uses the finite-ray stream.  RViz continues to
            # display the genuine /scan stream, where no-returns remain inf.
            "          topic: /scan": "          topic: /scan_clear",
            "    velocity_timeout: 1.0": [
                "    velocity_timeout: 1.0",
                ConditionalText(
                    "use_slam",
                    "\n\n" + slam_parameters.rstrip(),
                ),
            ],
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
    use_slam = LaunchConfiguration("use_slam")
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
            condition=IfCondition(use_slam),
            launch_arguments={
                "params_file": nav2_params,
                "use_sim_time": "True",
                "autostart": "False",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "localization_launch.py")
            ),
            condition=UnlessCondition(use_slam),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": nav2_params,
                "use_sim_time": "True",
                "autostart": "True",
                "use_composition": "False",
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
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("use_slam", default_value="False"),
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
        # base_footprint before localization/Nav2 create their lifecycle nodes.
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
