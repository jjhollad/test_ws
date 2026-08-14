# Copyright 2026 jjhollad
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from PyQt5.QtCore import QProcess, QProcessEnvironment, QSettings, QTimer, Qt
from PyQt5.QtGui import QColor, QTextCursor
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSlider,
    QStatusBar,
    QTabWidget,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)
import rclpy

from robot_run_manager.behavior_tree import SupervisorTree


ACTION_GROUPS = {
    'Safety and system': [
        'Preflight Check',
        'Start Robot',
        'Stop Robot',
        'Start Xbox Teleop',
        'Stop Xbox Teleop',
        'Emergency Stop',
        'Kill ROS Processes',
        'Open RViz',
        'Open Gazebo',
    ],
    'Collect real runs': [
        'Start Recording',
        'Stop Recording',
        'Mark Goal',
        'Mark Collision',
        'Mark Near Miss',
        'Mark Intervention',
    ],
    'Mapping and simulation': [
        'Start Mapping',
        'Stop Mapping',
        'Save Map',
        'Start Autonomous Exploration',
        'Pause Exploration',
        'Resume Exploration',
        'Stop Autonomous Exploration',
        'Exploration E-Stop',
        'Clear Exploration E-Stop',
        'Start Wall Follower',
        'Stop Wall Follower',
        'Start Simulation',
        'Stop Simulation',
        'Start Coverage',
        'Stop Coverage',
        'Replay Selected Run',
    ],
    'Data and code': [
        'Validate Selected Run',
        'Transfer Run',
        'Download Code',
        'Upload Code',
        'View Logs',
        'Flag & Open Logs',
        'Rebuild & Restart Manager',
        'Install Desktop Launcher',
    ],
}

ACTION_TOOLTIPS = {
    'Preflight Check': 'Check ROS, workspace, hardware, display, and disk readiness.',
    'Start Robot': 'Start the guarded real-robot motor, relay, and state-publisher stack.',
    'Stop Robot': 'Publish zero velocity and stop the GUI-managed real-robot stack.',
    'Start Xbox Teleop': (
        'Launch Xbox joystick input and teleop_twist_joy control for the '
        'running robot.'
    ),
    'Stop Xbox Teleop': (
        'Publish zero velocity and stop Xbox joystick teleoperation.'
    ),
    'Emergency Stop': 'Immediately publish repeated zero velocity and stop managed processes.',
    'Kill ROS Processes': 'Force-kill stale ROS, Gazebo, RViz, Nav2, SLAM, and driver processes.',
    'Open RViz': 'Open RViz for robot, sensor, map, and navigation visualization.',
    'Open Gazebo': 'Open or restore the Gazebo window for the active simulation server.',
    'Start Recording': 'Create a timestamped run and record configured ROS topics.',
    'Stop Recording': 'Cleanly finalize the active rosbag and run metadata.',
    'Mark Goal': 'Timestamp a successful goal event in the active run.',
    'Mark Collision': 'Timestamp a collision event in the active run.',
    'Mark Near Miss': 'Timestamp a near-miss event in the active run.',
    'Mark Intervention': 'Timestamp an operator-intervention event in the active run.',
    'Start Mapping': 'Start supervised real-robot SLAM, Nav2, LiDAR, joystick, and RViz.',
    'Stop Mapping': 'Stop autonomous behavior, publish zero velocity, and stop real mapping.',
    'Save Map': 'Save the active SLAM occupancy map as YAML and PGM files.',
    'Start Autonomous Exploration': (
        'Start SLAM, right-wall mapping, progress checking, and Nav2 frontier '
        'exploration on the selected real robot or local simulation.'
    ),
    'Pause Exploration': 'Cancel autonomous motion while keeping the exploration stack available.',
    'Resume Exploration': 'Clear the paused state and resume with readiness checks.',
    'Stop Autonomous Exploration': (
        'Stop autonomous behavior, publish zero velocity, and close its complete stack.'
    ),
    'Exploration E-Stop': (
        'Latch the autonomous emergency stop and cancel wall-following and Nav2 motion.'
    ),
    'Clear Exploration E-Stop': (
        'Clear the autonomous stop; readiness checks run again before motion resumes.'
    ),
    'Start Wall Follower': (
        'Start wall-first behavior-tree exploration. Wall tracing remains primary; '
        'stagnation, loops, or retracing trigger nearest-frontier relocation.'
    ),
    'Stop Wall Follower': (
        'Stop behavior-tree wall exploration and its frontier planner.'
    ),
    'Start Simulation': (
        'Start the corridor Gazebo world, rectangular robot, SLAM, Nav2, and RViz.'
    ),
    'Stop Simulation': 'Stop simulation, Gazebo window, replay, and autonomous behaviors.',
    'Start Coverage': (
        'Plan the configured field with OpenNav Coverage and execute its path '
        'through the existing Nav2 controller.'
    ),
    'Stop Coverage': 'Cancel coverage motion and publish zero velocity.',
    'Replay Selected Run': 'Replay a selected rosbag on the isolated development ROS graph.',
    'Validate Selected Run': 'Validate files and create a summary, dataset split, and checksums.',
    'Transfer Run': 'Upload or download a run using resumable SSH/rsync transfer.',
    'Download Code': 'Pull code with Git fast-forward only; requires a clean working tree.',
    'Upload Code': 'Commit only this utility and .gitignore, then push to GitHub.',
    'View Logs': 'Open the workspace ROS build and runtime log directory.',
    'Flag & Open Logs': (
        'Add a timestamped identifier to the log index and open recent logs '
        'in a terminal reader.'
    ),
    'Rebuild & Restart Manager': (
        'Save settings, build the complete workspace in a terminal, and '
        'automatically relaunch this Manager with the new code.'
    ),
    'Install Desktop Launcher': 'Create or refresh the desktop icon and ROS startup wrapper.',
}

IMPLEMENTED_ACTIONS = {
    'Preflight Check',
    'Start Robot',
    'Stop Robot',
    'Start Xbox Teleop',
    'Stop Xbox Teleop',
    'Emergency Stop',
    'Kill ROS Processes',
    'Open RViz',
    'Open Gazebo',
    'Start Recording',
    'Stop Recording',
    'Mark Goal',
    'Mark Collision',
    'Mark Near Miss',
    'Mark Intervention',
    'Start Mapping',
    'Stop Mapping',
    'Save Map',
    'Start Simulation',
    'Stop Simulation',
    'Start Coverage',
    'Stop Coverage',
    'Replay Selected Run',
    'Validate Selected Run',
    'Transfer Run',
    'Download Code',
    'Upload Code',
    'View Logs',
    'Flag & Open Logs',
    'Rebuild & Restart Manager',
    'Install Desktop Launcher',
    'Start Autonomous Exploration',
    'Pause Exploration',
    'Resume Exploration',
    'Stop Autonomous Exploration',
    'Exploration E-Stop',
    'Clear Exploration E-Stop',
    'Start Wall Follower',
    'Stop Wall Follower',
}

HIDDEN_SUPERVISOR_EXPLORATION_ACTIONS = {
    'Start Autonomous Exploration',
    'Pause Exploration',
    'Resume Exploration',
    'Stop Autonomous Exploration',
    'Exploration E-Stop',
    'Clear Exploration E-Stop',
}

RECORD_TOPICS = [
    '/cmd_vel',
    '/odom',
    '/scan',
    '/scan_raw',
    '/scan_clear',
    '/clock',
    '/tf',
    '/tf_static',
    '/joint_states',
    '/relay_status',
    '/chassis_contacts',
    '/front_caster_contacts',
    '/left_rear_wheel_contacts',
    '/right_rear_wheel_contacts',
    '/physical_contact',
    '/contact_event',
    '/wall_behavior_state',
    '/frontier_handoff_requested',
    '/frontier_navigation_complete',
    '/exploration_coordination_state',
    '/opencv_hallway_mapping_driver/status',
    '/mapping_progress_monitor/status',
    '/frontier_selector/status',
    '/exploration_supervisor/status',
]

TUNING_VARIABLES = {
    'target_wall_distance': (
        'Minimum wall clearance', 'Wall follower', 0.10, 1.50, 1.00, 2,
        'Minimum distance from the closest part of the robot footprint to the '
        'right wall. Angled front and rear corners are included.'
    ),
    'inside_corner_front_clearance': (
        'Inside-corner front clearance', 'Wall follower',
        0.40, 2.00, 1.00, 2,
        'Closest forward-footprint distance from the wall ahead at which the '
        'inside-corner left turn begins.'
    ),
    'linear_speed': (
        'Wall speed', 'Wall follower', 0.05, 0.40, 0.28, 2,
        'Forward speed after Nav2 hands control to direct wall following.'
    ),
    'nav2_wall_follow_distance': (
        'Nav2 straight lead-in', 'Wall follower', 1.0, 6.0, 3.0, 1,
        'Distance Nav2 follows the fitted wall line before wall control takes over.'
    ),
    'wall_fit_inlier_distance': (
        'LiDAR line tolerance', 'Wall follower', 0.03, 0.25, 0.10, 2,
        'Maximum point-to-line error accepted by the robust wall fit.'
    ),
    'minimum_handoff_wall_length': (
        'Minimum handoff wall', 'Wall follower', 5.0, 10.0, 5.0, 1,
        'Minimum continuous LiDAR-fitted wall length required before wall '
        'following may take control from the global planner.'
    ),
    'global_planning_cooldown': (
        'Behavior handoff timeout', 'Behavior coordination',
        5.0, 600.0, 300.0, 1,
        'Maximum seconds allowed for the OpenCV planner to reach the nearest '
        'frontier after wall following stalls. Wall following resumes when '
        'the frontier is reached; this timeout is only the fallback.'
    ),
    'dead_end_priority_multiplier': (
        'Dead-end exit priority', 'Behavior coordination',
        0.0, 3.0, 1.2, 1,
        'Multiplies measured corridor-entry distance to reserve wall-follower '
        'control while turning around and tracing back out. Zero disables it.'
    ),
    'heading_gain': (
        'Wall heading correction', 'Wall tracing', 0.1, 4.0, 1.4, 2,
        'How strongly the robot turns to become parallel with the fitted wall.'
    ),
    'distance_gain': (
        'Wall distance correction', 'Wall tracing', 0.1, 3.0, 0.9, 2,
        'How strongly the robot steers toward or away from the target wall offset.'
    ),
    'turn_speed': (
        'Inside-corner turn speed', 'Corner handling', 0.10, 0.80, 0.45, 2,
        'Counterclockwise angular speed used at blocked inside corners and dead ends.'
    ),
    'front_stop_distance': (
        'Front body clearance', 'Corner handling', 0.10, 2.00, 0.50, 2,
        'Minimum distance from the closest footprint point to a front wall '
        'before straight tracing changes into an inside-corner turn.'
    ),
    'wall_timeout': (
        'Lost-wall memory', 'Behavior coordination', 0.2, 20.0, 10.0, 1,
        'How long the last valid right wall remains trusted before lost-wall search.'
    ),
    'wall_fit_max_range': (
        'Wall fitting range', 'Wall tracing', 2.0, 12.0, 8.0, 1,
        'Maximum LiDAR range included when fitting a right-side wall.'
    ),
    'wall_fit_minimum_span': (
        'Minimum fitted span', 'Wall tracing', 0.3, 3.0, 0.8, 1,
        'Shortest supported straight surface accepted as a wall.'
    ),
    'outside_corner_loss_margin': (
        'Right-edge loss margin', 'Corner handling', 0.20, 1.50, 0.65, 2,
        'Extra distance beyond the target offset that declares the direct-right wall absent.'
    ),
    'outside_corner_confirmation_time': (
        'Corner confirmation time', 'Corner handling', 0.10, 2.00, 0.35, 2,
        'How long the right wall must remain absent before committing to a corner.'
    ),
    'outside_corner_turn_angle': (
        '90° turn target', 'Corner handling', 1.20, 1.90, 1.57, 2,
        'Clockwise odometry angle required for the normal outside-corner quarter-turn.'
    ),
    'outside_corner_linear_speed': (
        'Corner arc speed', 'Corner handling', 0.00, 0.20, 0.07, 2,
        'Forward speed during a clockwise outside-corner arc; zero turns in place.'
    ),
    'outside_corner_turn_speed': (
        'Outside-corner turn speed', 'Corner handling', 0.10, 0.80, 0.38, 2,
        'Clockwise angular speed during right-corner and wall-reacquisition maneuvers.'
    ),
    'outside_corner_timeout': (
        'Corner maneuver timeout', 'Corner handling', 2.0, 20.0, 8.0, 1,
        'Maximum time allowed for a corner maneuver before entering reacquisition.'
    ),
    'revisited_right_turn_radius': (
        'Prior-path detection radius', 'Corner handling', 0.3, 3.0, 1.25, 2,
        'Distance from an older trace that marks an approach as previously traveled.'
    ),
    'revisited_right_turn_minimum_age': (
        'Prior-path minimum age', 'Corner handling', 5.0, 180.0, 20.0, 1,
        'Trace age required before it can activate right-turn preference.'
    ),
    'revisited_right_turn_clearance': (
        'Open-right body clearance', 'Corner handling', 0.1, 3.0, 0.50, 2,
        'Required clearance from the closest footprint point before preferring '
        'a revisited-path right turn.'
    ),
    'right_turn_commitment_time': (
        'Right-turn commitment', 'Corner handling', 0.2, 8.0, 2.0, 1,
        'Time after a clockwise turn when ordinary inside-left steering is suppressed.'
    ),
    'emergency_front_distance': (
        'Emergency body clearance', 'Corner handling', 0.05, 0.9, 0.15, 2,
        'Closest footprint-to-wall distance that overrides right-turn '
        'preference to prevent collision.'
    ),
    'handoff_heading_tolerance': (
        'Handoff heading tolerance', 'Behavior coordination', 0.05, 0.60, 0.17, 2,
        'Maximum wall-parallel heading error allowed when Nav2 hands off control.'
    ),
    'handoff_distance_tolerance': (
        'Handoff distance tolerance', 'Behavior coordination', 0.05, 0.60, 0.20, 2,
        'Maximum wall-offset error allowed when Nav2 hands off control.'
    ),
    'alignment_settle_time': (
        'Alignment settle time', 'Behavior coordination', 0.1, 3.0, 0.75, 2,
        'Stationary time before submitting the fitted straight wall-alignment route.'
    ),
    'alignment_escape_clearance': (
        'Pre-Nav2 escape clearance', 'Behavior coordination',
        0.10, 0.60, 0.20, 2,
        'Minimum closest-body wall clearance required before Nav2 alignment. '
        'Below it, direct control rotates away because Nav2 correctly rejects '
        'a padded footprint that already overlaps its costmap.'
    ),
    'exploration_evaluation_period': (
        'Wall discovery evaluation', 'Behavior coordination', 5.0, 120.0, 25.0, 1,
        'Time window used to judge whether wall following is still discovering map area.'
    ),
    'minimum_free_area_gain': (
        'Minimum wall discovery gain', 'Behavior coordination', 0.0, 10.0, 1.5, 1,
        'New free square metres required during a wall-tracing evaluation window.'
    ),
    'loop_return_radius': (
        'Repeated-location radius', 'Behavior coordination', 0.2, 3.0, 1.0, 1,
        'Distance that counts as returning to an already traced location.'
    ),
    'loop_minimum_age': (
        'Repeated-location minimum age', 'Behavior coordination', 5.0, 180.0, 30.0, 1,
        'Minimum age of a prior trace point before it can trigger a planner handoff.'
    ),
    'retrace_radius': (
        'Traveled-path match radius', 'Behavior coordination',
        0.10, 2.00, 0.60, 2,
        'Maximum distance from an older recorded track for a current track '
        'sample to count as previously traveled.'
    ),
    'retrace_minimum_age': (
        'Traveled-path minimum age', 'Behavior coordination',
        5.0, 300.0, 20.0, 1,
        'How old a track sample must be before it can prove that the robot is '
        'repeating a route rather than extending its current pass.'
    ),
    'retrace_minimum_length': (
        'Traveled-path check length', 'Behavior coordination',
        0.5, 10.0, 2.0, 1,
        'Length of the robot current track compared with older travel. Larger '
        'values avoid switching at a single crossing.'
    ),
    'retrace_overlap_ratio': (
        'Traveled-path overlap', 'Behavior coordination',
        0.10, 1.00, 0.70, 2,
        'Fraction of current-track samples that must match older travel before '
        'requesting the nearest-frontier global path.'
    ),
    'dead_end_minimum_depth': (
        'Minimum dead-end depth', 'Corner handling', 0.2, 5.0, 1.0, 1,
        'Minimum corridor travel before dead-end exit priority can activate.'
    ),
    'dead_end_side_max_distance': (
        'Dead-end side-wall range', 'Corner handling', 1.0, 8.0, 4.0, 1,
        'Maximum side range used to confirm that the blocked front is a corridor dead end.'
    ),
    'information_weight': (
        'Unknown-space reward', 'Global frontier planner', 0.0, 10.0, 4.0, 1,
        'Reward per square metre of unknown space visible at a frontier.'
    ),
    'frontier_bonus': (
        'Frontier bonus', 'Global frontier planner', 0.0, 20.0, 6.0, 1,
        'Fixed reward for choosing a reachable frontier instead of open-space travel.'
    ),
    'revisit_weight': (
        'Revisited-route penalty', 'Global frontier planner', 0.0, 10.0, 3.0, 1,
        'Penalty for each planned metre overlapping previously traveled territory.'
    ),
    'visited_radius': (
        'Visited corridor radius', 'Global frontier planner', 0.20, 2.00, 0.75, 2,
        'Radius around recorded travel that counts as already visited.'
    ),
    'travel_weight': (
        'Route-length penalty', 'Global frontier planner', 0.0, 2.0, 0.20, 2,
        'General cost for distant goals, independent of whether the route is new.'
    ),
    'forward_weight': (
        'Forward preference', 'Global frontier planner', 0.0, 8.0, 2.0, 1,
        'Reward for frontiers already aligned with the robot heading.'
    ),
    'reverse_penalty': (
        'Rear-goal penalty', 'Global frontier planner', 0.0, 15.0, 6.0, 1,
        'Discourages turning toward goals behind the robot when forward options exist.'
    ),
    'clearance_weight': (
        'Obstacle-clearance reward', 'Global frontier planner', 0.0, 12.0, 5.0, 1,
        'Makes A* favor route cells farther from obstacles; high values can add detours.'
    ),
    'route_extension_period': (
        'Path extension interval', 'Global frontier planner', 1.0, 15.0, 3.0, 1,
        'Minimum seconds of stable Nav2 motion before a route near its tail is '
        'extended toward the next frontier. Lower values update more often.'
    ),
    'wall_transit_weight': (
        'Mapped-area wall preference', 'Global frontier planner', 0.0, 10.0, 4.0, 1,
        'Strength of the safe wall-offset preference when every frontier route '
        'must cross previously traveled space.'
    ),
    'unavoidable_route_horizon': (
        'Unavoidable transit horizon', 'Global frontier planner',
        10.0, 40.0, 25.0, 1,
        'Maximum wall-biased route length prepared through mapped space before '
        'rolling extension continues toward the frontier.'
    ),
    'progress_timeout': (
        'Segment completion timeout', 'Global frontier planner',
        5.0, 120.0, 20.0, 1,
        'Seconds Nav2 may take to make 0.25 m of significant route progress '
        'before the segment is considered stalled and recovery begins.'
    ),
    'discovery_progress_timeout': (
        'Map discovery timeout', 'Global frontier planner',
        10.0, 180.0, 30.0, 1,
        'Seconds Nav2 may follow a route without adding at least 1.0 m² of '
        'new free map before switching to a more productive frontier.'
    ),
    'robot_clearance': (
        'Robot route body clearance', 'Path geometry', 0.05, 1.20, 0.10, 2,
        'Minimum obstacle distance from any outside footprint point along '
        'OpenCV routes. The robot radius is added automatically.'
    ),
    'goal_clearance': (
        'Frontier goal body clearance', 'Path geometry', 0.05, 1.50, 0.15, 2,
        'Minimum obstacle distance from any outside footprint point at a '
        'frontier goal. The robot radius is added automatically.'
    ),
    'waypoint_spacing': (
        'Waypoint spacing', 'Path geometry', 0.20, 2.00, 0.75, 2,
        'Distance between poses sent to NavigateThroughPoses; smaller is denser.'
    ),
    'minimum_frontier_size': (
        'Minimum frontier size', 'Frontier selection', 0.10, 3.00, 0.50, 2,
        'Reject frontier groups shorter than this approximate map distance.'
    ),
    'minimum_goal_distance': (
        'Preferred goal distance', 'Frontier selection', 0.50, 6.00, 1.80, 2,
        'Preferred minimum distance to a frontier goal before fallback is used.'
    ),
    'minimum_route_length': (
        'Preferred route length', 'Path geometry', 0.50, 8.00, 2.00, 2,
        'Preferred minimum usable A* route length.'
    ),
    'fallback_goal_distance': (
        'Fallback goal distance', 'Frontier selection', 0.20, 3.00, 0.80, 2,
        'Shorter goal-distance requirement used when the map is still small.'
    ),
    'fallback_route_length': (
        'Fallback route length', 'Path geometry', 0.10, 2.00, 0.35, 2,
        'Shortest route accepted when no preferred-length route is available.'
    ),
    'route_horizon': (
        'Normal route horizon', 'Path geometry', 2.0, 30.0, 10.0, 1,
        'Maximum normal route length sent to Nav2 before rolling extension.'
    ),
    'planning_period': (
        'Planner evaluation period', 'Replanning and recovery',
        0.20, 5.00, 2.00, 2,
        'Seconds between frontier evaluations while the planner is available.'
    ),
    'preview_planning_period': (
        'Prepared-route refresh', 'Replanning and recovery',
        0.20, 3.00, 0.50, 2,
        'Seconds between preemptive global-route calculations while wall '
        'following owns motion. Lower values make handoff routes fresher.'
    ),
    'preview_maximum_age': (
        'Prepared-route maximum age', 'Replanning and recovery',
        0.30, 5.00, 1.25, 2,
        'Oldest cached global route that may be dispatched immediately when '
        'wall following yields; older routes are recalculated.'
    ),
    'maximum_route_poses': (
        'Maximum route poses', 'Path geometry', 5, 100, 40, 0,
        'Maximum number of poses retained in one NavigateThroughPoses goal.'
    ),
    'wall_heading_radius': (
        'Wall heading-fit radius', 'Path geometry', 0.30, 4.00, 1.50, 2,
        'Map radius used to fit nearby walls when orienting route poses.'
    ),
    'rolling_replan_poses': (
        'Route-tail pose trigger', 'Replanning and recovery', 0, 12, 4, 0,
        'Begin route extension when this many poses remain; zero disables it.'
    ),
    'rolling_replan_distance': (
        'Route-tail distance trigger', 'Replanning and recovery',
        0.5, 10.0, 4.0, 1,
        'Route extension also requires being within this distance of the tail.'
    ),
    'visit_record_spacing': (
        'Travel-history spacing', 'Novelty and mapped transit',
        0.05, 1.00, 0.25, 2,
        'Distance moved before another visited-location sample is recorded.'
    ),
    'unavoidable_transit_threshold': (
        'Unavoidable revisit threshold', 'Novelty and mapped transit',
        0.2, 5.0, 1.0, 1,
        'Minimum shared revisit distance that activates penalty-free wall transit.'
    ),
    'continuity_weight': (
        'Goal continuity weight', 'Frontier selection', 0.0, 5.0, 0.8, 1,
        'Discourages large jumps away from the current frontier target.'
    ),
    'frontier_lateral_penalty': (
        'Frontier lateral penalty', 'Frontier selection', 0.0, 5.0, 0.4, 1,
        'Penalizes frontier cells far to the side of the current heading.'
    ),
    'corridor_lookahead': (
        'Map corridor lookahead', 'Corridor planning', 1.0, 15.0, 6.0, 1,
        'Forward distance searched for a mapped corridor-center goal.'
    ),
    'corridor_half_width': (
        'Corridor search half-width', 'Corridor planning', 1.0, 10.0, 4.0, 1,
        'Lateral half-width of the mapped corridor search region.'
    ),
    'minimum_corridor_width': (
        'Minimum corridor width', 'Corridor planning', 0.8, 5.0, 1.2, 1,
        'Reject inferred corridors narrower than this safe width. The wall '
        'follower also places a virtual wall across narrower entrances.'
    ),
    'narrow_corridor_confirmation_time': (
        'Narrow-corridor confirmation', 'Corridor planning',
        0.0, 2.0, 0.3, 2,
        'Time that two adjacent LiDAR width measurements must remain below '
        'the minimum before the entrance becomes a virtual wall.'
    ),
    'significant_progress': (
        'Progress segment distance', 'Replanning and recovery',
        0.05, 1.00, 0.25, 2,
        'Distance Nav2 must advance to reset the segment completion timer.'
    ),
    'minimum_discovery_area_gain': (
        'Minimum discovery gain', 'Replanning and recovery',
        0.1, 10.0, 1.0, 1,
        'Square metres of new free map required to reset the discovery timer.'
    ),
    'stuck_radius': (
        'Stuck-region radius', 'Replanning and recovery', 0.5, 5.0, 2.0, 1,
        'Radius used to decide whether the robot remained in one physical region.'
    ),
    'stuck_timeout': (
        'Physical stuck timeout', 'Replanning and recovery',
        15.0, 300.0, 120.0, 1,
        'Time inside the stuck-region radius before backup recovery is allowed.'
    ),
    'recovery_backup_distance': (
        'Recovery backup distance', 'Replanning and recovery',
        0.2, 4.0, 2.14, 2,
        'Collision-checked Nav2 backup distance used only as the final '
        'recovery after contact, a prolonged stall, and failed replans.'
    ),
    'backup_replan_attempts': (
        'Replans before backup', 'Replanning and recovery',
        1, 8, 2, 0,
        'Number of failed least-explored global recovery routes required '
        'before physical-contact backup is allowed.'
    ),
    'slam_map_update_interval': (
        'Map refresh interval', 'SLAM mapping', 0.20, 5.00, 1.00, 2,
        'Seconds between published occupancy-map updates; lower is faster but uses more CPU.'
    ),
    'slam_resolution': (
        'Map resolution', 'SLAM mapping', 0.03, 0.10, 0.05, 2,
        'Map cell size in metres; smaller cells look sharper and cost more computation.'
    ),
    'slam_minimum_travel_distance': (
        'Scan travel distance', 'SLAM mapping', 0.05, 1.00, 0.30, 2,
        'Minimum robot translation before SLAM processes another scan.'
    ),
    'slam_minimum_travel_heading': (
        'Scan heading change', 'SLAM mapping', 0.02, 0.80, 0.10, 2,
        'Minimum rotation in radians before SLAM processes another scan.'
    ),
    'slam_scan_buffer_size': (
        'Scan buffer size', 'SLAM mapping', 5, 50, 10, 0,
        'Recent scans retained for matching; larger improves context but costs memory and CPU.'
    ),
    'slam_scan_buffer_distance': (
        'Scan buffer distance', 'SLAM mapping', 3.0, 20.0, 8.0, 1,
        'Maximum travel distance represented by scans retained in the matching buffer.'
    ),
    'slam_link_match_response': (
        'Fine link-match threshold', 'SLAM mapping', 0.05, 0.80, 0.20, 2,
        'Minimum fine scan-match confidence; lower accepts more matches and more risk.'
    ),
    'slam_link_scan_distance': (
        'Link scan distance', 'SLAM mapping', 0.5, 5.0, 2.0, 1,
        'Maximum distance between scans considered for local pose-graph links.'
    ),
    'slam_loop_search_distance': (
        'Loop search distance', 'SLAM mapping', 1.0, 12.0, 3.0, 1,
        'Radius searched for loop closures; larger corrects longer loops at greater cost.'
    ),
}

LOCALIZATION_PROFILE_VERSION = 3
LOCALIZATION_PROFILE_VALUES = {
    'planning_period': 2.0,
    'slam_map_update_interval': 1.0,
    'slam_minimum_travel_distance': 0.30,
    'slam_scan_buffer_size': 10,
    'slam_scan_buffer_distance': 8.0,
    'slam_link_match_response': 0.20,
    'slam_loop_search_distance': 3.0,
    # Version 2 changes these values from base-origin distance to actual
    # outside-footprint clearance; migrate the old radius-like defaults.
    'robot_clearance': 0.10,
    'goal_clearance': 0.15,
    # Version 3 preserves the physical gaps of settings that formerly meant
    # LiDAR/base-origin range but now correctly mean outside-body clearance.
    'target_wall_distance': 0.60,
    'front_stop_distance': 0.50,
    'revisited_right_turn_clearance': 0.50,
    'emergency_front_distance': 0.15,
}

SIMULATION_WORLDS = {
    'Current obstacle field': 'turtlebot3_world_spacious.world',
    'Corridor building': 'square_building_10ft_hallway.world',
}

REQUIRED_ROS_DISTRO = 'humble'
REQUIRED_SIMULATION_PACKAGES = (
    'gazebo_ros',
    'joint_state_publisher',
    'nav2_bringup',
    'rplidar_ros',
    'rviz2',
    'slam_toolbox',
    'twist_mux',
)
QXL_ERROR_PATTERNS = (
    'execbuffer failed',
    'failed to allocate gem object',
    'qxl_alloc_ioctl: failed',
    'qxl_process_single_command',
)
MINIMUM_QXL_VRAM_MIB = 64


def qxl_errors_from_journal(text):
    """Return QXL graphics failures found in kernel or Xorg journal text."""
    return sorted({
        line.strip()
        for line in text.splitlines()
        if 'qxl' in line.lower()
        and any(pattern in line.lower() for pattern in QXL_ERROR_PATTERNS)
    })


def qxl_vram_mib_from_journal(text):
    """Extract the most recent QXL VRAM size reported by the kernel."""
    matches = re.findall(
        r'qxl:\s+(\d+)\s*M(?:iB)?\s+of VRAM memory size',
        text,
        flags=re.IGNORECASE,
    )
    return int(matches[-1]) if matches else None


def find_workspace():
    """Find the source workspace without assuming the same home directory."""
    candidates = []
    configured = os.environ.get('ROBOT_WORKSPACE')
    if configured:
        candidates.append(Path(configured).expanduser())
    current = Path.cwd()
    candidates.extend([current, *current.parents, Path.home() / 'test_ws'])
    for candidate in candidates:
        package = candidate / 'src' / 'robot_run_manager' / 'package.xml'
        if package.is_file():
            return candidate.resolve()
    return Path.home() / 'test_ws'


def find_autonomy_workspace(manager_workspace):
    """Find the workspace containing the dedicated exploration launch."""
    configured = os.environ.get('BIGSWEEP_WORKSPACE')
    candidates = [
        Path(configured).expanduser() if configured else None,
        manager_workspace,
        Path.home() / 'BigSweepLogic',
    ]
    relative_launch = (
        Path('src/create_robot/create_driver/launch')
        / 'autonomous_exploration.launch.py'
    )
    for candidate in candidates:
        if candidate is not None and (candidate / relative_launch).is_file():
            return candidate.resolve()
    return None


class RunManagerWindow(QMainWindow):
    """Run-manager window with initial safety and process controls."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.cmd_vel_publisher = ros_node.create_publisher(Twist, '/cmd_vel', 10)
        self.workspace = find_workspace()
        self.autonomy_workspace = find_autonomy_workspace(self.workspace)
        self.processes = {}
        self.buttons = {}
        self.preflight_passed = False
        self.stop_publish_count = 0
        self.active_run = None
        self.run_metadata = None
        self.close_pending = False
        self.combined_wall_mapping = False
        self.settings = QSettings('test_ws', 'Robot Run Manager')
        self._apply_settings_migrations()
        self.config_sliders = {}
        self.config_value_labels = {}
        self.coverage_inputs = {}
        self.wall_behavior_state = 'inactive'
        self.coordination_state = 'FRONTIER_NAVIGATION'
        self.exploration_state = 'STOPPED'
        self.exploration_paused = False
        self.exploration_emergency_stopped = False
        self.wall_state_subscription = ros_node.create_subscription(
            String, '/wall_behavior_state', self._wall_state_changed, 10
        )
        self.coordination_state_subscription = ros_node.create_subscription(
            String, '/exploration_coordination_state',
            self._coordination_state_changed, 10
        )
        self.exploration_status_subscription = ros_node.create_subscription(
            String,
            '/exploration_supervisor/status',
            self._exploration_status_changed,
            10,
        )

        self.stop_timer = QTimer(self)
        self.stop_timer.setInterval(50)
        self.stop_timer.timeout.connect(self._publish_stop)

        self.supervisor_tree = SupervisorTree(
            ros_node, self._supervisor_state
        )
        self.behavior_tree_items = {}

        self.setWindowTitle('Robot Run Manager')
        self.resize(950, 760)
        self._build_ui()
        self.ros_spin_timer = QTimer(self)
        self.ros_spin_timer.setInterval(20)
        self.ros_spin_timer.timeout.connect(self._spin_ros_once)
        self.ros_spin_timer.start()
        self.behavior_tree_timer = QTimer(self)
        self.behavior_tree_timer.setInterval(500)
        self.behavior_tree_timer.timeout.connect(self._tick_behavior_tree)
        self.behavior_tree_timer.start()
        self._tick_behavior_tree()
        self._update_controls()

    def _apply_settings_migrations(self):
        """Apply one-time safer defaults to existing desktop settings."""
        current = self.settings.value(
            'configuration/localization_profile_version', 0, type=int
        )
        if current >= LOCALIZATION_PROFILE_VERSION:
            return
        for key, value in LOCALIZATION_PROFILE_VALUES.items():
            self.settings.setValue(f'tuning/{key}', value)
        self.settings.setValue(
            'configuration/localization_profile_version',
            LOCALIZATION_PROFILE_VERSION,
        )
        self.settings.sync()

    def _build_ui(self):
        root = QWidget()
        root_layout = QVBoxLayout(root)
        tabs = QTabWidget()
        tabs.addTab(root, 'Operations')

        title = QLabel('Robot Run Manager')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 24px; font-weight: bold;')
        root_layout.addWidget(title)

        role_layout = QGridLayout()
        role_layout.addWidget(QLabel('This machine:'), 0, 0)
        self.role_selector = QComboBox()
        self.role_selector.addItems([
            'Select machine...', 'Real robot', 'Development / simulation'
        ])
        self.role_selector.currentIndexChanged.connect(self._role_changed)
        role_layout.addWidget(self.role_selector, 0, 1)
        self.gazebo_gui_switch = QCheckBox('Show Gazebo GUI')
        self.gazebo_gui_switch.setToolTip(
            'Checked: open the Gazebo 3D window with the simulation. '
            'Unchecked: run only the Gazebo server to reduce CPU/GPU use.'
        )
        self.gazebo_gui_switch.setChecked(
            self.settings.value('simulation/show_gazebo_gui', True, type=bool)
        )
        self.gazebo_gui_switch.toggled.connect(self._gazebo_gui_changed)
        role_layout.addWidget(self.gazebo_gui_switch, 0, 2)
        self.rviz_gui_switch = QCheckBox('Show RViz GUI')
        self.rviz_gui_switch.setToolTip(
            'Checked: open RViz with the simulation. Unchecked: keep the '
            'simulation running without the RViz visualization window.'
        )
        self.rviz_gui_switch.setChecked(
            self.settings.value('simulation/show_rviz_gui', True, type=bool)
        )
        self.rviz_gui_switch.toggled.connect(self._rviz_gui_changed)
        role_layout.addWidget(self.rviz_gui_switch, 0, 3)
        role_layout.addWidget(QLabel('Gazebo FPS:'), 0, 4)
        self.gazebo_fps_selector = QComboBox()
        self.gazebo_fps_selector.addItems(['10', '15', '20', '30', '60'])
        saved_fps = str(
            self.settings.value('simulation/gazebo_gui_fps', 15, type=int)
        )
        if saved_fps not in {'10', '15', '20', '30', '60'}:
            saved_fps = '15'
        self.gazebo_fps_selector.setCurrentText(saved_fps)
        self.gazebo_fps_selector.setToolTip(
            'Maximum Gazebo window render rate. This does not change physics, '
            'sensor timestamps, or simulation clock speed.'
        )
        self.gazebo_fps_selector.currentTextChanged.connect(
            self._gazebo_fps_changed
        )
        role_layout.addWidget(self.gazebo_fps_selector, 0, 5)
        self.process_label = QLabel('Managed processes: none')
        role_layout.addWidget(self.process_label, 0, 6)
        role_layout.addWidget(QLabel('Simulation world:'), 1, 0)
        self.world_selector = QComboBox()
        self.world_selector.addItems(SIMULATION_WORLDS.keys())
        saved_world = self.settings.value(
            'simulation/world', 'Current obstacle field', type=str
        )
        if saved_world not in SIMULATION_WORLDS:
            saved_world = 'Current obstacle field'
        self.world_selector.setCurrentText(saved_world)
        self.world_selector.setToolTip(
            'Select the Gazebo world used the next time simulation starts.'
        )
        self.world_selector.currentTextChanged.connect(
            lambda value: self.settings.setValue('simulation/world', value)
        )
        role_layout.addWidget(self.world_selector, 1, 1, 1, 2)
        role_layout.setColumnStretch(6, 1)
        root_layout.addLayout(role_layout)

        run_layout = QGridLayout()
        run_layout.addWidget(QLabel('Operator:'), 0, 0)
        self.operator_input = QLineEdit()
        self.operator_input.setPlaceholderText('name or initials')
        self.operator_input.textChanged.connect(self._update_controls)
        run_layout.addWidget(self.operator_input, 0, 1)
        run_layout.addWidget(QLabel('Scenario:'), 0, 2)
        self.scenario_input = QLineEdit()
        self.scenario_input.setPlaceholderText('route / conditions / test purpose')
        run_layout.addWidget(self.scenario_input, 0, 3)
        run_layout.setColumnStretch(3, 1)
        root_layout.addLayout(run_layout)

        self.run_label = QLabel(f'Run storage: {self.workspace / "runs"}')
        self.run_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        root_layout.addWidget(self.run_label)

        map_layout = QGridLayout()
        map_layout.addWidget(QLabel('Map name:'), 0, 0)
        self.map_name_input = QLineEdit()
        self.map_name_input.setPlaceholderText('example: warehouse_east')
        map_layout.addWidget(self.map_name_input, 0, 1)
        map_layout.addWidget(
            QLabel(f'Saved under: {self.workspace / "maps"}'), 0, 2
        )
        map_layout.setColumnStretch(2, 1)
        root_layout.addLayout(map_layout)

        remote_layout = QGridLayout()
        remote_layout.addWidget(QLabel('Transfer:'), 0, 0)
        self.transfer_direction = QComboBox()
        self.transfer_direction.addItems(['Upload run', 'Download run'])
        remote_layout.addWidget(self.transfer_direction, 0, 1)
        remote_layout.addWidget(QLabel('SSH host:'), 0, 2)
        self.remote_host_input = QLineEdit()
        self.remote_host_input.setPlaceholderText('user@computer')
        remote_layout.addWidget(self.remote_host_input, 0, 3)
        remote_layout.addWidget(QLabel('Remote runs:'), 1, 0)
        self.remote_path_input = QLineEdit('~/test_ws/runs')
        remote_layout.addWidget(self.remote_path_input, 1, 1)
        remote_layout.addWidget(QLabel('Remote run name:'), 1, 2)
        self.remote_run_input = QLineEdit()
        self.remote_run_input.setPlaceholderText('needed for download')
        remote_layout.addWidget(self.remote_run_input, 1, 3)
        remote_layout.addWidget(QLabel('Git commit message:'), 2, 0)
        self.commit_message_input = QLineEdit()
        self.commit_message_input.setPlaceholderText('describe GUI changes')
        remote_layout.addWidget(self.commit_message_input, 2, 1, 1, 3)
        remote_layout.setColumnStretch(3, 1)
        root_layout.addLayout(remote_layout)

        self.notice = QLabel(
            'Run Preflight Check before starting hardware. Emergency Stop is '
            'a software stop; keep the physical emergency stop within reach.'
        )
        self.notice.setWordWrap(True)
        self.notice.setStyleSheet(
            'background: #fff3cd; border: 1px solid #e0b000; padding: 8px;'
        )
        root_layout.addWidget(self.notice)

        for group_name, actions in ACTION_GROUPS.items():
            group = QGroupBox(group_name)
            grid = QGridLayout(group)
            for index, action in enumerate(actions):
                button = QPushButton(action)
                button.setToolTip(ACTION_TOOLTIPS[action])
                self.buttons[action] = button
                grid.addWidget(button, index // 3, index % 3)
                if action in HIDDEN_SUPERVISOR_EXPLORATION_ACTIONS:
                    button.setVisible(False)
            root_layout.addWidget(group)

        self.buttons['Start Wall Follower'].setText(
            'Start Behavior-Tree Exploration'
        )
        self.buttons['Stop Wall Follower'].setText(
            'Stop Behavior-Tree Exploration'
        )

        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setMaximumBlockCount(2000)
        self.output.setPlaceholderText('Preflight and managed-process output')
        root_layout.addWidget(self.output, 1)

        self.buttons['Preflight Check'].clicked.connect(self.run_preflight)
        self.buttons['Start Robot'].clicked.connect(self.start_robot)
        self.buttons['Stop Robot'].clicked.connect(self.stop_robot)
        self.buttons['Start Xbox Teleop'].clicked.connect(
            self.start_xbox_teleop
        )
        self.buttons['Stop Xbox Teleop'].clicked.connect(
            self.stop_xbox_teleop
        )
        self.buttons['Emergency Stop'].clicked.connect(self.emergency_stop)
        self.buttons['Kill ROS Processes'].clicked.connect(
            self.kill_ros_processes
        )
        self.buttons['Kill ROS Processes'].setStyleSheet(
            'background: #b00020; color: white; font-weight: bold;'
        )
        self.buttons['Exploration E-Stop'].setStyleSheet(
            'background: #b00020; color: white; font-weight: bold;'
        )
        self.buttons['Open RViz'].clicked.connect(self.open_rviz)
        self.buttons['Open Gazebo'].clicked.connect(self.open_gazebo)
        self.buttons['Start Recording'].clicked.connect(self.start_recording)
        self.buttons['Stop Recording'].clicked.connect(self.stop_recording)
        for event_name in ('Goal', 'Collision', 'Near Miss', 'Intervention'):
            self.buttons[f'Mark {event_name}'].clicked.connect(
                lambda _checked=False, name=event_name: self.mark_event(name)
            )
        self.buttons['Start Mapping'].clicked.connect(self.start_mapping)
        self.buttons['Stop Mapping'].clicked.connect(self.stop_mapping)
        self.buttons['Save Map'].clicked.connect(self.save_map)
        self.buttons['Start Autonomous Exploration'].clicked.connect(
            self.start_autonomous_exploration
        )
        self.buttons['Pause Exploration'].clicked.connect(
            lambda: self.set_exploration_paused(True)
        )
        self.buttons['Resume Exploration'].clicked.connect(
            lambda: self.set_exploration_paused(False)
        )
        self.buttons['Stop Autonomous Exploration'].clicked.connect(
            self.stop_autonomous_exploration
        )
        self.buttons['Exploration E-Stop'].clicked.connect(
            lambda: self.set_exploration_emergency_stop(True)
        )
        self.buttons['Clear Exploration E-Stop'].clicked.connect(
            lambda: self.set_exploration_emergency_stop(False)
        )
        self.buttons['Start Wall Follower'].clicked.connect(
            self.start_wall_follower
        )
        self.buttons['Stop Wall Follower'].clicked.connect(
            self.stop_wall_follower
        )
        self.buttons['Start Simulation'].clicked.connect(self.start_simulation)
        self.buttons['Stop Simulation'].clicked.connect(self.stop_simulation)
        self.buttons['Start Coverage'].clicked.connect(self.start_coverage)
        self.buttons['Stop Coverage'].clicked.connect(self.stop_coverage)
        self.buttons['Replay Selected Run'].clicked.connect(self.toggle_replay)
        self.buttons['Validate Selected Run'].clicked.connect(self.validate_run)
        self.buttons['Transfer Run'].clicked.connect(self.transfer_run)
        self.buttons['Download Code'].clicked.connect(self.download_code)
        self.buttons['Upload Code'].clicked.connect(self.upload_code)
        self.buttons['View Logs'].clicked.connect(self.view_logs)
        self.buttons['Flag & Open Logs'].clicked.connect(self.flag_and_open_logs)
        self.buttons['Rebuild & Restart Manager'].clicked.connect(
            self.rebuild_and_restart_manager
        )
        self.buttons['Install Desktop Launcher'].clicked.connect(
            self.install_desktop_launcher
        )

        tabs.addTab(self._build_configuration_tab(), 'Configuration')
        tabs.addTab(self._build_coverage_tab(), 'Coverage')
        tabs.addTab(self._build_behavior_tree_tab(), 'Behavior Tree')
        self.setCentralWidget(tabs)
        status = QStatusBar()
        status.showMessage('Ready for preflight')
        self.setStatusBar(status)

    def _build_configuration_tab(self):
        page = QWidget()
        page_layout = QVBoxLayout(page)
        introduction = QLabel(
            'These values are applied the next time a mapper or wall follower '
            'starts. Hover over a slider for the same description.'
        )
        introduction.setWordWrap(True)
        page_layout.addWidget(introduction)
        groups = {}
        rows = {}
        for key, specification in TUNING_VARIABLES.items():
            name, group_name, minimum, maximum, default, decimals, description = (
                specification
            )
            if group_name not in groups:
                box = QGroupBox(group_name)
                layout = QGridLayout(box)
                layout.setColumnStretch(1, 1)
                groups[group_name] = layout
                rows[group_name] = 0
                page_layout.addWidget(box)
            layout = groups[group_name]
            row = rows[group_name]
            scale = 10 ** decimals
            slider = QSlider(Qt.Horizontal)
            slider.setRange(round(minimum * scale), round(maximum * scale))
            stored = self.settings.value(f'tuning/{key}', default, type=float)
            stored = min(maximum, max(minimum, stored))
            slider.setValue(round(stored * scale))
            slider.setToolTip(description)
            value_label = QLabel()
            value_label.setMinimumWidth(55)
            description_label = QLabel(description)
            description_label.setWordWrap(True)
            description_label.setStyleSheet('color: #555;')
            slider.valueChanged.connect(
                lambda value, setting=key, digits=decimals, factor=scale:
                self._configuration_changed(setting, value, factor, digits)
            )
            self.config_sliders[key] = slider
            self.config_value_labels[key] = value_label
            layout.addWidget(QLabel(name), row, 0)
            layout.addWidget(slider, row, 1)
            layout.addWidget(value_label, row, 2)
            layout.addWidget(description_label, row + 1, 0, 1, 3)
            rows[group_name] += 2
            self._configuration_changed(key, slider.value(), scale, decimals)
        for group_name, layout in groups.items():
            section_reset = QPushButton(f'Reset {group_name} defaults')
            section_reset.setToolTip(
                f'Restore only the {group_name} sliders to documented defaults.'
            )
            section_reset.clicked.connect(
                lambda _checked=False, section=group_name:
                self._reset_configuration(section)
            )
            layout.addWidget(section_reset, rows[group_name], 0, 1, 3)
        reset = QPushButton('Reset tuning defaults')
        reset.setToolTip('Restore every tuning slider to its documented default.')
        reset.clicked.connect(lambda: self._reset_configuration())
        page_layout.addWidget(reset)
        page_layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        return scroll

    def _build_coverage_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        introduction = QLabel(
            'OpenNav Coverage plans parallel sweeping passes inside the field '
            'rectangle below. Physical values apply when simulation starts; '
            'route choices apply each time coverage starts.'
        )
        introduction.setWordWrap(True)
        layout.addWidget(introduction)

        physical_box = QGroupBox('Rectangular robot')
        physical_layout = QGridLayout(physical_box)
        physical_specs = {
            'robot_width': (
                'Robot width', 0.20, 2.00, 0.735, 3,
                'Full 0.735 m Nav2 footprint width, including the sweep envelope.',
            ),
            'operation_width': (
                'Sweep width', 0.05, 1.50, 0.520, 3,
                '0.570 m rear wheel-center spacing minus 0.050 m overlap.',
            ),
            'turning_radius': (
                'Turning radius', 0.05, 3.00, 0.500, 3,
                'Smallest smooth forward coverage turn. The robot can pivot, '
                'but smooth turns reduce caster scrub and missed coverage.',
            ),
        }
        for row, (key, spec) in enumerate(physical_specs.items()):
            label, minimum, maximum, default, decimals, explanation = spec
            control = QDoubleSpinBox()
            control.setRange(minimum, maximum)
            control.setDecimals(decimals)
            control.setSingleStep(0.01)
            control.setSuffix(' m')
            control.setValue(
                min(maximum, max(minimum, self.settings.value(
                    f'coverage/{key}', default, type=float
                )))
            )
            control.valueChanged.connect(
                lambda value, setting=key:
                self.settings.setValue(f'coverage/{setting}', value)
            )
            description = QLabel(explanation)
            description.setWordWrap(True)
            description.setStyleSheet('color: #555;')
            physical_layout.addWidget(QLabel(label), row * 2, 0)
            physical_layout.addWidget(control, row * 2, 1)
            physical_layout.addWidget(description, row * 2 + 1, 0, 1, 2)
            self.coverage_inputs[key] = control
        layout.addWidget(physical_box)

        route_box = QGroupBox('Coverage strategy')
        route_layout = QGridLayout(route_box)
        selectors = {
            'route_type': (
                'Route',
                ['BOUSTROPHEDON', 'SNAKE', 'SPIRAL', 'CUSTOM'],
                'BOUSTROPHEDON',
                'BOUSTROPHEDON: alternating lawnmower passes; SNAKE: sequential '
                'winding swaths; SPIRAL: progressively inward/outward passes; '
                'CUSTOM: manually specified swath order.',
            ),
            'continuity_type': (
                'Continuity',
                ['CONTINUOUS', 'DISCONTINUOUS'],
                'CONTINUOUS',
                'CONTINUOUS joins passes with drivable curves. DISCONTINUOUS '
                'returns separate segments without connecting turns.',
            ),
            'path_type': (
                'Path',
                ['DUBIN', 'REEDS_SHEPP'],
                'DUBIN',
                'DUBIN uses forward-only curves. REEDS_SHEPP may use forward '
                'and reverse motion.',
            ),
        }
        for row, (key, spec) in enumerate(selectors.items()):
            label, options, default, explanation = spec
            selector = QComboBox()
            selector.addItems(options)
            stored = self.settings.value(
                f'coverage/{key}', default, type=str
            )
            selector.setCurrentText(stored if stored in options else default)
            selector.currentTextChanged.connect(
                lambda value, setting=key:
                self.settings.setValue(f'coverage/{setting}', value)
            )
            description = QLabel(explanation)
            description.setWordWrap(True)
            description.setStyleSheet('color: #555;')
            route_layout.addWidget(QLabel(label), row * 2, 0)
            route_layout.addWidget(selector, row * 2, 1)
            route_layout.addWidget(description, row * 2 + 1, 0, 1, 2)
            self.coverage_inputs[key] = selector
        layout.addWidget(route_box)

        field_box = QGroupBox('Field rectangle in map coordinates')
        field_layout = QGridLayout(field_box)
        field_specs = {
            'field_x_min': ('Minimum X', -100.0, 100.0, 5.0),
            'field_x_max': ('Maximum X', -100.0, 100.0, 15.0),
            'field_y_min': ('Minimum Y', -100.0, 100.0, -5.0),
            'field_y_max': ('Maximum Y', -100.0, 100.0, 5.0),
        }
        for row, (key, spec) in enumerate(field_specs.items()):
            label, minimum, maximum, default = spec
            control = QDoubleSpinBox()
            control.setRange(minimum, maximum)
            control.setDecimals(2)
            control.setSingleStep(0.25)
            control.setSuffix(' m')
            control.setValue(
                min(maximum, max(minimum, self.settings.value(
                    f'coverage/{key}', default, type=float
                )))
            )
            control.valueChanged.connect(
                lambda value, setting=key:
                self.settings.setValue(f'coverage/{setting}', value)
            )
            field_layout.addWidget(QLabel(label), row, 0)
            field_layout.addWidget(control, row, 1)
            self.coverage_inputs[key] = control
        field_note = QLabel(
            'The polygon must describe clear drivable floor. OpenNav Coverage '
            'does not infer the field boundary from the occupancy map.'
        )
        field_note.setWordWrap(True)
        field_note.setStyleSheet('color: #555;')
        field_layout.addWidget(field_note, len(field_specs), 0, 1, 2)
        layout.addWidget(field_box)

        reset = QPushButton('Reset rectangular coverage defaults')
        reset.clicked.connect(self._reset_coverage_defaults)
        layout.addWidget(reset)
        layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        return scroll

    def _reset_coverage_defaults(self):
        defaults = {
            'robot_width': 0.735,
            'operation_width': 0.520,
            'turning_radius': 0.500,
            'route_type': 'BOUSTROPHEDON',
            'continuity_type': 'CONTINUOUS',
            'path_type': 'DUBIN',
            'field_x_min': 5.0,
            'field_x_max': 15.0,
            'field_y_min': -5.0,
            'field_y_max': 5.0,
        }
        for key, value in defaults.items():
            control = self.coverage_inputs[key]
            if isinstance(control, QComboBox):
                control.setCurrentText(value)
            else:
                control.setValue(value)
        self.statusBar().showMessage('Restored rectangular coverage defaults')

    def _coverage_value(self, key):
        control = self.coverage_inputs[key]
        if isinstance(control, QComboBox):
            return control.currentText()
        return control.value()

    def _build_behavior_tree_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        explanation = QLabel(
            'Live navigation supervisor and behavior editor. Select a leaf to '
            'see why it runs and tune its persistent settings. Changes apply '
            'the next time the wall follower starts.'
        )
        explanation.setWordWrap(True)
        layout.addWidget(explanation)
        self.behavior_tree_view = QTreeWidget()
        self.behavior_tree_view.setColumnCount(3)
        self.behavior_tree_view.setHeaderLabels(
            ['Behavior', 'Status', 'Plain-language detail']
        )
        self.behavior_tree_view.setAlternatingRowColors(True)
        self.behavior_tree_behaviors = {}
        layout.addWidget(self.behavior_tree_view, 1)

        def add_behavior(behavior, parent=None):
            item = QTreeWidgetItem([
                behavior.name, behavior.status.value, behavior.feedback_message
            ])
            if parent is None:
                self.behavior_tree_view.addTopLevelItem(item)
            else:
                parent.addChild(item)
            self.behavior_tree_items[behavior.id] = item
            self.behavior_tree_behaviors[id(item)] = behavior
            for child in behavior.children:
                add_behavior(child, item)

        add_behavior(self.supervisor_tree.root)
        self.behavior_tree_view.expandAll()
        self.behavior_tree_view.resizeColumnToContents(0)
        self.behavior_tree_editor = QGroupBox('Selected behavior settings')
        self.behavior_tree_editor_layout = QGridLayout(self.behavior_tree_editor)
        editor_scroll = QScrollArea()
        editor_scroll.setWidgetResizable(True)
        editor_scroll.setMinimumHeight(300)
        editor_scroll.setWidget(self.behavior_tree_editor)
        layout.addWidget(editor_scroll)
        self.behavior_tree_view.currentItemChanged.connect(
            self._behavior_tree_selection_changed
        )
        if self.behavior_tree_view.topLevelItemCount():
            self.behavior_tree_view.setCurrentItem(
                self.behavior_tree_view.topLevelItem(0)
            )
        open_viewer = QPushButton('Open py_trees ROS Viewer')
        open_viewer.setToolTip(
            'Open the official external viewer and connect to this tree snapshot stream.'
        )
        open_viewer.clicked.connect(self.open_behavior_tree_viewer)
        layout.addWidget(open_viewer)
        return page

    def _behavior_tree_selection_changed(self, item, _previous=None):
        """Build a synchronized editor for the selected behaviour leaf."""
        layout = self.behavior_tree_editor_layout
        while layout.count():
            child = layout.takeAt(0)
            if child.widget() is not None:
                child.widget().hide()
                child.widget().deleteLater()
        behavior = self.behavior_tree_behaviors.get(id(item)) if item else None
        if behavior is None:
            return
        detail = QLabel(getattr(behavior, 'detail', '') or
                        'Supervisory branch; select one of its leaves to tune it.')
        detail.setWordWrap(True)
        layout.addWidget(detail, 0, 0, 1, 3)
        keys = tuple(getattr(behavior, 'setting_keys', ()))
        for row, key in enumerate(keys, 1):
            name, _group, minimum, maximum, _default, decimals, description = (
                TUNING_VARIABLES[key]
            )
            scale = 10 ** decimals
            editor = QSlider(Qt.Horizontal)
            editor.setRange(round(minimum * scale), round(maximum * scale))
            editor.setValue(self.config_sliders[key].value())
            editor.setToolTip(description)
            value = QLabel()
            value.setMinimumWidth(55)
            value.setText(f'{editor.value() / scale:.{decimals}f}')
            editor.valueChanged.connect(
                lambda raw, source=self.config_sliders[key], label=value,
                factor=scale, digits=decimals: (
                    source.setValue(raw),
                    label.setText(f'{raw / factor:.{digits}f}')
                )
            )
            self.config_sliders[key].valueChanged.connect(editor.setValue)
            control_row = row * 2 - 1
            description_label = QLabel(
                f'{description} Allowed range: {minimum:.{decimals}f}–'
                f'{maximum:.{decimals}f}; documented default: '
                f'{_default:.{decimals}f}. The saved value is used when the '
                'wall follower is next started.'
            )
            description_label.setWordWrap(True)
            description_label.setStyleSheet('color: #555; margin-bottom: 4px;')
            layout.addWidget(QLabel(name), control_row, 0)
            layout.addWidget(editor, control_row, 1)
            layout.addWidget(value, control_row, 2)
            layout.addWidget(
                description_label, control_row + 1, 0, 1, 3
            )
        if not keys:
            layout.addWidget(QLabel('This branch has no direct tuning values.'), 1, 0, 1, 3)
        else:
            reset = QPushButton('Reset this behavior to defaults')
            reset.clicked.connect(
                lambda _checked=False, selected=keys:
                self._reset_behavior_settings(selected)
            )
            layout.addWidget(reset, len(keys) * 2 + 1, 0, 1, 3)

    def _reset_behavior_settings(self, keys):
        for key in keys:
            default = TUNING_VARIABLES[key][4]
            decimals = TUNING_VARIABLES[key][5]
            self.config_sliders[key].setValue(round(default * 10 ** decimals))
        self.statusBar().showMessage('Restored defaults for selected behavior')

    def _supervisor_state(self):
        node_names = set(self.ros_node.get_node_names())
        navigation_active = any([
            self._is_running('simulation'),
            self._is_running('mapping'),
            self._is_running('autonomous_exploration'),
        ])
        return {
            'always': True,
            'emergency': self.stop_timer.isActive(),
            'system_ready': self.preflight_passed,
            'navigation_active': navigation_active,
            'localization': navigation_active and '/slam_toolbox' in node_names,
            'return_home': False,
            'dead_end_exit': False,
            'recovery': self.coordination_state == 'RECOVERY',
            'wall_following': (
                self.coordination_state == 'WALL_FOLLOWING'
            ),
            'wall_state': self.wall_behavior_state,
            'frontier_navigation': (
                self.coordination_state == 'FRONTIER_NAVIGATION'
            ),
        }

    def _wall_state_changed(self, message):
        self.wall_behavior_state = message.data

    def _coordination_state_changed(self, message):
        self.coordination_state = message.data

    def _exploration_status_changed(self, message):
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.exploration_state = str(status.get('state', 'UNKNOWN'))
        self.exploration_paused = bool(status.get('paused', False))
        self.exploration_emergency_stopped = bool(
            status.get('emergency_stopped', False)
        )
        self.statusBar().showMessage(
            f'Autonomous exploration: {self.exploration_state}'
        )
        self._update_controls()

    def _spin_ros_once(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)

    def _tick_behavior_tree(self):
        self.supervisor_tree.tick()
        colors = {
            'SUCCESS': QColor('#c8e6c9'),
            'RUNNING': QColor('#fff3b0'),
            'FAILURE': QColor('#ffcdd2'),
            'INVALID': QColor('#eeeeee'),
        }
        for behavior in self.supervisor_tree.root.iterate():
            item = self.behavior_tree_items.get(behavior.id)
            if item is None:
                continue
            status = behavior.status.value
            item.setText(1, status)
            item.setText(2, behavior.feedback_message or '')
            for column in range(3):
                item.setBackground(column, colors[status])

    def open_behavior_tree_viewer(self):
        self._start_process(
            'behavior_tree_viewer', 'py-trees-tree-viewer', []
        )

    def _configuration_changed(self, key, slider_value, scale, decimals):
        value = slider_value / scale
        self.config_value_labels[key].setText(f'{value:.{decimals}f}')
        self.settings.setValue(f'tuning/{key}', value)

    def _reset_configuration(self, group_name=None):
        for key, specification in TUNING_VARIABLES.items():
            if group_name is not None and specification[1] != group_name:
                continue
            default = specification[4]
            decimals = specification[5]
            self.config_sliders[key].setValue(round(default * 10 ** decimals))
        label = group_name if group_name is not None else 'all tuning sections'
        self.statusBar().showMessage(f'Restored defaults: {label}')

    def _configuration_value(self, key):
        decimals = TUNING_VARIABLES[key][5]
        if decimals == 0:
            return self.config_sliders[key].value()
        return self.config_sliders[key].value() / 10 ** decimals

    def _save_configuration(self):
        """Flush every current slider value to persistent desktop settings."""
        for key in TUNING_VARIABLES:
            self.settings.setValue(
                f'tuning/{key}', self._configuration_value(key)
            )
        self.settings.setValue(
            'simulation/show_gazebo_gui', self.gazebo_gui_switch.isChecked()
        )
        self.settings.setValue(
            'simulation/show_rviz_gui', self.rviz_gui_switch.isChecked()
        )
        self.settings.setValue(
            'simulation/gazebo_gui_fps', self.gazebo_fps_selector.currentText()
        )
        self.settings.setValue(
            'simulation/world', self.world_selector.currentText()
        )
        for key in self.coverage_inputs:
            self.settings.setValue(
                f'coverage/{key}', self._coverage_value(key)
            )
        self.settings.sync()
        if self.settings.status() != QSettings.NoError:
            self.output.appendPlainText(
                'WARNING: configuration settings could not be saved.'
            )
            return False
        return True

    def _planner_launch_arguments(self):
        keys = (
            'robot_clearance', 'goal_clearance', 'waypoint_spacing',
            'minimum_frontier_size', 'minimum_goal_distance',
            'minimum_route_length', 'fallback_goal_distance',
            'fallback_route_length', 'route_horizon', 'planning_period',
            'preview_planning_period', 'preview_maximum_age',
            'maximum_route_poses', 'wall_heading_radius',
            'rolling_replan_poses', 'rolling_replan_distance',
            'information_weight', 'frontier_bonus', 'revisit_weight',
            'visited_radius', 'visit_record_spacing',
            'unavoidable_transit_threshold', 'travel_weight',
            'continuity_weight', 'forward_weight', 'reverse_penalty',
            'frontier_lateral_penalty', 'clearance_weight',
            'corridor_lookahead', 'corridor_half_width',
            'minimum_corridor_width',
            'route_extension_period',
            'wall_transit_weight', 'unavoidable_route_horizon',
            'progress_timeout', 'significant_progress',
            'discovery_progress_timeout', 'minimum_discovery_area_gain',
            'stuck_radius', 'stuck_timeout', 'recovery_backup_distance',
            'backup_replan_attempts',
        )
        arguments = [
            f'{key}:={self._configuration_value(key)}' for key in keys
        ]
        # The OpenCV map stores obstacle distance from the base origin. For a
        # parallel right wall, add the 0.45 m right footprint extent so this
        # preference uses the same nearest-body clearance as wall following.
        wall_transit_distance = (
            float(self._configuration_value("target_wall_distance")) + 0.45
        )
        arguments.append(f'wall_transit_distance:={wall_transit_distance}')
        return arguments

    def _slam_launch_arguments(self):
        keys = (
            'slam_map_update_interval', 'slam_resolution',
            'slam_minimum_travel_distance', 'slam_minimum_travel_heading',
            'slam_scan_buffer_size', 'slam_scan_buffer_distance',
            'slam_link_match_response', 'slam_link_scan_distance',
            'slam_loop_search_distance',
        )
        return [f'{key}:={self._configuration_value(key)}' for key in keys]

    def _autonomous_launch_arguments(self):
        mapping = {
            'wall_clearance': 'target_wall_distance',
            'wall_speed': 'linear_speed',
            'wall_lost_timeout': 'wall_timeout',
            'wall_heading_gain': 'heading_gain',
            'wall_distance_gain': 'distance_gain',
            'wall_turn_speed': 'turn_speed',
            'wall_front_stop_distance': 'front_stop_distance',
            'wall_emergency_clearance': 'emergency_front_distance',
        }
        return [
            f'{argument}:={self._configuration_value(setting)}'
            for argument, setting in mapping.items()
        ]

    def _role_changed(self):
        self.preflight_passed = False
        self.statusBar().showMessage('Role changed — run preflight again')
        self._update_controls()

    def _machine_selected(self):
        return self.role_selector.currentIndex() in (1, 2)

    def _is_real_robot_role(self):
        return self.role_selector.currentIndex() == 1

    def _set_workflow_styles(
        self, real_robot, running, recording, mapping, simulation,
        replaying, exploring, wall_following,
    ):
        """Highlight the next action and make active stop actions conspicuous."""
        green = (
            'background: #2e7d32; color: white; font-weight: bold; '
            'border: 2px solid #1b5e20; padding: 5px;'
        )
        green_input = (
            'background: #e8f5e9; border: 2px solid #2e7d32; padding: 3px;'
        )
        red = (
            'background: #c62828; color: white; font-weight: bold; '
            'border: 2px solid #8e0000; padding: 5px;'
        )
        for button in self.buttons.values():
            button.setStyleSheet('')
        self.buttons['Kill ROS Processes'].setStyleSheet(
            'background: #b00020; color: white; font-weight: bold;'
        )
        self.buttons['Exploration E-Stop'].setStyleSheet(
            'background: #b00020; color: white; font-weight: bold;'
        )
        self.role_selector.setStyleSheet('')
        self.operator_input.setStyleSheet('')

        operator_ready = bool(self.operator_input.text().strip())
        if not self._machine_selected():
            self.role_selector.setStyleSheet(green_input)
        elif not self.preflight_passed:
            self.buttons['Preflight Check'].setStyleSheet(green)
        elif not operator_ready:
            self.operator_input.setStyleSheet(green_input)
        elif not any((running, mapping, simulation, replaying)):
            primary = 'Start Robot' if real_robot else 'Start Simulation'
            if self.buttons[primary].isEnabled():
                self.buttons[primary].setStyleSheet(green)
            if real_robot and self.buttons['Start Mapping'].isEnabled():
                self.buttons['Start Mapping'].setStyleSheet(green)
            if self.buttons['Start Autonomous Exploration'].isEnabled():
                self.buttons['Start Autonomous Exploration'].setStyleSheet(green)
        elif real_robot and not exploring:
            action = 'Start Autonomous Exploration'
            if self.buttons[action].isEnabled():
                self.buttons[action].setStyleSheet(green)
        elif simulation and not wall_following:
            if self.buttons['Start Wall Follower'].isEnabled():
                self.buttons['Start Wall Follower'].setStyleSheet(green)

        stop_states = {
            'Stop Robot': running,
            'Stop Xbox Teleop': self._is_running('xbox_teleop'),
            'Stop Recording': recording,
            'Stop Mapping': mapping,
            'Stop Simulation': simulation or replaying,
            'Stop Autonomous Exploration': exploring,
            'Stop Wall Follower': wall_following,
        }
        for action, active in stop_states.items():
            if active:
                self.buttons[action].setStyleSheet(red)
        if exploring and self.exploration_emergency_stopped:
            self.buttons['Clear Exploration E-Stop'].setStyleSheet(green)
        elif exploring and self.exploration_paused:
            self.buttons['Resume Exploration'].setStyleSheet(green)
        elif exploring:
            self.buttons['Pause Exploration'].setStyleSheet(red)

    def _gazebo_gui_changed(self, show_gui):
        """Persist and apply the Gazebo client preference."""
        self.settings.setValue('simulation/show_gazebo_gui', show_gui)
        self.settings.sync()
        if self._is_running('simulation'):
            if show_gui:
                self.open_gazebo()
            else:
                self._request_stop('gazebo_client')
                self.statusBar().showMessage(
                    'Gazebo GUI closed; simulation continues headless'
                )
        self._update_controls()

    def _gazebo_fps_changed(self, fps):
        """Persist the client-only Gazebo render limit."""
        self.settings.setValue('simulation/gazebo_gui_fps', int(fps))
        self.settings.sync()
        if self._is_running('gazebo_client'):
            self._request_stop('gazebo_client')
            QTimer.singleShot(750, self.open_gazebo)
            self.statusBar().showMessage(
                f'Restarting Gazebo GUI with a {fps} FPS limit'
            )

    def _rviz_gui_changed(self, show_gui):
        """Persist and apply the RViz window preference."""
        self.settings.setValue('simulation/show_rviz_gui', show_gui)
        self.settings.sync()
        if self._is_running('simulation'):
            if show_gui:
                self.open_rviz()
            else:
                self._request_stop('rviz')
                self.statusBar().showMessage(
                    'RViz GUI closed; simulation continues without it'
                )
        self._update_controls()

    def _update_controls(self):
        running = self._is_running('robot')
        recording = self._is_running('recording')
        mapping = self._is_running('mapping')
        saving_map = self._is_running('save_map')
        simulation = self._is_running('simulation')
        replaying = self._is_running('replay')
        exploring = self._is_running('autonomous_exploration')
        wall_following = self._is_running('wall_follower')
        xbox_teleop = self._is_running('xbox_teleop')
        gazebo_client = self._is_running('gazebo_client')
        rviz = self._is_running('rviz')
        validating = self._is_running('validation')
        transferring = self._is_running('transfer')
        machine_selected = self._machine_selected()
        real_robot = self._is_real_robot_role()
        for action, button in self.buttons.items():
            button.setEnabled(action in IMPLEMENTED_ACTIONS)
            if action not in IMPLEMENTED_ACTIONS:
                button.setToolTip('Planned for a later step')
        self.buttons['Preflight Check'].setEnabled(machine_selected)
        self.buttons['Start Robot'].setEnabled(
            real_robot and self.preflight_passed and not running
            and not mapping and not exploring
        )
        self.buttons['Stop Robot'].setEnabled(running)
        self.buttons['Start Xbox Teleop'].setEnabled(
            real_robot
            and self.preflight_passed
            and running
            and not mapping
            and not xbox_teleop
        )
        self.buttons['Stop Xbox Teleop'].setEnabled(xbox_teleop)
        self.buttons['Emergency Stop'].setEnabled(True)
        self.buttons['Kill ROS Processes'].setEnabled(True)
        self.buttons['Open Gazebo'].setEnabled(
            simulation
            and self.gazebo_gui_switch.isChecked()
            and not gazebo_client
        )
        self.buttons['Open RViz'].setEnabled(
            not rviz
            and (not simulation or self.rviz_gui_switch.isChecked())
        )
        self.buttons['Start Recording'].setEnabled(
            self.preflight_passed and not recording
        )
        self.buttons['Stop Recording'].setEnabled(recording)
        for event_name in ('Goal', 'Collision', 'Near Miss', 'Intervention'):
            self.buttons[f'Mark {event_name}'].setEnabled(recording)
        self.buttons['Start Mapping'].setEnabled(
            real_robot and self.preflight_passed and not running
            and not mapping and not exploring
        )
        self.buttons['Stop Mapping'].setEnabled(mapping)
        self.buttons['Save Map'].setEnabled(
            (mapping or simulation or exploring) and not saving_map
        )
        self.buttons['Start Autonomous Exploration'].setEnabled(
            not exploring
        )
        if exploring:
            self.buttons['Start Autonomous Exploration'].setToolTip(
                'Autonomous exploration is already running.'
            )
        else:
            self.buttons['Start Autonomous Exploration'].setToolTip(
                ACTION_TOOLTIPS['Start Autonomous Exploration']
                + ' Click to see any unmet startup requirement.'
            )
        self.buttons['Pause Exploration'].setEnabled(
            exploring
            and not self.exploration_paused
            and not self.exploration_emergency_stopped
        )
        self.buttons['Resume Exploration'].setEnabled(
            exploring
            and self.exploration_paused
            and not self.exploration_emergency_stopped
        )
        self.buttons['Stop Autonomous Exploration'].setEnabled(exploring)
        self.buttons['Exploration E-Stop'].setEnabled(
            exploring and not self.exploration_emergency_stopped
        )
        self.buttons['Clear Exploration E-Stop'].setEnabled(
            exploring and self.exploration_emergency_stopped
        )
        self.buttons['Start Wall Follower'].setEnabled(
            self.preflight_passed
            and not real_robot
            and not running
            and not mapping
            and not exploring
            and not wall_following
            and not replaying
        )
        self.buttons['Stop Wall Follower'].setEnabled(wall_following)
        self.buttons['Start Simulation'].setEnabled(
            not real_robot
            and self.preflight_passed
            and not simulation
            and not running
            and not mapping
            and not exploring
        )
        self.buttons['Stop Simulation'].setEnabled(simulation or replaying)
        self.buttons['Replay Selected Run'].setEnabled(
            not real_robot
            and self.preflight_passed
            and not recording
            and not running
            and not mapping
        )
        self.buttons['Replay Selected Run'].setText(
            'Stop Replay' if replaying else 'Replay Selected Run'
        )
        self.buttons['Validate Selected Run'].setEnabled(
            not recording and not validating
        )
        self.buttons['Transfer Run'].setEnabled(
            not recording and not transferring
        )
        self.buttons['Rebuild & Restart Manager'].setEnabled(
            not any(self._is_running(name) for name in self.processes)
        )
        self.buttons['Download Code'].setEnabled(not any([
            running, recording, mapping, simulation, replaying, exploring,
        ]))
        self.buttons['Upload Code'].setEnabled(not any([
            running, recording, mapping, simulation, replaying, exploring,
        ]))
        self.buttons['View Logs'].setEnabled(True)
        self.buttons['Flag & Open Logs'].setEnabled(True)
        self.buttons['Install Desktop Launcher'].setEnabled(True)
        active = []
        if running:
            active.append('robot')
        if recording:
            active.append('recording')
        if mapping:
            active.append('mapping')
        if saving_map:
            active.append('map saver')
        if simulation:
            active.append('simulation')
        if replaying:
            active.append('replay')
        if exploring:
            active.append(f'autonomous exploration ({self.exploration_state})')
        if wall_following:
            active.append('wall follower')
        if xbox_teleop:
            active.append('Xbox teleop')
        if gazebo_client:
            active.append('Gazebo window')
        if rviz:
            active.append('RViz window')
        if validating:
            active.append('validation')
        if transferring:
            active.append('transfer')
        self.role_selector.setEnabled(not any([
            running, recording, mapping, simulation, replaying, exploring,
            wall_following, xbox_teleop,
        ]))
        self.process_label.setText(
            f'Managed processes: {", ".join(active)} running' if active
            else 'Managed processes: none'
        )
        self._set_workflow_styles(
            real_robot, running, recording, mapping, simulation, replaying,
            exploring, wall_following,
        )

    def _is_running(self, name):
        process = self.processes.get(name)
        return process is not None and process.state() != QProcess.NotRunning

    def _check_ros_package(self, package):
        try:
            result = subprocess.run(
                ['ros2', 'pkg', 'prefix', package],
                capture_output=True,
                check=False,
                text=True,
                timeout=5,
            )
            return result.returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            return False

    def _journal_output(self):
        try:
            result = subprocess.run(
                ['journalctl', '-b', '--no-pager'],
                capture_output=True,
                check=False,
                text=True,
                timeout=10,
            )
            return result.stdout if result.returncode == 0 else ''
        except (OSError, subprocess.TimeoutExpired):
            return ''

    def _check_opengl(self):
        glxinfo = shutil.which('glxinfo')
        if glxinfo is None:
            return False, 'glxinfo unavailable; install mesa-utils'
        try:
            result = subprocess.run(
                [glxinfo, '-B'],
                capture_output=True,
                check=False,
                text=True,
                timeout=10,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            return False, f'probe failed: {error}'
        output = f'{result.stdout}\n{result.stderr}'
        renderer = re.search(r'OpenGL renderer string:\s*(.+)', output)
        version = re.search(r'OpenGL version string:\s*(.+)', output)
        rendered = result.returncode == 0 and renderer is not None
        details = []
        if renderer:
            details.append(f'renderer={renderer.group(1).strip()}')
        if version:
            details.append(f'version={version.group(1).strip()}')
        if not details:
            details.append(
                output.strip().splitlines()[-1]
                if output.strip()
                else 'no output'
            )
        return rendered, '; '.join(details)

    def run_preflight(self):
        if not self._machine_selected():
            self.statusBar().showMessage('Select this machine before preflight')
            return
        real_robot = self._is_real_robot_role()
        ros_distro = os.environ.get('ROS_DISTRO', '')
        checks = [
            ('ROS 2 command', shutil.which('ros2') is not None, True),
            (
                f'ROS distribution ({REQUIRED_ROS_DISTRO})',
                ros_distro == REQUIRED_ROS_DISTRO,
                True,
            ),
            ('Workspace', self.workspace.is_dir(), True),
            (
                'robot_run_manager package',
                self._check_ros_package('robot_run_manager'),
                True,
            ),
            (
                'generic_motor_driver package',
                self._check_ros_package('generic_motor_driver'),
                real_robot,
            ),
            (
                'Autonomous exploration workspace',
                self.autonomy_workspace is not None
                and (
                    self.autonomy_workspace / 'install' / 'setup.bash'
                ).is_file(),
                True,
            ),
            ('Graphical display', bool(os.environ.get('DISPLAY')), False),
            ('Git command', shutil.which('git') is not None, False),
            ('SSH command', shutil.which('ssh') is not None, False),
            ('Rsync command', shutil.which('rsync') is not None, False),
        ]
        if real_robot:
            checks.extend([
                ('Motor device /dev/ttyUSB0', Path('/dev/ttyUSB0').exists(), True),
                ('Relay device /dev/ttyACM0', Path('/dev/ttyACM0').exists(), True),
                ('LiDAR device /dev/lidar', Path('/dev/lidar').exists(), False),
                ('Physical emergency stop confirmed', False, False),
            ])
        else:
            checks.extend([
                *[
                    (
                        f'ROS package {package}',
                        self._check_ros_package(package),
                        True,
                    )
                    for package in REQUIRED_SIMULATION_PACKAGES
                ],
                (
                    'BigSweep simulation overlay',
                    self.autonomy_workspace is not None
                    and (
                        self.autonomy_workspace / 'install' / 'setup.bash'
                    ).is_file(),
                    True,
                ),
            ])
            journal = self._journal_output()
            qxl_errors = qxl_errors_from_journal(journal)
            qxl_vram_mib = qxl_vram_mib_from_journal(journal)
            qxl_in_use = 'qxl' in journal.lower()
            checks.append((
                'Current-boot QXL health'
                + (f' ({len(qxl_errors)} errors)' if qxl_errors else ''),
                not qxl_errors,
                False,
            ))
            if qxl_in_use:
                checks.append((
                    'QXL graphics memory'
                    + (
                        f' ({qxl_vram_mib} MiB; '
                        f'{MINIMUM_QXL_VRAM_MIB} MiB recommended)'
                        if qxl_vram_mib is not None
                        else ' (unable to determine)'
                    ),
                    qxl_vram_mib is not None
                    and qxl_vram_mib >= MINIMUM_QXL_VRAM_MIB,
                    False,
                ))
            else:
                checks.append((
                    'Graphics adapter is not QXL',
                    True,
                    True,
                ))
            opengl_ok, opengl_detail = self._check_opengl()
            checks.append((
                f'OpenGL rendering ({opengl_detail})',
                opengl_ok,
                True,
            ))

        self.output.appendPlainText('\nPRE-FLIGHT CHECK')
        required_ok = True
        for label, passed, required in checks:
            if required and not passed:
                required_ok = False
            requirement = 'required' if required else 'advisory'
            mark = 'PASS' if passed else ('FAIL' if required else 'WARN')
            self.output.appendPlainText(f'[{mark}] {label} ({requirement})')

        usage = shutil.disk_usage(self.workspace if self.workspace.exists() else Path.home())
        free_gib = usage.free / (1024 ** 3)
        disk_ok = free_gib >= 5.0
        required_ok = required_ok and disk_ok
        self.output.appendPlainText(
            f'[{"PASS" if disk_ok else "FAIL"}] Free disk: {free_gib:.1f} GiB '
            '(5 GiB required)'
        )
        self.preflight_passed = required_ok
        result = 'PASSED' if required_ok else 'FAILED'
        self.output.appendPlainText(f'Preflight {result}.')
        self.statusBar().showMessage(f'Preflight {result}')
        self._update_controls()

    def _start_process(self, name, program, arguments, environment=None):
        if self._is_running(name):
            return
        process = QProcess(self)
        if environment:
            process_environment = QProcessEnvironment.systemEnvironment()
            for key, value in environment.items():
                process_environment.insert(key, str(value))
            process.setProcessEnvironment(process_environment)
        process.setWorkingDirectory(str(self.workspace))
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda p=process: self._read_process_output(p)
        )
        process.started.connect(
            lambda n=name: self._process_started(n)
        )
        process.finished.connect(
            lambda code, status, n=name: self._process_finished(n, code, status)
        )
        process.errorOccurred.connect(
            lambda error, n=name: self._process_error(n, error)
        )
        self.processes[name] = process
        self.output.appendPlainText(
            f'Launching managed process: {program} {" ".join(arguments)}'
        )
        process.start(program, arguments)
        self._update_controls()

    def _read_process_output(self, process):
        data = bytes(process.readAllStandardOutput()).decode(errors='replace')
        if data:
            self.output.appendPlainText(data.rstrip())

    def _process_started(self, name):
        self.output.appendPlainText(f'{name}: started')
        self.statusBar().showMessage(f'{name} running')
        self._update_controls()

    def _process_finished(self, name, exit_code, _exit_status):
        self.output.appendPlainText(f'{name}: exited with code {exit_code}')
        if name == 'recording':
            self._finalize_recording(exit_code)
        if name == 'autonomous_exploration':
            self.exploration_state = 'STOPPED'
            self.exploration_paused = False
            self.exploration_emergency_stopped = False
        if self.combined_wall_mapping and name in {
            'wall_follower', 'wandering_mapper',
        }:
            partner = (
                'wandering_mapper' if name == 'wall_follower'
                else 'wall_follower'
            )
            self.combined_wall_mapping = False
            self._request_stop(partner)
        self.statusBar().showMessage(f'{name} stopped')
        self._update_controls()

    def _process_error(self, name, error):
        self.output.appendPlainText(f'{name}: process error {error}')
        self._update_controls()

    def start_robot(self):
        if not self.preflight_passed or not self._is_real_robot_role():
            return
        answer = QMessageBox.warning(
            self,
            'Start real robot?',
            'Confirm the robot is raised or in a clear supervised area, the '
            'physical emergency stop works, and nobody is in its path.',
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Ok:
            return
        self._start_process(
            'robot',
            'ros2',
            [
                'launch',
                'generic_motor_driver',
                'complete_robot.launch.py',
                'rviz:=false',
            ],
        )

    def _request_stop(self, name):
        process = self.processes.get(name)
        if process is None or process.state() == QProcess.NotRunning:
            return
        process.terminate()
        QTimer.singleShot(3000, lambda p=process: self._kill_if_running(p))

    @staticmethod
    def _kill_if_running(process):
        if process.state() != QProcess.NotRunning:
            process.kill()

    def stop_robot(self):
        self._begin_zero_velocity_burst()
        self._request_stop('xbox_teleop')
        self._request_stop('robot')
        self.statusBar().showMessage('Stopping robot stack')

    def start_xbox_teleop(self):
        if (
            not self.preflight_passed
            or not self._is_real_robot_role()
            or not self._is_running('robot')
            or self._is_running('mapping')
            or self._is_running('xbox_teleop')
        ):
            return
        if not Path('/dev/input/js0').exists():
            QMessageBox.critical(
                self,
                'Joystick unavailable',
                'Xbox teleoperation requires /dev/input/js0.',
            )
            return
        self._start_process(
            'xbox_teleop',
            'ros2',
            [
                'launch',
                'teleop_twist_joy',
                'teleop-launch.py',
                'joy_config:=xbox',
            ],
        )

    def stop_xbox_teleop(self):
        self._begin_zero_velocity_burst()
        self._request_stop('xbox_teleop')
        self.statusBar().showMessage('Stopping Xbox teleoperation')

    def start_mapping(self):
        if (
            not self.preflight_passed
            or not self._is_real_robot_role()
            or self._is_running('robot')
            or self._is_running('mapping')
        ):
            return
        if not Path('/dev/lidar').exists():
            QMessageBox.critical(
                self,
                'LiDAR unavailable',
                'Mapping requires the configured LiDAR device at /dev/lidar.',
            )
            return
        answer = QMessageBox.warning(
            self,
            'Start real-robot mapping?',
            'This starts the motor, relay, LiDAR, SLAM, Nav2, joystick, and RViz '
            'stack. Confirm a clear supervised area and working physical stop.',
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Ok:
            return
        self._start_process(
            'mapping',
            'ros2',
            [
                'launch',
                'generic_motor_driver',
                'complete_robot_mapping.launch.py',
                'use_nav2:=true',
                'rviz:=true',
            ],
        )

    def stop_mapping(self):
        self.stop_wandering_mapper()
        self._begin_zero_velocity_burst()
        self._request_stop('mapping')
        self.statusBar().showMessage('Stopping mapping stack')

    def start_autonomous_exploration(self):
        if self._is_running('autonomous_exploration'):
            return
        if not self._machine_selected():
            QMessageBox.information(
                self,
                'Select a machine role',
                'Select Real Robot or Local Simulation, then run Preflight Check.',
            )
            return
        if not self.preflight_passed:
            QMessageBox.information(
                self,
                'Preflight required',
                'Run Preflight Check and resolve every required failure before '
                'starting autonomous motion.',
            )
            return
        if self.autonomy_workspace is None:
            QMessageBox.critical(
                self,
                'Autonomy workspace unavailable',
                'The manager could not find autonomous_exploration.launch.py. '
                'Set BIGSWEEP_WORKSPACE or place the workspace at '
                '~/BigSweepLogic.',
            )
            return
        real_robot = self._is_real_robot_role()
        simulation_running = self._is_running('simulation')
        conflicting = [
            name for name in (
                'robot', 'mapping', 'simulation', 'wall_follower', 'replay'
            )
            if self._is_running(name)
            and not (name == 'simulation' and not real_robot)
        ]
        if conflicting:
            QMessageBox.warning(
                self,
                'Stop conflicting processes',
                'Stop these managed processes before starting autonomous '
                f'exploration: {", ".join(conflicting)}.',
            )
            return
        setup_file = self.autonomy_workspace / 'install' / 'setup.bash'
        if not setup_file.is_file():
            QMessageBox.critical(
                self,
                'Autonomy workspace is not built',
                f'Build {self.autonomy_workspace} before starting exploration.',
            )
            return
        if real_robot and not Path('/dev/lidar').exists():
            QMessageBox.critical(
                self,
                'LiDAR unavailable',
                'Autonomous exploration requires /dev/lidar.',
            )
            return
        target = 'physical robot' if real_robot else 'Gazebo simulation'
        simulation_description = (
            'The exploration behaviors will attach to the running Gazebo and '
            'Nav2 stack.' if simulation_running else
            'Gazebo and RViz will start as part of the managed process.'
        )
        answer = QMessageBox.warning(
            self,
            'Start autonomous exploration?',
            f'This starts autonomous exploration on the {target}, including '
            'SLAM, right-wall following, frontier selection, and Nav2. '
            + (
                'The robot will move after readiness checks pass. Confirm a '
                'closed clear area and keep the physical emergency stop within reach.'
                if real_robot else simulation_description
            ),
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Ok:
            return
        launch_file = (
            'autonomous_exploration.launch.py'
            if real_robot else
            'autonomous_exploration_sim.launch.py'
        )
        autonomy_arguments = ' '.join(self._autonomous_launch_arguments())
        command = (
            f'source {shlex.quote(str(setup_file))} && '
            'exec ros2 launch generic_motor_driver '
            f'{launch_file} {autonomy_arguments}'
        )
        if not real_robot and simulation_running:
            command += (
                ' attach_to_simulation:=True use_rviz:=False '
                'wall_command_topic:=/cmd_vel_nav'
            )
        self.exploration_state = 'STARTING'
        self.exploration_paused = False
        self.exploration_emergency_stopped = False
        self._start_process(
            'autonomous_exploration', 'bash', ['-lc', command]
        )

    def _call_exploration_service(self, operation, service, value):
        if not self._is_running('autonomous_exploration'):
            return
        self._start_process(
            f'exploration_{operation}',
            'ros2',
            [
                'service', 'call', service, 'std_srvs/srv/SetBool',
                f'{{data: {str(value).lower()}}}',
            ],
        )

    def set_exploration_paused(self, paused):
        self._call_exploration_service(
            'pause' if paused else 'resume',
            '/exploration_supervisor/set_paused',
            paused,
        )
        self.statusBar().showMessage(
            'Requesting exploration pause' if paused
            else 'Requesting exploration resume'
        )

    def set_exploration_emergency_stop(self, stopped):
        self._begin_zero_velocity_burst()
        self._call_exploration_service(
            'estop' if stopped else 'clear_estop',
            '/exploration_supervisor/set_emergency_stop',
            stopped,
        )
        self.statusBar().showMessage(
            'AUTONOMOUS EMERGENCY STOP REQUESTED' if stopped
            else 'Requesting autonomous emergency-stop reset'
        )

    def stop_autonomous_exploration(self):
        if not self._is_running('autonomous_exploration'):
            return
        self._begin_zero_velocity_burst()
        self._call_exploration_service(
            'stop', '/exploration_supervisor/set_enabled', False
        )
        QTimer.singleShot(
            750, lambda: self._request_stop('autonomous_exploration')
        )
        self.statusBar().showMessage('Stopping autonomous exploration stack')

    def _safe_map_name(self):
        name = self.map_name_input.text().strip()
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', name):
            return None
        return name

    def save_map(self):
        if (
            not (
                self._is_running('mapping')
                or self._is_running('simulation')
                or self._is_running('autonomous_exploration')
            )
            or self._is_running('save_map')
        ):
            return
        map_name = self._safe_map_name()
        if map_name is None:
            QMessageBox.warning(
                self,
                'Invalid map name',
                'Use 1–64 letters, numbers, underscores, or hyphens; start '
                'with a letter or number.',
            )
            return
        maps_dir = self.workspace / 'maps'
        maps_dir.mkdir(parents=True, exist_ok=True)
        map_prefix = maps_dir / map_name
        existing = [
            map_prefix.with_suffix('.yaml'),
            map_prefix.with_suffix('.pgm'),
        ]
        if any(path.exists() for path in existing):
            answer = QMessageBox.warning(
                self,
                'Replace existing map?',
                f'A map named {map_name} already exists. Replace its files?',
                QMessageBox.Yes | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            if answer != QMessageBox.Yes:
                return
        self._start_process(
            'save_map',
            'ros2',
            ['run', 'nav2_map_server', 'map_saver_cli', '-f', str(map_prefix)],
        )
        self.output.appendPlainText(f'Saving map: {map_prefix}')

    def start_wandering_mapper(self):
        if (
            not self.preflight_passed
            or self._is_running('wandering_mapper')
            or not (
                self._is_running('mapping')
                or self._is_running('simulation')
            )
        ):
            return
        real_robot = self._is_real_robot_role()
        warning = (
            'The real robot will autonomously navigate toward unexplored map '
            'frontiers. Keep it supervised in a closed, clear area with the '
            'physical emergency stop in hand.'
            if real_robot else
            'The simulated robot will autonomously navigate toward unexplored '
            'map frontiers using Nav2.'
        )
        answer = QMessageBox.warning(
            self,
            'Start wandering mapper?',
            warning,
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Ok:
            return
        self._start_process(
            'wandering_mapper', 'ros2', [
                'launch', 'create2_demo', 'wandering_mapping.launch.py',
                *self._planner_launch_arguments(),
            ],
        )

    def stop_wandering_mapper(self):
        if not self._is_running('wandering_mapper'):
            return
        self._request_stop('wandering_mapper')
        if self.combined_wall_mapping:
            self.combined_wall_mapping = False
            self._request_stop('wall_follower')
        self._begin_zero_velocity_burst()
        self.statusBar().showMessage('Stopping wandering mapper')

    def start_wall_follower(self):
        if (
            not self.preflight_passed
            or self._is_real_robot_role()
            or self._is_running('robot')
            or self._is_running('mapping')
            or self._is_running('wall_follower')
        ):
            return
        answer = QMessageBox.warning(
            self,
            'Start coordinated wall mapping?',
            'This will bring up the corridor Gazebo world, RViz, SLAM, Nav2, '
            'chassis-filtered LiDAR, the OpenCV global frontier planner, and '
            'clockwise wall tracing. Each behavior yields control to the other.',
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Ok:
            return
        if not self._is_running('simulation'):
            self.start_simulation()
            QTimer.singleShot(1500, self._start_wall_follower_process)
        else:
            self._start_wall_follower_process()

    def _start_wall_follower_process(self):
        if (
            not self._is_running('simulation')
            or self._is_running('wall_follower')
        ):
            return
        if not self._is_running('wandering_mapper'):
            self._start_process(
                'wandering_mapper', 'ros2', [
                    'launch', 'create2_demo', 'wandering_mapping.launch.py',
                    *self._planner_launch_arguments(),
                ],
            )
        self.combined_wall_mapping = True
        wall_keys = (
            'target_wall_distance', 'inside_corner_front_clearance',
            'linear_speed', 'heading_gain',
            'distance_gain', 'turn_speed', 'front_stop_distance',
            'wall_timeout', 'nav2_wall_follow_distance',
            'wall_fit_inlier_distance', 'wall_fit_max_range',
            'wall_fit_minimum_span', 'minimum_handoff_wall_length',
            'handoff_heading_tolerance', 'handoff_distance_tolerance',
            'alignment_settle_time', 'alignment_escape_clearance',
            'outside_corner_loss_margin',
            'outside_corner_confirmation_time', 'outside_corner_turn_angle',
            'outside_corner_linear_speed', 'outside_corner_turn_speed',
            'outside_corner_timeout', 'dead_end_priority_multiplier',
            'revisited_right_turn_radius',
            'revisited_right_turn_minimum_age',
            'revisited_right_turn_clearance', 'right_turn_commitment_time',
            'emergency_front_distance',
            'minimum_corridor_width', 'corridor_lookahead',
            'narrow_corridor_confirmation_time',
            'dead_end_minimum_depth', 'dead_end_side_max_distance',
            'exploration_evaluation_period', 'minimum_free_area_gain',
            'loop_return_radius', 'loop_minimum_age',
            'retrace_radius', 'retrace_minimum_age',
            'retrace_minimum_length', 'retrace_overlap_ratio',
            'global_planning_cooldown',
        )
        wall_parameters = []
        for key in wall_keys:
            wall_parameters.extend([
                '-p', f'{key}:={self._configuration_value(key)}'
            ])
        self._start_process(
            'wall_follower',
            'ros2',
            [
                'run',
                'create2_demo',
                'clockwise_wall_tracer.py',
                '--ros-args',
                '-p', 'use_sim_time:=true',
                *wall_parameters,
            ],
        )

    def stop_wall_follower(self):
        if not self._is_running('wall_follower'):
            return
        self._request_stop('wall_follower')
        if self.combined_wall_mapping:
            self.combined_wall_mapping = False
            self._request_stop('wandering_mapper')
        self._begin_zero_velocity_burst()
        self.statusBar().showMessage('Stopping wall follower')

    def start_simulation(self):
        if (
            self._is_real_robot_role()
            or not self.preflight_passed
            or self._is_running('simulation')
            or self._is_running('robot')
            or self._is_running('mapping')
        ):
            return
        if self.autonomy_workspace is None:
            QMessageBox.critical(
                self,
                'Simulation workspace unavailable',
                'The manager could not find the BigSweep workspace. Set '
                'BIGSWEEP_WORKSPACE or place it at ~/BigSweepLogic.',
            )
            return
        setup_file = self.autonomy_workspace / 'install' / 'setup.bash'
        if not setup_file.is_file():
            QMessageBox.critical(
                self,
                'Simulation workspace is not built',
                f'Build {self.autonomy_workspace} before starting simulation.',
            )
            return
        world_name = self.world_selector.currentText()
        world_file = (
            self.autonomy_workspace
            / 'install/generic_motor_driver/share/generic_motor_driver/worlds'
            / SIMULATION_WORLDS[world_name]
        )
        if not world_file.is_file():
            QMessageBox.critical(
                self,
                'Simulation world unavailable',
                f'The selected world is not installed: {world_file}',
            )
            return
        command = (
            f'source {shlex.quote(str(setup_file))} && '
            'exec ros2 launch generic_motor_driver '
            'rectangular_classic_nav2_sim.launch.py '
            'headless:=True use_rviz:=False slam:=True '
            f'{shlex.quote(f"world:={world_file}")} '
            f'coverage_robot_width:={self._coverage_value("robot_width")} '
            f'coverage_operation_width:={self._coverage_value("operation_width")} '
            f'coverage_turning_radius:={self._coverage_value("turning_radius")}'
        )
        self._start_process(
            'simulation',
            'bash',
            ['-lc', command],
        )
        if self.gazebo_gui_switch.isChecked():
            QTimer.singleShot(3000, self.open_gazebo)
        if self.rviz_gui_switch.isChecked():
            QTimer.singleShot(3000, self.open_rviz)

    def stop_simulation(self):
        self.stop_coverage()
        if self._is_running('autonomous_exploration'):
            self.stop_autonomous_exploration()
        self.stop_wandering_mapper()
        self.stop_wall_follower()
        self._request_stop('replay')
        self._request_stop('gazebo_client')
        self._request_stop('rviz')
        self._request_stop('simulation')
        self.statusBar().showMessage('Stopping simulation and replay')

    def start_coverage(self):
        if not self._is_running('simulation') or self._is_running('coverage'):
            return
        x_min = self._coverage_value('field_x_min')
        x_max = self._coverage_value('field_x_max')
        y_min = self._coverage_value('field_y_min')
        y_max = self._coverage_value('field_y_max')
        if x_min >= x_max or y_min >= y_max:
            QMessageBox.warning(
                self,
                'Invalid coverage field',
                'Each field minimum must be smaller than its maximum.',
            )
            return
        setup_file = self.autonomy_workspace / 'install' / 'setup.bash'
        parameters = {
            'field_x_min': x_min,
            'field_x_max': x_max,
            'field_y_min': y_min,
            'field_y_max': y_max,
            'route_type': self._coverage_value('route_type'),
            'continuity_type': self._coverage_value('continuity_type'),
            'path_type': self._coverage_value('path_type'),
        }
        ros_arguments = ' '.join(
            f'-p {shlex.quote(f"{key}:={value}")}'
            for key, value in parameters.items()
        )
        command = (
            f'source {shlex.quote(str(setup_file))} && '
            'exec ros2 run generic_motor_driver '
            'opennav_coverage_executor.py --ros-args '
            f'{ros_arguments}'
        )
        self._start_process('coverage', 'bash', ['-lc', command])
        self.statusBar().showMessage('Starting complete coverage')

    def stop_coverage(self):
        if not self._is_running('coverage'):
            return
        self._request_stop('coverage')
        self._begin_zero_velocity_burst()
        self.statusBar().showMessage('Canceling coverage')

    @staticmethod
    def _resolve_bag_directory(selected_path):
        selected = Path(selected_path)
        if (selected / 'metadata.yaml').is_file():
            return selected
        nested_bag = selected / 'bag'
        if (nested_bag / 'metadata.yaml').is_file():
            return nested_bag
        return None

    def toggle_replay(self):
        if self._is_running('replay'):
            self._request_stop('replay')
            return
        if (
            self._is_real_robot_role()
            or not self.preflight_passed
            or self._is_running('recording')
            or self._is_running('robot')
            or self._is_running('mapping')
        ):
            return
        selected = QFileDialog.getExistingDirectory(
            self,
            'Select a run folder or rosbag folder',
            str(self.workspace / 'runs'),
        )
        if not selected:
            return
        bag_directory = self._resolve_bag_directory(selected)
        if bag_directory is None:
            QMessageBox.warning(
                self,
                'No rosbag found',
                'Select a run containing bag/metadata.yaml or select the bag '
                'directory itself.',
            )
            return
        answer = QMessageBox.warning(
            self,
            'Replay recorded topics?',
            'Replay can publish recorded velocity, sensor, odometry, and TF '
            'topics. Continue only on an isolated development ROS domain.',
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Ok:
            return
        self._start_process(
            'replay',
            'ros2',
            ['bag', 'play', str(bag_directory)],
        )
        self.output.appendPlainText(f'Replaying bag: {bag_directory}')

    def _select_run(self, title):
        selected = QFileDialog.getExistingDirectory(
            self,
            title,
            str(self.workspace / 'runs'),
        )
        if not selected:
            return None
        run = Path(selected).resolve()
        if not (run / 'metadata.json').is_file():
            QMessageBox.warning(
                self,
                'Invalid run folder',
                'Select a run folder containing metadata.json.',
            )
            return None
        return run

    def validate_run(self):
        run = self._select_run('Select a run to validate')
        if run is None:
            return
        self._start_process(
            'validation',
            'ros2',
            ['run', 'robot_run_manager', 'validate_run', str(run)],
        )
        self.output.appendPlainText(f'Validating and hashing: {run}')

    @staticmethod
    def _valid_remote_settings(host, remote_path):
        host_ok = bool(re.fullmatch(r'[A-Za-z0-9_.@-]+', host))
        path_ok = bool(re.fullmatch(r'[A-Za-z0-9_./~-]+', remote_path))
        return host_ok and path_ok

    def transfer_run(self):
        host = self.remote_host_input.text().strip()
        remote_path = self.remote_path_input.text().strip().rstrip('/')
        if not self._valid_remote_settings(host, remote_path):
            QMessageBox.warning(
                self,
                'Invalid transfer settings',
                'Enter an SSH host such as user@computer and a remote path '
                'without spaces or shell characters.',
            )
            return
        ssh_command = 'ssh -o BatchMode=yes -o ConnectTimeout=8'
        common = ['-av', '--partial', '--checksum', '-e', ssh_command]
        if self.transfer_direction.currentIndex() == 0:
            run = self._select_run('Select a run to upload')
            if run is None:
                return
            source = f'{run}/'
            destination = f'{host}:{remote_path}/{run.name}/'
            description = f'Upload {run.name} to {host}'
        else:
            run_name = self.remote_run_input.text().strip()
            if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', run_name):
                QMessageBox.warning(
                    self,
                    'Invalid run name',
                    'Enter the exact remote run-folder name.',
                )
                return
            local_run = self.workspace / 'runs' / run_name
            if local_run.exists():
                answer = QMessageBox.warning(
                    self,
                    'Merge with local run?',
                    f'{local_run} exists. Continue without deleting local files?',
                    QMessageBox.Yes | QMessageBox.Cancel,
                    QMessageBox.Cancel,
                )
                if answer != QMessageBox.Yes:
                    return
            local_run.mkdir(parents=True, exist_ok=True)
            source = f'{host}:{remote_path}/{run_name}/'
            destination = f'{local_run}/'
            description = f'Download {run_name} from {host}'
        answer = QMessageBox.question(
            self,
            'Confirm run transfer',
            f'{description}? SSH keys must already be configured.',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return
        self._start_process(
            'transfer',
            'rsync',
            [*common, source, destination],
        )

    def _run_git(self, arguments):
        try:
            result = subprocess.run(
                ['git', '-C', str(self.workspace), *arguments],
                capture_output=True,
                check=False,
                text=True,
                timeout=120,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.output.appendPlainText(f'Git failed: {exc}')
            return None
        combined = '\n'.join(
            part.strip() for part in (result.stdout, result.stderr) if part.strip()
        )
        if combined:
            self.output.appendPlainText(combined)
        return result

    def download_code(self):
        status = self._run_git(['status', '--porcelain'])
        if status is None or status.returncode != 0:
            return
        if status.stdout.strip():
            QMessageBox.warning(
                self,
                'Local changes present',
                'Code download is blocked until local changes are committed or '
                'otherwise handled. No files were changed.',
            )
            return
        answer = QMessageBox.question(
            self,
            'Download code?',
            'Run git pull --ff-only from origin for the current branch?',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return
        result = self._run_git(['pull', '--ff-only'])
        if result is not None:
            self.statusBar().showMessage(
                'Code download complete' if result.returncode == 0
                else 'Code download failed; see output'
            )

    def upload_code(self):
        message = self.commit_message_input.text().strip()
        if not message:
            QMessageBox.warning(
                self,
                'Commit message required',
                'Enter a concise Git commit message before uploading code.',
            )
            return
        staged = self._run_git(['diff', '--cached', '--name-only'])
        if staged is None or staged.returncode != 0:
            return

        def allowed(path):
            return (
                path == '.gitignore'
                or path.startswith('src/robot_run_manager/')
            )
        unexpected = [
            path for path in staged.stdout.splitlines() if not allowed(path)
        ]
        if unexpected:
            QMessageBox.critical(
                self,
                'Unrelated staged changes',
                'Upload is blocked because unrelated files are staged:\n'
                + '\n'.join(unexpected),
            )
            return
        answer = QMessageBox.question(
            self,
            'Commit and upload utility?',
            'Stage only .gitignore and src/robot_run_manager, commit them, then '
            'push the current branch to origin?',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return
        add_result = self._run_git([
            'add', '--', '.gitignore', 'src/robot_run_manager',
        ])
        if add_result is None or add_result.returncode != 0:
            return
        commit_result = self._run_git(['commit', '-m', message])
        if commit_result is None or commit_result.returncode != 0:
            return
        push_result = self._run_git(['push', 'origin', 'HEAD'])
        if push_result is not None:
            self.statusBar().showMessage(
                'Code uploaded to origin' if push_result.returncode == 0
                else 'Push failed; commit remains local'
            )

    def view_logs(self):
        log_dir = self.workspace / 'log'
        log_dir.mkdir(parents=True, exist_ok=True)
        run = self._default_log_run()
        flag = self._latest_log_flag(run)
        anchor = self._log_anchor_time(run, flag)
        ros_logs = self._logs_near_time(anchor)
        report = log_dir / 'latest_run_log_context.txt'
        self._write_log_context_report(report, run, flag, ros_logs, anchor)
        self._open_log_report(report, flag)
        label = run.name if run is not None else 'latest ROS session'
        suffix = f' at flag "{flag["identifier"]}"' if flag else ''
        self.statusBar().showMessage(f'Viewing {label}{suffix}')

    def _open_log_report(self, report, flag):
        dialog = QDialog(self)
        dialog.setWindowTitle('Run Log Viewer')
        dialog.resize(1100, 760)
        layout = QVBoxLayout(dialog)
        location = QLabel(f'Focused report: {report}')
        location.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(location)
        search_row = QHBoxLayout()
        search_row.addWidget(QLabel('Find:'))
        search = QLineEdit()
        search.setPlaceholderText('log flag, warning, node, or timestamp')
        if flag:
            search.setText(flag.get('identifier', ''))
        search_row.addWidget(search, 1)
        find_next = QPushButton('Find Next')
        search_row.addWidget(find_next)
        open_folder = QPushButton('Open Log Folder')
        search_row.addWidget(open_folder)
        layout.addLayout(search_row)
        viewer = QPlainTextEdit()
        viewer.setReadOnly(True)
        viewer.setMaximumBlockCount(20000)
        viewer.setPlainText(report.read_text(encoding='utf-8', errors='replace'))
        layout.addWidget(viewer, 1)

        def find_text():
            term = search.text()
            if not term:
                return
            if not viewer.find(term):
                cursor = viewer.textCursor()
                cursor.movePosition(QTextCursor.Start)
                viewer.setTextCursor(cursor)
                viewer.find(term)

        find_next.clicked.connect(find_text)
        search.returnPressed.connect(find_text)
        open_folder.clicked.connect(
            lambda: subprocess.Popen(['xdg-open', str(report.parent)])
        )
        dialog.finished.connect(lambda: setattr(self, 'log_viewer_dialog', None))
        self.log_viewer_dialog = dialog
        dialog.show()
        if flag:
            find_text()

    def _default_log_run(self):
        if self.active_run is not None:
            return self.active_run
        runs_dir = self.workspace / 'runs'
        candidates = [
            path.parent for path in runs_dir.glob('*/metadata.json')
        ] if runs_dir.is_dir() else []
        return max(candidates, key=lambda path: path.stat().st_mtime) \
            if candidates else None

    def _latest_log_flag(self, run):
        flags_path = self.workspace / 'log/user_log_flags.csv'
        if not flags_path.is_file():
            return None
        with flags_path.open(newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
        if run is not None:
            matching = [row for row in rows if row.get('active_run') == str(run)]
            return matching[-1] if matching else None
        return rows[-1] if rows else None

    @staticmethod
    def _log_anchor_time(run, flag):
        if flag and flag.get('utc_timestamp'):
            try:
                return datetime.fromisoformat(flag['utc_timestamp']).timestamp()
            except ValueError:
                pass
        if run is not None:
            return run.stat().st_mtime
        launch_dirs = list((Path.home() / '.ros/log').glob('20*'))
        return max(path.stat().st_mtime for path in launch_dirs) \
            if launch_dirs else datetime.now().timestamp()

    @staticmethod
    def _logs_near_time(anchor):
        ros_log_dir = Path.home() / '.ros/log'
        candidates = [
            path for path in ros_log_dir.glob('*.log')
            if path.is_file() and abs(path.stat().st_mtime - anchor) <= 3600
        ]
        return sorted(
            candidates,
            key=lambda path: abs(path.stat().st_mtime - anchor),
        )[:16]

    @staticmethod
    def _write_log_context_report(report, run, flag, logs, anchor):
        lines = [
            'ROBOT RUN LOG CONTEXT',
            f'Run: {run if run is not None else "latest ROS session"}',
            f'Anchor time: {datetime.fromtimestamp(anchor).astimezone().isoformat()}',
        ]
        if flag:
            lines.extend([
                f'Flag: {flag.get("identifier", "")}',
                f'Flag UTC: {flag.get("utc_timestamp", "")}',
                f'Flag ROS nanoseconds: {flag.get("ros_time_nanoseconds", "")}',
            ])
        else:
            lines.append('Flag: none; showing context near the latest run activity')
        lines.append('')
        timestamp_pattern = re.compile(r'\[(\d{10}(?:\.\d+)?)\]')
        for path in logs:
            try:
                source_lines = path.read_text(
                    encoding='utf-8', errors='replace'
                ).splitlines()
            except OSError:
                continue
            timed = []
            for index, line in enumerate(source_lines):
                match = timestamp_pattern.search(line)
                if match:
                    timed.append((abs(float(match.group(1)) - anchor), index))
            center = min(timed)[1] if timed else max(0, len(source_lines) - 20)
            start = max(0, center - 12)
            end = min(len(source_lines), center + 13)
            lines.extend([
                '=' * 72,
                f'{path.name}  (lines {start + 1}-{end})',
                '=' * 72,
                *source_lines[start:end],
                '',
            ])
        report.write_text('\n'.join(lines) + '\n', encoding='utf-8')

    def flag_and_open_logs(self):
        identifier, accepted = QInputDialog.getText(
            self,
            'Flag logs',
            'Identifier for this point in the logs:',
        )
        identifier = identifier.strip()
        if not accepted:
            return
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_. -]{0,79}', identifier):
            QMessageBox.warning(
                self,
                'Invalid log identifier',
                'Use 1–80 letters, numbers, spaces, periods, underscores, or hyphens.',
            )
            return
        log_dir = self.workspace / 'log'
        log_dir.mkdir(parents=True, exist_ok=True)
        flags_path = log_dir / 'user_log_flags.csv'
        new_file = not flags_path.exists()
        with flags_path.open('a', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            if new_file:
                writer.writerow([
                    'utc_timestamp', 'ros_time_nanoseconds', 'identifier',
                    'active_run', 'managed_processes',
                ])
            writer.writerow([
                self._utc_now(),
                self.ros_node.get_clock().now().nanoseconds,
                identifier,
                str(self.active_run or ''),
                self.process_label.text(),
            ])
        if self.active_run is not None and self._is_running('recording'):
            events_path = self.active_run / 'events.csv'
            with events_path.open('a', newline='', encoding='utf-8') as stream:
                csv.writer(stream).writerow([
                    self._utc_now(),
                    self.ros_node.get_clock().now().nanoseconds,
                    f'log_flag:{identifier}',
                    self.operator_input.text().strip(),
                    self.scenario_input.text().strip(),
                ])
        self.output.appendPlainText(
            f'Log flag written: {identifier} ({flags_path})'
        )
        self.statusBar().showMessage(f'Logs flagged: {identifier}')
        self.view_logs()

    def rebuild_and_restart_manager(self):
        """Build the workspace after this process exits, then relaunch it."""
        active = [name for name in self.processes if self._is_running(name)]
        if active:
            QMessageBox.warning(
                self,
                'Managed processes are active',
                'Stop these processes before rebuilding: '
                + ', '.join(active),
            )
            return
        answer = QMessageBox.question(
            self,
            'Rebuild and restart?',
            'Save settings, close this Manager, rebuild the complete ROS '
            'workspace, and automatically open the updated Manager?',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Yes,
        )
        if answer != QMessageBox.Yes or not self._save_configuration():
            return
        script = (
            self.workspace
            / 'src/robot_run_manager/scripts/rebuild_and_restart.sh'
        )
        if not script.is_file():
            QMessageBox.critical(
                self,
                'Rebuild unavailable',
                'The rebuild script is missing.',
            )
            return
        rebuild_log = self.workspace / 'log/manager_rebuild.log'
        rebuild_log.parent.mkdir(parents=True, exist_ok=True)
        try:
            with rebuild_log.open('a', encoding='utf-8') as output:
                output.write(f'\n--- rebuild requested {self._utc_now()} ---\n')
                output.flush()
                subprocess.Popen(
                    [
                        'bash', str(script), str(self.workspace),
                        str(os.getpid()),
                    ],
                    cwd=self.workspace,
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,
                )
        except OSError as error:
            QMessageBox.critical(self, 'Could not start rebuild', str(error))
            return
        self.statusBar().showMessage('Closing for workspace rebuild')
        QTimer.singleShot(250, self.close)

    def install_desktop_launcher(self):
        answer = QMessageBox.question(
            self,
            'Install desktop launcher?',
            'Create or refresh the Robot Run Manager desktop icon and startup '
            'wrapper for this workspace?',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Yes,
        )
        if answer != QMessageBox.Yes:
            return
        try:
            from robot_run_manager.desktop import main as install_launcher
            result = install_launcher()
        except OSError as exc:
            QMessageBox.critical(
                self,
                'Launcher installation failed',
                str(exc),
            )
            return
        if result == 0:
            launcher = Path.home() / 'Desktop/Robot Run Manager.desktop'
            self.output.appendPlainText(f'Desktop launcher installed: {launcher}')
            QMessageBox.information(
                self,
                'Desktop launcher installed',
                f'Launcher created at:\n{launcher}\n\nIf Ubuntu asks, '
                'right-click it and select Allow Launching.',
            )

    def _git_revision(self):
        try:
            result = subprocess.run(
                ['git', '-C', str(self.workspace), 'rev-parse', 'HEAD'],
                capture_output=True,
                check=False,
                text=True,
                timeout=3,
            )
            return result.stdout.strip() if result.returncode == 0 else None
        except (OSError, subprocess.TimeoutExpired):
            return None

    @staticmethod
    def _utc_now():
        return datetime.now(timezone.utc).isoformat()

    def _write_metadata(self):
        if self.active_run is None or self.run_metadata is None:
            return
        metadata_path = self.active_run / 'metadata.json'
        temporary_path = self.active_run / 'metadata.json.tmp'
        temporary_path.write_text(
            json.dumps(self.run_metadata, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        temporary_path.replace(metadata_path)

    def _capture_calibration(self, run_dir):
        calibration_dir = run_dir / 'calibration'
        calibration_dir.mkdir()
        sources = {
            'nav2_params.yaml': (
                'src/create_robot/create_driver/config/nav2_params.yaml'
            ),
            'mapper_params_online_async.yaml': (
                'src/create_robot/create_driver/config/'
                'mapper_params_online_async.yaml'
            ),
            'complete_robot.launch.py': (
                'src/create_robot/create_driver/launch/complete_robot.launch.py'
            ),
            'robot.urdf.xacro': (
                'src/create_robot/create_description/urdf/'
                'rectangular_robot.urdf.xacro'
            ),
        }
        captured = []
        for destination_name, relative_source in sources.items():
            source = self.workspace / relative_source
            if source.is_file():
                shutil.copy2(source, calibration_dir / destination_name)
                captured.append(destination_name)
        return captured

    def start_recording(self):
        if not self.preflight_passed or self._is_running('recording'):
            return
        runs_dir = self.workspace / 'runs'
        runs_dir.mkdir(parents=True, exist_ok=True)
        run_id = datetime.now().astimezone().strftime('%Y-%m-%d-%H%M%S')
        run_dir = runs_dir / run_id
        suffix = 1
        while run_dir.exists():
            run_dir = runs_dir / f'{run_id}-{suffix:02d}'
            suffix += 1
        run_dir.mkdir()
        calibration_files = self._capture_calibration(run_dir)

        events_path = run_dir / 'events.csv'
        with events_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow([
                'utc_timestamp',
                'ros_time_nanoseconds',
                'event',
                'operator',
                'scenario',
            ])

        self.active_run = run_dir
        self.run_metadata = {
            'schema_version': 1,
            'run_id': run_dir.name,
            'status': 'recording',
            'started_utc': self._utc_now(),
            'ended_utc': None,
            'hostname': socket.gethostname(),
            'machine_role': self.role_selector.currentText(),
            'operator': self.operator_input.text().strip(),
            'scenario': self.scenario_input.text().strip(),
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'git_revision': self._git_revision(),
            'topics_requested': RECORD_TOPICS,
            'bag_directory': 'bag',
            'events_file': 'events.csv',
            'calibration_files': calibration_files,
        }
        self._write_metadata()
        self.run_label.setText(f'Active run: {run_dir}')
        self._start_process(
            'recording',
            'ros2',
            ['bag', 'record', '-o', str(run_dir / 'bag'), *RECORD_TOPICS],
        )

    def stop_recording(self):
        process = self.processes.get('recording')
        if process is None or process.state() == QProcess.NotRunning:
            return
        self.output.appendPlainText('Stopping recording cleanly...')
        try:
            os.kill(process.processId(), signal.SIGINT)
        except (OSError, ProcessLookupError):
            process.terminate()
        QTimer.singleShot(5000, lambda p=process: self._kill_if_running(p))

    def _finalize_recording(self, exit_code):
        if self.run_metadata is None:
            return
        self.run_metadata['status'] = (
            'complete' if exit_code == 0 else 'incomplete'
        )
        self.run_metadata['ended_utc'] = self._utc_now()
        self.run_metadata['recorder_exit_code'] = exit_code
        self._write_metadata()
        completed_run = self.active_run
        self.active_run = None
        self.run_metadata = None
        self.run_label.setText(f'Last run: {completed_run}')
        if self.close_pending:
            QTimer.singleShot(0, self.close)

    def mark_event(self, event_name):
        if self.active_run is None or not self._is_running('recording'):
            return
        events_path = self.active_run / 'events.csv'
        with events_path.open('a', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow([
                self._utc_now(),
                self.ros_node.get_clock().now().nanoseconds,
                event_name.lower().replace(' ', '_'),
                self.operator_input.text().strip(),
                self.scenario_input.text().strip(),
            ])
        self.output.appendPlainText(f'Event marked: {event_name}')
        self.statusBar().showMessage(f'Event marked: {event_name}')

    def _begin_zero_velocity_burst(self):
        self.stop_publish_count = 0
        self.stop_timer.start()
        self._publish_stop()

    def _publish_stop(self):
        self.cmd_vel_publisher.publish(Twist())
        self.stop_publish_count += 1
        if self.stop_publish_count >= 20:
            self.stop_timer.stop()

    def emergency_stop(self):
        self._begin_zero_velocity_burst()
        for name in list(self.processes):
            if name == 'recording':
                self.stop_recording()
            else:
                self._request_stop(name)
        self.output.appendPlainText(
            'EMERGENCY STOP: publishing zero velocity and stopping all '
            'GUI-managed processes. Use the physical stop for assured isolation.'
        )
        self.statusBar().showMessage('EMERGENCY STOP ACTIVATED')
        self.notice.setStyleSheet(
            'background: #f8d7da; border: 2px solid #b00020; padding: 8px; '
            'font-weight: bold;'
        )

    def kill_ros_processes(self):
        answer = QMessageBox.critical(
            self,
            'Kill all ROS processes?',
            'This publishes zero velocity, then force-kills ROS 2, Gazebo, '
            'RViz, Nav2, SLAM, lidar, teleop, and robot-driver processes for '
            'this user session. The GUI will remain open. Continue?',
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return
        self._begin_zero_velocity_burst()
        self.output.appendPlainText(
            'KILL ROS requested: sending zero velocity before stack cleanup.'
        )
        QTimer.singleShot(750, self._run_kill_ros_script)

    def _run_kill_ros_script(self):
        script = (
            self.workspace
            / 'src/create_robot/create_driver/scripts/kill_robot_stack.sh'
        )
        if not script.is_file():
            QMessageBox.critical(
                self,
                'Cleanup script missing',
                f'Cannot find {script}',
            )
            return
        self._start_process('kill_ros', 'bash', [str(script)])
        self.statusBar().showMessage('Force-stopping ROS processes')

    def open_rviz(self):
        if self._is_running('rviz'):
            return
        arguments = []
        if self._is_running('simulation'):
            if not self.rviz_gui_switch.isChecked():
                return
            rviz_config = (
                Path('/opt/ros')
                / os.environ.get('ROS_DISTRO', REQUIRED_ROS_DISTRO)
                / 'share/nav2_bringup/rviz/nav2_default_view.rviz'
            )
            if rviz_config.is_file():
                arguments.extend(['-d', str(rviz_config)])
            arguments.extend(['--ros-args', '-p', 'use_sim_time:=True'])
        self._start_process('rviz', 'rviz2', arguments)

    def open_gazebo(self):
        if (
            self._is_running('simulation')
            and self.gazebo_gui_switch.isChecked()
            and not self._is_running('gazebo_client')
        ):
            plugin = (
                self.workspace / 'install/create2_demo/lib/'
                'libgazebo_fps_limiter.so'
            )
            arguments = (
                ['--gui-client-plugin', str(plugin)] if plugin.is_file() else []
            )
            self._start_process(
                'gazebo_client',
                'gzclient',
                arguments,
                environment={
                    'ROBOT_GAZEBO_GUI_FPS':
                    self.gazebo_fps_selector.currentText(),
                },
            )

    def closeEvent(self, event):
        if self._is_running('recording'):
            if self.close_pending:
                event.ignore()
                return
            answer = QMessageBox.question(
                self,
                'Recording is active',
                'Stop the active recording and close?',
                QMessageBox.Yes | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return
            self.close_pending = True
            self.stop_recording()
            event.ignore()
            return
        if any([
            self._is_running('robot'),
            self._is_running('mapping'),
            self._is_running('autonomous_exploration'),
            self._is_running('wandering_mapper'),
            self._is_running('wall_follower'),
        ]):
            self._begin_zero_velocity_burst()
        for name in list(self.processes):
            if name != 'recording':
                self._request_stop(name)
        if self._save_configuration():
            self.output.appendPlainText('Configuration settings saved.')
        self.behavior_tree_timer.stop()
        self.ros_spin_timer.stop()
        self.supervisor_tree.shutdown()
        event.accept()


def main(args=None):
    rclpy.init(args=None)
    ros_node = rclpy.create_node('robot_run_manager_gui')
    app = QApplication(args if args is not None else sys.argv)
    window = RunManagerWindow(ros_node)
    window.show()
    result = app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
