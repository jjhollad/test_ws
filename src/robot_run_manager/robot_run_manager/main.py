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
import shutil
import signal
import socket
import subprocess
import sys

from geometry_msgs.msg import Twist
from PyQt5.QtCore import QProcess, QSettings, QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
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
    QVBoxLayout,
    QWidget,
)
import rclpy


ACTION_GROUPS = {
    'Safety and system': [
        'Preflight Check',
        'Start Robot',
        'Stop Robot',
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
        'Start Wandering Mapper',
        'Stop Wandering Mapper',
        'Start Wall Follower',
        'Stop Wall Follower',
        'Start Simulation',
        'Stop Simulation',
        'Replay Selected Run',
    ],
    'Data and code': [
        'Validate Selected Run',
        'Transfer Run',
        'Download Code',
        'Upload Code',
        'View Logs',
        'Install Desktop Launcher',
    ],
}

ACTION_TOOLTIPS = {
    'Preflight Check': 'Check ROS, workspace, hardware, display, and disk readiness.',
    'Start Robot': 'Start the guarded real-robot motor, relay, and state-publisher stack.',
    'Stop Robot': 'Publish zero velocity and stop the GUI-managed real-robot stack.',
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
    'Start Wandering Mapper': 'Start the OpenCV global frontier planner through Nav2.',
    'Stop Wandering Mapper': 'Cancel global frontier exploration and publish zero velocity.',
    'Start Wall Follower': (
        'Start coordinated mapping; Nav2 aligns to the nearest right wall, '
        'drives a straight 3 m parallel segment, then hands off to tracing.'
    ),
    'Stop Wall Follower': 'Stop both coordinated planners and publish zero velocity.',
    'Start Simulation': (
        'Start the corridor Gazebo world, rectangular robot, SLAM, Nav2, and RViz.'
    ),
    'Stop Simulation': 'Stop simulation, Gazebo window, replay, and autonomous behaviors.',
    'Replay Selected Run': 'Replay a selected rosbag on the isolated development ROS graph.',
    'Validate Selected Run': 'Validate files and create a summary, dataset split, and checksums.',
    'Transfer Run': 'Upload or download a run using resumable SSH/rsync transfer.',
    'Download Code': 'Pull code with Git fast-forward only; requires a clean working tree.',
    'Upload Code': 'Commit only this utility and .gitignore, then push to GitHub.',
    'View Logs': 'Open the workspace ROS build and runtime log directory.',
    'Install Desktop Launcher': 'Create or refresh the desktop icon and ROS startup wrapper.',
}

IMPLEMENTED_ACTIONS = {
    'Preflight Check',
    'Start Robot',
    'Stop Robot',
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
    'Replay Selected Run',
    'Validate Selected Run',
    'Transfer Run',
    'Download Code',
    'Upload Code',
    'View Logs',
    'Install Desktop Launcher',
    'Start Wandering Mapper',
    'Stop Wandering Mapper',
    'Start Wall Follower',
    'Stop Wall Follower',
}

RECORD_TOPICS = [
    '/cmd_vel',
    '/odom',
    '/scan',
    '/tf',
    '/tf_static',
    '/joint_states',
    '/relay_status',
]

TUNING_VARIABLES = {
    'target_wall_distance': (
        'Wall distance', 'Wall follower', 0.60, 1.50, 1.05, 2,
        'Desired distance from the robot center to the right wall. Larger '
        'values leave more chassis clearance.'
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
}


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


class RunManagerWindow(QMainWindow):
    """Run-manager window with initial safety and process controls."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.cmd_vel_publisher = ros_node.create_publisher(Twist, '/cmd_vel', 10)
        self.workspace = find_workspace()
        self.processes = {}
        self.buttons = {}
        self.preflight_passed = False
        self.stop_publish_count = 0
        self.active_run = None
        self.run_metadata = None
        self.close_pending = False
        self.combined_wall_mapping = False
        self.settings = QSettings('test_ws', 'Robot Run Manager')
        self.config_sliders = {}
        self.config_value_labels = {}

        self.stop_timer = QTimer(self)
        self.stop_timer.setInterval(50)
        self.stop_timer.timeout.connect(self._publish_stop)

        self.setWindowTitle('Robot Run Manager')
        self.resize(950, 760)
        self._build_ui()
        self._update_controls()

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
        self.role_selector.addItems(['Real robot', 'Development / simulation'])
        self.role_selector.currentIndexChanged.connect(self._role_changed)
        role_layout.addWidget(self.role_selector, 0, 1)
        self.process_label = QLabel('Managed processes: none')
        role_layout.addWidget(self.process_label, 0, 2)
        role_layout.setColumnStretch(2, 1)
        root_layout.addLayout(role_layout)

        run_layout = QGridLayout()
        run_layout.addWidget(QLabel('Operator:'), 0, 0)
        self.operator_input = QLineEdit()
        self.operator_input.setPlaceholderText('name or initials')
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
            root_layout.addWidget(group)

        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setMaximumBlockCount(2000)
        self.output.setPlaceholderText('Preflight and managed-process output')
        root_layout.addWidget(self.output, 1)

        self.buttons['Preflight Check'].clicked.connect(self.run_preflight)
        self.buttons['Start Robot'].clicked.connect(self.start_robot)
        self.buttons['Stop Robot'].clicked.connect(self.stop_robot)
        self.buttons['Emergency Stop'].clicked.connect(self.emergency_stop)
        self.buttons['Kill ROS Processes'].clicked.connect(
            self.kill_ros_processes
        )
        self.buttons['Kill ROS Processes'].setStyleSheet(
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
        self.buttons['Start Wandering Mapper'].clicked.connect(
            self.start_wandering_mapper
        )
        self.buttons['Stop Wandering Mapper'].clicked.connect(
            self.stop_wandering_mapper
        )
        self.buttons['Start Wall Follower'].clicked.connect(
            self.start_wall_follower
        )
        self.buttons['Stop Wall Follower'].clicked.connect(
            self.stop_wall_follower
        )
        self.buttons['Start Simulation'].clicked.connect(self.start_simulation)
        self.buttons['Stop Simulation'].clicked.connect(self.stop_simulation)
        self.buttons['Replay Selected Run'].clicked.connect(self.toggle_replay)
        self.buttons['Validate Selected Run'].clicked.connect(self.validate_run)
        self.buttons['Transfer Run'].clicked.connect(self.transfer_run)
        self.buttons['Download Code'].clicked.connect(self.download_code)
        self.buttons['Upload Code'].clicked.connect(self.upload_code)
        self.buttons['View Logs'].clicked.connect(self.view_logs)
        self.buttons['Install Desktop Launcher'].clicked.connect(
            self.install_desktop_launcher
        )

        tabs.addTab(self._build_configuration_tab(), 'Configuration')
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
        reset = QPushButton('Reset tuning defaults')
        reset.setToolTip('Restore every tuning slider to its documented default.')
        reset.clicked.connect(self._reset_configuration)
        page_layout.addWidget(reset)
        page_layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        return scroll

    def _configuration_changed(self, key, slider_value, scale, decimals):
        value = slider_value / scale
        self.config_value_labels[key].setText(f'{value:.{decimals}f}')
        self.settings.setValue(f'tuning/{key}', value)

    def _reset_configuration(self):
        for key, specification in TUNING_VARIABLES.items():
            default = specification[4]
            decimals = specification[5]
            self.config_sliders[key].setValue(round(default * 10 ** decimals))

    def _configuration_value(self, key):
        decimals = TUNING_VARIABLES[key][5]
        return self.config_sliders[key].value() / 10 ** decimals

    def _planner_launch_arguments(self):
        keys = (
            'information_weight', 'frontier_bonus', 'revisit_weight',
            'visited_radius', 'travel_weight', 'forward_weight',
            'reverse_penalty', 'clearance_weight',
            'route_extension_period',
        )
        return [f'{key}:={self._configuration_value(key)}' for key in keys]

    def _role_changed(self):
        self.preflight_passed = False
        self.statusBar().showMessage('Role changed — run preflight again')
        self._update_controls()

    def _update_controls(self):
        running = self._is_running('robot')
        recording = self._is_running('recording')
        mapping = self._is_running('mapping')
        saving_map = self._is_running('save_map')
        simulation = self._is_running('simulation')
        replaying = self._is_running('replay')
        wandering = self._is_running('wandering_mapper')
        wall_following = self._is_running('wall_follower')
        gazebo_client = self._is_running('gazebo_client')
        validating = self._is_running('validation')
        transferring = self._is_running('transfer')
        real_robot = self.role_selector.currentIndex() == 0
        for action, button in self.buttons.items():
            button.setEnabled(action in IMPLEMENTED_ACTIONS)
            if action not in IMPLEMENTED_ACTIONS:
                button.setToolTip('Planned for a later step')
        self.buttons['Start Robot'].setEnabled(
            real_robot and self.preflight_passed and not running and not mapping
        )
        self.buttons['Stop Robot'].setEnabled(running)
        self.buttons['Emergency Stop'].setEnabled(True)
        self.buttons['Kill ROS Processes'].setEnabled(True)
        self.buttons['Open Gazebo'].setEnabled(simulation and not gazebo_client)
        self.buttons['Start Recording'].setEnabled(
            self.preflight_passed and not recording
        )
        self.buttons['Stop Recording'].setEnabled(recording)
        for event_name in ('Goal', 'Collision', 'Near Miss', 'Intervention'):
            self.buttons[f'Mark {event_name}'].setEnabled(recording)
        self.buttons['Start Mapping'].setEnabled(
            real_robot and self.preflight_passed and not running and not mapping
        )
        self.buttons['Stop Mapping'].setEnabled(mapping)
        self.buttons['Save Map'].setEnabled(
            (mapping or simulation) and not saving_map
        )
        self.buttons['Start Wandering Mapper'].setEnabled(
            self.preflight_passed
            and (mapping or simulation)
            and not wandering
            and not wall_following
            and not replaying
        )
        self.buttons['Stop Wandering Mapper'].setEnabled(wandering)
        self.buttons['Start Wall Follower'].setEnabled(
            self.preflight_passed
            and not real_robot
            and not running
            and not mapping
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
        self.buttons['Download Code'].setEnabled(not any([
            running, recording, mapping, simulation, replaying,
        ]))
        self.buttons['Upload Code'].setEnabled(not any([
            running, recording, mapping, simulation, replaying,
        ]))
        self.buttons['View Logs'].setEnabled(True)
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
        if wandering:
            active.append('wandering mapper')
        if wall_following:
            active.append('wall follower')
        if gazebo_client:
            active.append('Gazebo window')
        if validating:
            active.append('validation')
        if transferring:
            active.append('transfer')
        self.role_selector.setEnabled(not any([
            running, recording, mapping, simulation, replaying, wandering,
            wall_following,
        ]))
        self.process_label.setText(
            f'Managed processes: {", ".join(active)} running' if active
            else 'Managed processes: none'
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

    def run_preflight(self):
        real_robot = self.role_selector.currentIndex() == 0
        checks = [
            ('ROS 2 command', shutil.which('ros2') is not None, True),
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
                ('Gazebo package', self._check_ros_package('gazebo_ros'), True),
                ('Nav2 package', self._check_ros_package('nav2_bringup'), True),
                (
                    'Merged create2_demo package',
                    self._check_ros_package('create2_demo'),
                    True,
                ),
            ])

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

    def _start_process(self, name, program, arguments):
        if self._is_running(name):
            return
        process = QProcess(self)
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
        if not self.preflight_passed or self.role_selector.currentIndex() != 0:
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
        self._request_stop('robot')
        self.statusBar().showMessage('Stopping robot stack')

    def start_mapping(self):
        if (
            not self.preflight_passed
            or self.role_selector.currentIndex() != 0
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
        real_robot = self.role_selector.currentIndex() == 0
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
            or self.role_selector.currentIndex() == 0
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
        self._start_process(
            'wall_follower',
            'ros2',
            [
                'run',
                'create2_demo',
                'clockwise_wall_tracer.py',
                '--ros-args',
                '-p', 'use_sim_time:=true',
                '-p', 'target_wall_distance:='
                f'{self._configuration_value("target_wall_distance")}',
                '-p', 'linear_speed:='
                f'{self._configuration_value("linear_speed")}',
                '-p', 'nav2_wall_follow_distance:='
                f'{self._configuration_value("nav2_wall_follow_distance")}',
                '-p', 'wall_fit_inlier_distance:='
                f'{self._configuration_value("wall_fit_inlier_distance")}',
                '-p', 'minimum_handoff_wall_length:='
                f'{self._configuration_value("minimum_handoff_wall_length")}',
                '-p', 'exploration_evaluation_period:=25.0',
                '-p', 'global_planning_cooldown:=300.0',
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
            self.role_selector.currentIndex() == 0
            or not self.preflight_passed
            or self._is_running('simulation')
            or self._is_running('robot')
            or self._is_running('mapping')
        ):
            return
        self._start_process(
            'simulation',
            'ros2',
            [
                'launch',
                'create2_demo',
                'rectangular_robot_demo.launch.py',
                'headless:=True',
                'use_rviz:=True',
                'start_wall_follower:=false',
            ],
        )
        QTimer.singleShot(3000, self.open_gazebo)

    def stop_simulation(self):
        self.stop_wandering_mapper()
        self.stop_wall_follower()
        self._request_stop('replay')
        self._request_stop('gazebo_client')
        self._request_stop('simulation')
        self.statusBar().showMessage('Stopping simulation and replay')

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
            self.role_selector.currentIndex() == 0
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
        target = self.workspace / 'log'
        target.mkdir(parents=True, exist_ok=True)
        self._start_process('logs', 'xdg-open', [str(target)])

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
        self._start_process('rviz', 'rviz2', [])

    def open_gazebo(self):
        if (
            self._is_running('simulation')
            and not self._is_running('gazebo_client')
        ):
            self._start_process('gazebo_client', 'gzclient', [])

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
            self._is_running('wandering_mapper'),
            self._is_running('wall_follower'),
        ]):
            self._begin_zero_velocity_burst()
        for name in list(self.processes):
            if name != 'recording':
                self._request_stop(name)
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
