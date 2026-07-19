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

"""Inspectable py_trees_ros navigation supervisor used by the Manager."""

import py_trees
import py_trees_ros


class StateBehaviour(py_trees.behaviour.Behaviour):
    """Report a status from the Manager's current state dictionary."""

    def __init__(self, name, state_provider, key, active_status,
                 detail='', setting_keys=(), expected=True):
        super().__init__(name=name)
        self.state_provider = state_provider
        self.key = key
        self.active_status = active_status
        self.detail = detail
        self.setting_keys = tuple(setting_keys)
        self.expected = expected

    def update(self):
        value = self.state_provider().get(self.key, False)
        active = (
            value in self.expected
            if isinstance(self.expected, tuple)
            else value == self.expected
        )
        self.feedback_message = 'active' if active else 'inactive'
        return self.active_status if active else py_trees.common.Status.FAILURE


class SupervisorTree:
    """Construct and publish the non-commanding navigation supervisor tree."""

    def __init__(self, node, state_provider):
        success = py_trees.common.Status.SUCCESS
        running = py_trees.common.Status.RUNNING

        safety = py_trees.composites.Selector('Safety Gate', memory=False)
        safety.add_children([
            StateBehaviour('Emergency Stop Active', state_provider, 'emergency', running),
            StateBehaviour('System Ready', state_provider, 'system_ready', success),
            StateBehaviour('Awaiting Preflight', state_provider, 'always', running),
        ])

        localization = py_trees.composites.Selector(
            'Localization Gate', memory=False
        )
        localization.add_children([
            StateBehaviour(
                'Localization Healthy', state_provider, 'localization', success
            ),
            StateBehaviour(
                'Localization Recovery Required',
                state_provider,
                'navigation_active',
                running,
            ),
            StateBehaviour('Navigation Stack Idle', state_provider, 'always', success),
        ])

        mission = py_trees.composites.Selector('Mission Selection', memory=False)
        wall = py_trees.composites.Selector('Wall Following', memory=False)
        wall.detail = 'Choose one mutually exclusive wall-control subtask.'
        wall.setting_keys = ()
        corners = py_trees.composites.Selector('Corner Geometry', memory=False)
        corners.detail = (
            'Classify the wall geometry, commit to one maneuver, then reacquire.'
        )
        corners.setting_keys = ()
        inside = py_trees.composites.Selector(
            'Inside / Concave Corners', memory=False
        )
        inside.detail = (
            'Front and side returns converge toward the robot; turn away from '
            'the obstruction while preserving the right-wall rule.'
        )
        inside.setting_keys = ()
        inside.add_children([
            StateBehaviour(
                '90° Inside Left Corner', state_provider, 'wall_state', running,
                'A right wall continues into a front wall. Slow forward motion '
                'and turn counterclockwise until front clearance returns.',
                ('front_stop_distance', 'emergency_front_distance',
                 'turn_speed', 'wall_timeout'),
                ('inside_corner_left_90',),
            ),
            StateBehaviour(
                '180° Inside Dead-End Turnaround', state_provider,
                'wall_state', running,
                'Both side walls and the front are enclosed. Turn around, then '
                'reserve wall control for the measured exit distance.',
                ('front_stop_distance', 'turn_speed',
                 'dead_end_minimum_depth', 'dead_end_side_max_distance',
                 'dead_end_priority_multiplier'),
                ('inside_dead_end_turn_180',
                 'dead_end_turnaround_priority'),
            ),
            StateBehaviour(
                '270° Inside Pocket Exit', state_provider, 'wall_state', running,
                'A deep concave pocket needs continued left rotation or a '
                'clear backup before the right wall can be reacquired.',
                ('turn_speed', 'front_stop_distance',
                 'emergency_front_distance', 'wall_timeout'),
                ('backup',),
            ),
            StateBehaviour(
                '360° Enclosed-Corner Recovery', state_provider,
                'wall_state', running,
                'No safe forward turn remains. Search all headings, then defer '
                'to backup/global recovery if no qualifying wall is found.',
                ('wall_timeout', 'wall_fit_max_range',
                 'wall_fit_minimum_span', 'progress_timeout'),
                ('search',),
            ),
        ])
        outside = py_trees.composites.Selector(
            'Outside / Convex Corners', memory=False
        )
        outside.detail = (
            'The right wall ends or bends away; commit clockwise and avoid '
            'mistaking the old wall behind the robot for the new face.'
        )
        outside.setting_keys = ()
        outside.add_children([
            StateBehaviour(
                '90° Outside Right Corner', state_provider, 'wall_state', running,
                'Confirm loss of the direct-right wall, then execute a measured '
                'clockwise quarter-turn.',
                ('outside_corner_loss_margin', 'outside_corner_confirmation_time',
                 'outside_corner_turn_angle', 'outside_corner_linear_speed',
                 'outside_corner_turn_speed', 'outside_corner_timeout',
                 'revisited_right_turn_radius',
                 'revisited_right_turn_minimum_age',
                 'revisited_right_turn_clearance',
                 'right_turn_commitment_time', 'emergency_front_distance'),
                ('outside_corner_turn_right_90',),
            ),
            StateBehaviour(
                '180° Outside End-Cap Sweep', state_provider,
                'wall_state', running,
                'Maintain clockwise commitment around a broad convex end until '
                'the former rear face can no longer cause a left-turn reversal.',
                ('right_turn_commitment_time', 'outside_corner_turn_speed',
                 'outside_corner_linear_speed', 'emergency_front_distance'),
                ('right_turn_commitment_guard', 'acquire_right_wall'),
            ),
            StateBehaviour(
                '270° Outside Right Reacquisition', state_provider,
                'wall_state', running,
                'Continue clockwise when a quarter-turn did not expose the wall.',
                ('outside_corner_turn_speed', 'outside_corner_linear_speed',
                 'outside_corner_timeout', 'wall_timeout'),
                ('outside_corner_reacquire_right_270',),
            ),
            StateBehaviour(
                '360° Loop / Previously Traveled Path Checker', state_provider,
                'wall_state', running,
                'A complete perimeter returns onto older travel. The path '
                'overlap checker ends the loop and requests a frontier route.',
                ('retrace_radius', 'retrace_minimum_age',
                 'retrace_minimum_length', 'retrace_overlap_ratio'),
                ('retrace_detected_request_nearest_frontier',),
            ),
        ])
        corners.add_children([inside, outside])
        wall.add_children([
            StateBehaviour(
                'Nav2 Wall Alignment', state_provider, 'wall_state', running,
                'Fit a wall, reach its offset, then drive the straight lead-in.',
                ('target_wall_distance', 'nav2_wall_follow_distance',
                 'minimum_handoff_wall_length', 'handoff_heading_tolerance',
                 'handoff_distance_tolerance', 'alignment_settle_time',
                 'alignment_escape_clearance'),
                ('claiming_nav2_for_wall_alignment',
                 'nav2_align_parallel_to_right_wall',
                 'wall_clearance_escape_before_nav2'),
            ),
            corners,
            StateBehaviour(
                'Trace Straight Right Wall', state_provider, 'wall_state', running,
                'Project the complete asymmetric footprint onto the fitted '
                'wall, then blend heading and nearest-body clearance errors '
                'into smooth forward motion without using the robot center.',
                ('target_wall_distance', 'linear_speed', 'heading_gain',
                 'distance_gain', 'wall_fit_inlier_distance'),
                ('trace_right_wall_clockwise',),
            ),
            StateBehaviour(
                'Narrow Corridor Virtual Wall', state_provider,
                'wall_state', running,
                'Measure paired LiDAR walls ahead. If two adjacent slices are '
                'narrower than the safe corridor width, stop at the entrance '
                'and turn left as though a solid wall spans the opening.',
                ('minimum_corridor_width', 'corridor_lookahead',
                 'narrow_corridor_confirmation_time', 'front_stop_distance',
                 'turn_speed'),
                ('narrow_corridor_virtual_wall',),
            ),
            StateBehaviour(
                'Yield to Global Frontier Planner', state_provider,
                'wall_state', running,
                'Measure map growth during wall following. If the configured '
                'time passes without enough new free area, release velocity '
                'control and request the nearest reachable frontier.',
                ('exploration_evaluation_period', 'minimum_free_area_gain',
                 'global_planning_cooldown', 'loop_return_radius',
                 'loop_minimum_age'),
                ('yield_to_global_exploration',
                 'global_planning_until_long_wall_alignment'),
            ),
            StateBehaviour(
                'Frontier Reached: Re-enable Wall Following', state_provider,
                'wall_state', running,
                'After OpenCV/Nav2 reports that the requested frontier was '
                'reached, require wall fitting and Nav2 alignment again before '
                'direct wall commands can resume.',
                ('minimum_handoff_wall_length', 'alignment_settle_time',
                 'global_planning_cooldown'),
                ('frontier_reached_reenable_wall_alignment',),
            ),
        ])
        mission.add_children([
            StateBehaviour('Return to Origin', state_provider, 'return_home', running),
            StateBehaviour('Dead-End Exit', state_provider, 'dead_end_exit', running),
            wall,
            StateBehaviour(
                'Frontier Navigation', state_provider, 'frontier_navigation', running
            ),
            StateBehaviour('Awaiting Operator', state_provider, 'always', running),
        ])

        root = py_trees.composites.Sequence(
            'Navigation Supervisor',
            memory=False,
            children=[safety, localization, mission],
        )
        self.tree = py_trees_ros.trees.BehaviourTree(root=root)
        self.tree.setup(node=node)

    @property
    def root(self):
        """Return the root behaviour for visualization."""
        return self.tree.root

    def tick(self):
        """Evaluate one observation cycle."""
        self.tree.tick()

    def shutdown(self):
        """Release ROS snapshot services and tree resources."""
        self.tree.shutdown()
