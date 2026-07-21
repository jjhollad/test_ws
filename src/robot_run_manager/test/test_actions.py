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

from pathlib import Path

from robot_run_manager.main import (
    ACTION_GROUPS,
    ACTION_TOOLTIPS,
    IMPLEMENTED_ACTIONS,
    TUNING_VARIABLES,
    find_autonomy_workspace,
)


def test_every_implemented_action_has_a_button():
    buttons = {
        action
        for actions in ACTION_GROUPS.values()
        for action in actions
    }
    assert IMPLEMENTED_ACTIONS <= buttons
    assert buttons == set(ACTION_TOOLTIPS)
    assert all(ACTION_TOOLTIPS.values())
    assert 'Start Wandering Mapper' not in buttons
    assert 'Stop Wandering Mapper' not in buttons
    assert {'Start Xbox Teleop', 'Stop Xbox Teleop'} <= buttons
    assert {
        'Start Autonomous Exploration',
        'Pause Exploration',
        'Resume Exploration',
        'Stop Autonomous Exploration',
        'Exploration E-Stop',
        'Clear Exploration E-Stop',
    } <= buttons


def test_autonomy_workspace_discovery(tmp_path, monkeypatch):
    launch = (
        tmp_path / 'src/create_robot/create_driver/launch'
        / 'autonomous_exploration.launch.py'
    )
    launch.parent.mkdir(parents=True)
    launch.write_text('')
    monkeypatch.setenv('BIGSWEEP_WORKSPACE', str(tmp_path))
    assert find_autonomy_workspace(Path('/missing')) == tmp_path


def test_tuning_variables_have_safe_ranges_and_descriptions():
    assert TUNING_VARIABLES
    for specification in TUNING_VARIABLES.values():
        name, group, minimum, maximum, default, decimals, description = specification
        assert name and group and description
        assert minimum <= default <= maximum
        assert decimals in (0, 1, 2)
