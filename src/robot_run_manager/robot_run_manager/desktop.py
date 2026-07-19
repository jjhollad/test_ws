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
import shlex

from robot_run_manager.main import find_workspace


def wrapper_text(workspace):
    """Return the wrapper which builds and prepares the ROS environment."""
    quoted_workspace = shlex.quote(str(workspace))
    return (
        '#!/usr/bin/env bash\n'
        'set -eo pipefail\n'
        f'cd {quoted_workspace}\n'
        'source /opt/ros/humble/setup.bash\n'
        'colcon build --symlink-install --base-paths src\n'
        f'source {shlex.quote(str(workspace / "install/setup.bash"))}\n'
        'exec ros2 run robot_run_manager run_manager_gui\n'
    )


def launcher_text(workspace, executable=None):
    """Return a desktop entry which opens the environment wrapper."""
    if executable is None:
        executable = Path.home() / '.local/bin/robot-run-manager'
    escaped_executable = str(executable).replace('\\', '\\\\').replace('"', '\\"')
    return (
        '[Desktop Entry]\n'
        'Type=Application\n'
        'Version=1.0\n'
        'Name=Robot Run Manager\n'
        'Comment=Build workspace, then open robot mapping and simulation controls\n'
        f'Exec="{escaped_executable}"\n'
        f'Path={workspace}\n'
        'Icon=applications-engineering\n'
        'Terminal=false\n'
        'Categories=Development;Science;\n'
        'StartupNotify=true\n'
    )


def main():
    workspace = find_workspace()
    executable = Path.home() / '.local/bin/robot-run-manager'
    executable.parent.mkdir(parents=True, exist_ok=True)
    executable.write_text(wrapper_text(workspace), encoding='utf-8')
    executable.chmod(0o755)
    desktop = Path.home() / 'Desktop'
    desktop.mkdir(parents=True, exist_ok=True)
    launcher = desktop / 'Robot Run Manager.desktop'
    launcher.write_text(
        launcher_text(workspace, executable=executable),
        encoding='utf-8',
    )
    launcher.chmod(0o755)
    print(f'Installed desktop launcher: {launcher}')
    return 0
