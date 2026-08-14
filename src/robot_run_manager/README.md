# Robot Run Manager

ROS 2 Humble desktop GUI for operating the real robot, collecting reproducible
runs, mapping, simulating, validating datasets, and moving code or data between
the robot and development computers.

## Safety

The GUI emergency stop publishes zero `/cmd_vel` messages and stops processes
started by this GUI. It cannot electrically isolate motors and does not replace a
physical, hardwired emergency stop. Keep the physical stop within reach whenever
hardware is powered.

Rosbag replay is restricted to **Development / simulation** mode because a bag
may contain recorded velocity commands. Use a separate ROS domain from any real
robot when replaying.

## Features

- role-aware preflight checks;
- guarded robot, mapping, Gazebo, Nav2, SLAM, RViz, and replay processes;
- confirmation-gated **Kill ROS Processes** hard-stop for stale ROS/Gazebo
  stacks, preceded by a zero-velocity burst;
- separately managed Gazebo server/window with an **Open Gazebo** recovery button;
- supervised frontier-based wandering to expand an active SLAM map;
- merged `create2_demo` corridor-museum world, rectangular sweeper, scan
  clearing, wall tracing, and supervised right-wall/frontier autonomous exploration;
- coordinated OpenCV frontier planning and clockwise wall following using the
  tuned 1.05 m offset, 0.28 m/s speed, and `/wall_tracing_active` handoff;
- Nav2-controlled wall acquisition that uses a robust straight-line fit for
  the nearest right-side LiDAR surface, aligns at the configured offset, and
  follows collinear 0.5 m waypoints for 3 m before direct tracing takes over;
- strict wall-control handoff after every global-planning interval: the fitted
  wall must exceed 5 m and the robot must be parallel and at the target offset;
- configurable **Behavior handoff timeout** sets how long the global planner
  retains control before wall acquisition may attempt another takeover;
- dead-end detection measures corridor entry depth and reserves wall-follower
  exit control for `depth × Dead-end exit priority`, suppressing planner/stale
  handoffs until that outbound distance is completed;
- automatic mission completion after sustained frontier exhaustion: wall
  following stops, Nav2 returns to the recorded start pose, and mapping ends;
- novelty-biased global planning that rewards reachable unknown-space gain and
  penalizes candidate route metres overlapping the robot's travel history;
- unavoidable revisit distance is exempted: with only one feasible route its
  revisit penalty is zero, and shared overlap among alternatives is not taxed;
- unavoidable mapped-area transit switches A* to a configurable safe wall
  offset, suspends revisit penalties, and permits a longer 25 m route horizon
  while traveling out to the next frontier;
- persistent **Configuration** tab with described sliders for every planner
  variable used by this application: wall geometry, route construction,
  frontier scoring, corridor inference, novelty, replanning, and recovery;
  values apply to the next behavior start;
- all configuration sliders are explicitly synchronized to the desktop
  settings store when the manager closes and restored on its next launch;
- every configuration section has its own defaults button so one subsystem can
  be reset without disturbing tuning values in the other sections;
- a persistent **Show Gazebo GUI** switch runs the simulation headless when
  unchecked, reducing graphics load while keeping Gazebo physics active;
- a persistent **Show RViz GUI** switch starts or closes RViz independently
  while leaving Gazebo, Nav2, and the rest of the simulation running;
- a persistent Gazebo GUI FPS selector limits client rendering to
  10/15/20/30/60 FPS without changing simulation or sensor timing;
- **Rebuild & Restart Manager** safely saves settings, rebuilds the complete
  workspace in a terminal, and relaunches the GUI with the updated code;
- a live, editable **Behavior Tree** tab uses `py_trees_ros` to show safety,
  localization, mission selection, wall tracing, corner geometry, recovery,
  and planner handoffs; selecting a leaf opens its persistent tuning controls;
- guarded rolling Nav2 route extension, adjustable with **Path extension
  interval**, adds farther frontier poses near the route tail without rapid
  mid-route preemption;
- configurable **Segment completion timeout** controls how long Nav2 may take
  to advance each 0.25 m progress segment before stalled-route recovery;
- configurable **Map discovery timeout** cancels a physically moving but
  unproductive route when it fails to add 1.0 m² of free map in time;
- **SLAM mapping** sliders tune map refresh/resolution, scan travel and heading
  thresholds, scan-buffer size/range, local link matching, and loop search;
- **Flag & Open Logs** writes a timestamped identifier to
  `log/user_log_flags.csv`, mirrors it into an active run's events, and opens
  the flag index plus recent log files in a terminal reader;
- one-click wall-mapping bringup starts the corridor world, RViz, SLAM, Nav2,
  chassis-aware LiDAR filtering, the global planner, and wall following;
- timestamped rosbag runs with JSON metadata and CSV event markers;
- map saving with safe file names;
- run summaries, stable 80/10/10 splits, and SHA-256 checksums;
- resumable SSH/`rsync` run upload and download;
- guarded Git pull and utility-scoped commit/push;
- live process output and ROS log-folder access.

## Build and run

```bash
cd ~/test_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_run_manager
source install/setup.bash
ros2 run robot_run_manager run_manager_gui
```

Install a clickable desktop icon:

```bash
ros2 run robot_run_manager install_desktop_launcher
```

Then double-click **Robot Run Manager** on the desktop. On Ubuntu, the first
launch may require right-clicking the icon and selecting **Allow Launching**.
The same installer is available inside the GUI as **Install Desktop Launcher**.
Each desktop launch builds the active `src` tree with
`colcon build --symlink-install --base-paths src`, sources ROS 2 and
`install/setup.bash`, then opens the GUI. Limiting discovery to `src` avoids
duplicate packages stored under nested transfer workspaces.

Set `ROBOT_WORKSPACE` before launching if the clone is not at `~/test_ws`:

```bash
export ROBOT_WORKSPACE=/absolute/path/to/test_ws
```

Repeat the clone/build commands on the other ROS 2 Humble machine. Select **Real
robot** on the robot computer and **Development / simulation** on the workstation.

## Run contents

```text
runs/YYYY-MM-DD-HHMMSS/
├── bag/
├── calibration/           # snapshot of navigation/robot configuration
├── checksums.sha256       # created by validation
├── events.csv
├── metadata.json
└── summary.json           # created by validation
```

The requested recording topics are `/cmd_vel`, `/odom`, `/scan`, `/tf`,
`/tf_static`, `/joint_states`, and `/relay_status`. Add camera or other sensor
topics to `RECORD_TOPICS` in `robot_run_manager/main.py` when those sensors are
installed.

Validation assigns each complete run to `train`, `validation`, or `test` using a
stable hash of its run ID. Entire runs remain together; frames from one run are
never split across datasets.

## Machine-to-machine transfer

Install `rsync` and configure SSH keys in both directions first. The GUI uses
non-interactive SSH and will not accept a password prompt:

```bash
ssh-copy-id user@other-computer
ssh user@other-computer true
```

Enter `user@other-computer`, its runs directory, and the direction in the GUI.
Transfers use checksums, resume partial files, and never use `--delete`.

## Git controls

Run datasets are excluded by `.gitignore`; GitHub carries code, not rosbags.

- **Download Code** runs `git pull --ff-only` only when the working tree is clean.
- **Upload Code** stages only `.gitignore` and `src/robot_run_manager`, requires a
  commit message and confirmation, commits, then pushes `HEAD` to `origin`.
- Any unrelated staged file blocks GUI upload.

Existing unrelated working-tree changes are intentionally not staged or altered.
