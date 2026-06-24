# Robot backup

This repository is a backup of all ROS 2 workspaces and maps on this machine.

## Layout

| Path | Source on disk | Notes |
|------|----------------|-------|
| `src/` (repo root) | `~/test_ws` | Primary workspace — tracked directly at repo root |
| `workspaces/create_ws/` | `~/create_ws` | Older Create robot workspace |
| `workspaces/create3_examples_ws/` | `~/create3_examples_ws` | iRobot Create 3 examples |
| `workspaces/sweepbot_ws/` | `~/sweepbot_ws` | Sweepbot project |
| `workspaces/launch_ws/` | `~/launch_ws` | Launch/scripts workspace |
| `workspaces/jackal_navigation/` | `~/jackal_navigation` | Jackal navigation snippets |
| `backup_maps/` | `~/maps`, `~/EERC_SB*`, workspace maps | SLAM / navigation maps |

`build/`, `install/`, `log/`, and nested `.git/` directories are excluded (regenerate with `colcon build`).

## Refresh backup from live workspaces

```bash
./scripts/sync_robot_backup.sh
```

## Restore a workspace elsewhere

```bash
cp -a workspaces/sweepbot_ws ~/sweepbot_ws_restored
cd ~/sweepbot_ws_restored
colcon build
```

## Primary workspace

The repo root **is** `test_ws`. After cloning:

```bash
cd test_ws
colcon build
source install/setup.bash
```
