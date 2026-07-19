#!/usr/bin/env bash

set -o pipefail

workspace="$1"
manager_pid="$2"

while kill -0 "$manager_pid" 2>/dev/null; do
  sleep 0.2
done

cd "$workspace" || exit 1
source /opt/ros/humble/setup.bash

echo "Building ROS workspace: $workspace"
echo "Package source root: $workspace/src"
if ! colcon build --base-paths "$workspace/src"; then
  echo "BUILD FAILED. Restarting the previously installed Manager."
  source "$workspace/install/setup.bash"
  nohup "$workspace/install/robot_run_manager/lib/robot_run_manager/run_manager_gui" \
    >>"$workspace/log/manager_restart.log" 2>&1 </dev/null &
  exit 1
fi

mkdir -p "$workspace/log"
restart_log="$workspace/log/manager_restart.log"
echo "Sourcing the rebuilt workspace." >"$restart_log"
source "$workspace/install/setup.bash" 2>>"$restart_log"
echo "Build complete. Restarting Robot Run Manager."
nohup "$workspace/install/robot_run_manager/lib/robot_run_manager/run_manager_gui" \
  >>"$restart_log" 2>&1 </dev/null &
new_manager_pid=$!
sleep 1
if ! kill -0 "$new_manager_pid" 2>/dev/null; then
  echo "RESTART FAILED. Details from $restart_log:"
  tail -40 "$restart_log"
  exit 1
fi
echo "Manager restarted successfully with PID $new_manager_pid."
