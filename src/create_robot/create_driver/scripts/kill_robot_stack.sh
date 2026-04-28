#!/usr/bin/env bash
# Hard-stop ROS 2 and common robot nodes (SIGKILL). Use when a normal Ctrl+C / ros2 lifecycle stop is not enough.
#
# Does NOT match bare "python3" (too broad on a dev machine). Add manually if you need it.
#
# Usage:
#   ./kill_robot_stack.sh
#   KILL_PYTHON=1 ./kill_robot_stack.sh   # also SIGKILL processes whose cmdline contains "python3" (dangerous)

set -euo pipefail

PATTERN='ros2|gazebo|gzserver|gzclient|ign|nav2|amcl|bt_navigator|controller_server|velocity_smoother|planner_server|behavior_server|waypoint_follower|smoother_server|rviz2|teleop_twist_joy|teleop_node|joy_node|generic_motor_driver|relay_controller|robot_state_publisher|joint_state_publisher|slam_toolbox|async_slam|rplidar|assisted_teleop|cmd_vel_relay|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator|create_driver|generic_motor'

if [[ "${KILL_PYTHON:-0}" == "1" ]]; then
  PATTERN="${PATTERN}|python3"
fi

declare -A EXCLUDE_PIDS=()
EXCLUDE_PIDS["$$"]=1
EXCLUDE_PIDS["${BASHPID:-$$}"]=1

# Exclude this shell's parent chain (terminal + wrappers) so the terminal stays open.
parent="${PPID:-0}"
while [[ "${parent}" =~ ^[0-9]+$ ]] && [[ "${parent}" -gt 1 ]]; do
  EXCLUDE_PIDS["${parent}"]=1
  next_parent="$(ps -o ppid= -p "${parent}" 2>/dev/null | tr -d '[:space:]')"
  [[ -z "${next_parent}" ]] && break
  parent="${next_parent}"
done

echo "kill_robot_stack: matching processes (excluding current terminal chain):"
while IFS= read -r pid; do
  [[ -z "${pid}" ]] && continue
  if [[ -n "${EXCLUDE_PIDS[${pid}]:-}" ]]; then
    continue
  fi
  cmd="$(ps -o cmd= -p "${pid}" 2>/dev/null || true)"
  [[ -z "${cmd}" ]] && continue
  echo "killing ${pid}: ${cmd}"
  kill -9 "${pid}" 2>/dev/null || true
done < <(pgrep -f "${PATTERN}" 2>/dev/null || true)

echo "kill_robot_stack: SIGKILL sent to matching stack processes (if any were running)."
