#!/usr/bin/env bash
# Hard-stop ROS 2 and common robot nodes (SIGKILL). Use when a normal Ctrl+C / ros2 lifecycle stop is not enough.
#
# Does NOT match bare "python3" (too broad on a dev machine). Add manually if you need it.
#
# Usage:
#   ./kill_robot_stack.sh
#   KILL_PYTHON=1 ./kill_robot_stack.sh   # also SIGKILL processes whose cmdline contains "python3" (dangerous)

set -euo pipefail

PATTERN='ros2|ros |gazebo|gz |nav2|amcl|bt_navigator|controller_server|velocity_smoother|planner_server|behavior_server|waypoint_follower|smoother_server|rviz2|teleop_twist_joy|teleop_node|joy_node|generic_motor_driver|relay_controller|robot_state_publisher|joint_state_publisher|slam_toolbox|async_slam|rplidar|assisted_teleop|cmd_vel_relay|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator|create_driver|generic_motor'

if [[ "${KILL_PYTHON:-0}" == "1" ]]; then
  PATTERN="${PATTERN}|python3"
fi

pkill -9 -f "${PATTERN}" 2>/dev/null || true
echo "kill_robot_stack: SIGKILL sent to processes matching the stack pattern (if any were running)."
