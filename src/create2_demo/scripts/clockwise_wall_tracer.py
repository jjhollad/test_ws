#!/usr/bin/env python3
"""Continuous clockwise (right-hand) lidar wall tracing behavior."""

import math

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ClockwiseWallTracer(Node):
    def __init__(self):
        super().__init__("clockwise_wall_tracer")
        defaults = {
            "target_wall_distance": 0.80,
            "front_stop_distance": 1.25,
            "linear_speed": 0.22,
            "turn_speed": 0.45,
            "heading_gain": 1.4,
            "distance_gain": 0.9,
            "wall_timeout": 2.0,
            # Preserve the later recovery policy: backup is only allowed after
            # remaining inside a 2 m odometry radius for two minutes.
            "progress_timeout": 120.0,
            "significant_progress": 2.0,
            "backup_distance": 2.14,
            "backup_speed": 0.15,
            "startup_delay": 30.0,
            "exploration_evaluation_period": 25.0,
            "minimum_free_area_gain": 1.5,
            "loop_return_radius": 1.0,
            "loop_minimum_age": 30.0,
            "global_planning_cooldown": 45.0,
            "trace_path_spacing": 0.10,
            "trace_path_maximum_poses": 20000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._scan = None
        self._scan_start_time = None
        self._pose = None
        self._free_area = 0.0
        self._trace_start_time = None
        self._trace_start_free_area = 0.0
        self._cooldown_until = None
        self._history = []
        self._last_history_time = None
        self._progress_pose = None
        self._progress_time = self.get_clock().now()
        self._last_wall_time = None
        self._state = "search"
        self._backup_start = None
        self._trace_path = Path()
        self._trace_path.header.frame_id = "odom"
        self._cmd = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._active = self.create_publisher(Bool, "/wall_tracing_active", 10)
        self._trace_path_pub = self.create_publisher(
            Path, "/robot_global_trace", 10
        )
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, 10)
        self.create_timer(0.10, self._control)

    def _scan_callback(self, message):
        self._scan = message
        if self._scan_start_time is None:
            self._scan_start_time = self.get_clock().now()

    def _odom_callback(self, message):
        self._pose = (
            message.pose.pose.position.x, message.pose.pose.position.y
        )
        append = not self._trace_path.poses
        if not append:
            previous = self._trace_path.poses[-1].pose.position
            append = math.hypot(
                self._pose[0] - previous.x, self._pose[1] - previous.y
            ) >= float(self.get_parameter("trace_path_spacing").value)
        if append:
            pose = PoseStamped()
            pose.header = message.header
            pose.header.frame_id = "odom"
            pose.pose = message.pose.pose
            self._trace_path.poses.append(pose)
            maximum = int(
                self.get_parameter("trace_path_maximum_poses").value
            )
            if len(self._trace_path.poses) > maximum:
                self._trace_path.poses = self._trace_path.poses[-maximum:]
            self._trace_path.header.stamp = message.header.stamp
            self._trace_path_pub.publish(self._trace_path)

    def _map_callback(self, message):
        values = np.asarray(message.data, dtype=np.int16)
        free = np.count_nonzero((values >= 0) & (values <= 20))
        self._free_area = free * message.info.resolution ** 2

    def _yield_to_global_planner(self, reason, now):
        cooldown = float(self.get_parameter("global_planning_cooldown").value)
        self._cooldown_until = now + Duration(seconds=cooldown)
        self._trace_start_time = None
        self._history.clear()
        self._set_state("yield_to_global_exploration")
        self.get_logger().info(
            f"Yielding clockwise tracing for {cooldown:.0f} s: {reason}."
        )

    def _exploration_is_stale(self, now):
        if self._pose is None:
            return False
        if self._last_history_time is None or (
            now - self._last_history_time
        ).nanoseconds >= 1e9:
            minimum_age = float(self.get_parameter("loop_minimum_age").value)
            radius = float(self.get_parameter("loop_return_radius").value)
            for stamp, x, y in self._history:
                age = (now - stamp).nanoseconds / 1e9
                if age >= minimum_age and math.hypot(
                    self._pose[0] - x, self._pose[1] - y
                ) <= radius:
                    return "returned to an already traced location"
            self._history.append((now, self._pose[0], self._pose[1]))
            self._history = self._history[-300:]
            self._last_history_time = now
        if self._trace_start_time is None:
            self._trace_start_time = now
            self._trace_start_free_area = self._free_area
            return False
        evaluation = float(
            self.get_parameter("exploration_evaluation_period").value
        )
        if (now - self._trace_start_time).nanoseconds / 1e9 < evaluation:
            return False
        gain = self._free_area - self._trace_start_free_area
        minimum_gain = float(self.get_parameter("minimum_free_area_gain").value)
        if gain < minimum_gain:
            return f"only {gain:.1f} m^2 of new free map was added"
        self._trace_start_time = now
        self._trace_start_free_area = self._free_area
        return False

    def _set_state(self, state):
        if state != self._state:
            self._state = state
            self.get_logger().info(f"Clockwise tracing behavior: {state}.")

    def _right_wall(self, ranges, angles):
        valid = np.isfinite(ranges)
        x, y = ranges * np.cos(angles), ranges * np.sin(angles)
        selected = valid & (angles < math.radians(-35)) & (
            angles > math.radians(-125)
        ) & (ranges < 5.0)
        points = np.column_stack((x[selected], y[selected]))
        if len(points) < 8:
            return None
        vx, vy, px, py = cv2.fitLine(
            points.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01
        ).reshape(-1)
        yaw = math.atan2(float(vy), float(vx))
        if math.cos(yaw) < 0.0:
            yaw = math.atan2(-float(vy), -float(vx))
        distance = abs(float(vx) * float(py) - float(vy) * float(px))
        return distance, yaw

    def _update_progress(self):
        if self._pose is None:
            return False
        now = self.get_clock().now()
        if self._progress_pose is None:
            self._progress_pose = self._pose
            self._progress_time = now
            return False
        progress = math.hypot(
            self._pose[0] - self._progress_pose[0],
            self._pose[1] - self._progress_pose[1],
        )
        if progress >= float(self.get_parameter("significant_progress").value):
            self._progress_pose = self._pose
            self._progress_time = now
            return False
        return ((now - self._progress_time).nanoseconds / 1e9
                >= float(self.get_parameter("progress_timeout").value))

    def _control(self):
        if self._scan is None:
            return
        now = self.get_clock().now()
        startup_elapsed = (
            (now - self._scan_start_time).nanoseconds / 1e9
            if self._scan_start_time is not None else 0.0
        )
        if startup_elapsed < float(self.get_parameter("startup_delay").value):
            self._active.publish(Bool(data=False))
            self._progress_pose = self._pose
            self._progress_time = now
            return
        if self._cooldown_until is not None and now < self._cooldown_until:
            self._active.publish(Bool(data=False))
            self._cmd.publish(Twist())
            return
        if self._cooldown_until is not None:
            self._cooldown_until = None
            self._progress_pose, self._progress_time = self._pose, now
        scan = self._scan
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        front = ranges[np.abs(angles) < math.radians(22.5)]
        front = front[np.isfinite(front)]
        front_distance = float(np.min(front)) if len(front) else scan.range_max
        wall = self._right_wall(ranges, angles)
        if wall is not None:
            self._last_wall_time = now
        recently_seen = (
            self._last_wall_time is not None
            and (now - self._last_wall_time).nanoseconds / 1e9
            <= float(self.get_parameter("wall_timeout").value)
        )
        stale_reason = self._exploration_is_stale(now) if recently_seen else False
        if stale_reason:
            self._yield_to_global_planner(stale_reason, now)
            self._active.publish(Bool(data=False))
            self._cmd.publish(Twist())
            return
        self._active.publish(Bool(data=recently_seen or wall is not None))
        command = Twist()
        if not recently_seen and wall is None:
            self._set_state("search")
            self._cmd.publish(command)
            return

        if self._state == "backup":
            rear = ranges[np.abs(np.abs(angles) - math.pi) < math.radians(25)]
            rear = rear[np.isfinite(rear)]
            travelled = (math.hypot(
                self._pose[0] - self._backup_start[0],
                self._pose[1] - self._backup_start[1],
            ) if self._pose is not None and self._backup_start is not None else 0.0)
            if travelled >= float(self.get_parameter("backup_distance").value):
                self._progress_pose, self._progress_time = self._pose, now
                self._set_state("acquire_right_wall")
            elif len(rear) and np.min(rear) < 0.65:
                self._set_state("inside_corner_left")
            else:
                command.linear.x = -float(
                    self.get_parameter("backup_speed").value
                )
                self._cmd.publish(command)
                return

        if self._update_progress():
            self._backup_start = self._pose
            self._set_state("backup")
            command.linear.x = -float(self.get_parameter("backup_speed").value)
        elif front_distance < float(
            self.get_parameter("front_stop_distance").value
        ):
            self._set_state("inside_corner_left")
            command.linear.x = 0.04
            command.angular.z = float(self.get_parameter("turn_speed").value)
        elif wall is None:
            self._set_state("outside_corner_reacquire_right")
            command.linear.x = 0.08
            command.angular.z = -float(self.get_parameter("turn_speed").value)
        else:
            distance, wall_yaw = wall
            target = float(self.get_parameter("target_wall_distance").value)
            angular = (
                float(self.get_parameter("heading_gain").value) * wall_yaw
                - float(self.get_parameter("distance_gain").value)
                * (distance - target)
            )
            self._set_state("trace_right_wall_clockwise")
            command.linear.x = float(self.get_parameter("linear_speed").value)
            command.angular.z = float(np.clip(angular, -0.55, 0.55))
        self._cmd.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = ClockwiseWallTracer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
