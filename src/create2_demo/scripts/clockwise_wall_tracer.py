#!/usr/bin/env python3
"""Continuous clockwise (right-hand) lidar wall tracing behavior."""

import math

import cv2
import numpy as np
import rclpy
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


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
            "align_with_nav2": True,
            "alignment_settle_time": 0.75,
            "alignment_lead_in": 0.60,
            "nav2_wall_follow_distance": 3.0,
            "wall_fit_max_range": 8.0,
            "wall_fit_inlier_distance": 0.10,
            "wall_fit_minimum_span": 0.80,
            "minimum_handoff_wall_length": 5.0,
            "handoff_heading_tolerance": 0.17,
            "handoff_distance_tolerance": 0.20,
            "alignment_waypoint_spacing": 0.50,
            "dead_end_priority_multiplier": 1.20,
            "dead_end_minimum_depth": 1.0,
            "dead_end_side_max_distance": 4.0,
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
        self._returning_home = False
        self._alignment_complete = False
        self._alignment_pending = False
        self._alignment_claimed_at = None
        self._alignment_goal_handle = None
        self._odometry_distance = 0.0
        self._corridor_entry_distance = 0.0
        self._dead_end_escape_remaining = 0.0
        self._dead_end_escape_active = False
        self._backup_start = None
        self._trace_path = Path()
        self._trace_path.header.frame_id = "odom"
        self._cmd = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._active = self.create_publisher(Bool, "/wall_tracing_active", 10)
        self._navigation = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses"
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._trace_path_pub = self.create_publisher(
            Path, "/robot_global_trace", 10
        )
        self._alignment_path_pub = self.create_publisher(
            Path, "/wall_alignment_path", 10
        )
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, 10)
        self.create_subscription(
            Bool, "/mapping_return_home", self._return_home_callback, 10
        )
        self.create_timer(0.10, self._control)

    def _return_home_callback(self, message):
        self._returning_home = message.data
        if self._returning_home:
            if self._alignment_goal_handle is not None:
                self._alignment_goal_handle.cancel_goal_async()
            self._set_state("yield_for_return_home")
            self._cmd.publish(Twist())
            self._active.publish(Bool(data=False))

    def _scan_callback(self, message):
        self._scan = message
        if self._scan_start_time is None:
            self._scan_start_time = self.get_clock().now()

    def _odom_callback(self, message):
        previous_pose = self._pose
        self._pose = (
            message.pose.pose.position.x, message.pose.pose.position.y
        )
        if previous_pose is not None:
            travelled = math.hypot(
                self._pose[0] - previous_pose[0],
                self._pose[1] - previous_pose[1],
            )
            self._odometry_distance += travelled
            if self._dead_end_escape_active:
                self._dead_end_escape_remaining = max(
                    0.0, self._dead_end_escape_remaining - travelled
                )
                if self._dead_end_escape_remaining == 0.0:
                    self._dead_end_escape_active = False
                    self._history.clear()
                    self._trace_start_time = self.get_clock().now()
                    self._trace_start_free_area = self._free_area
                    self.get_logger().info(
                        "Dead-end escape priority distance completed; normal "
                        "behavior handoffs restored."
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
        self._alignment_complete = False
        self._alignment_pending = False
        self._alignment_claimed_at = None
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
        x = np.full_like(ranges, np.nan)
        y = np.full_like(ranges, np.nan)
        x[valid] = ranges[valid] * np.cos(angles[valid])
        y[valid] = ranges[valid] * np.sin(angles[valid])
        selected = (
            valid
            & (angles < math.radians(-10))
            & (angles > math.radians(-170))
            & (ranges < float(self.get_parameter("wall_fit_max_range").value))
        )
        side_ranges = ranges[selected]
        if len(side_ranges) < 8:
            return None
        points = np.column_stack((x[selected], y[selected]))
        # RANSAC keeps the longest strongly supported straight surface and
        # rejects doorway edges, corners, furniture, and farther walls.
        threshold = float(
            self.get_parameter("wall_fit_inlier_distance").value
        )
        generator = np.random.default_rng(7)
        best_inliers = None
        best_score = -1.0
        best_span = 0.0
        trials = min(160, max(40, len(points) * 2))
        for _ in range(trials):
            first, second = generator.choice(len(points), 2, replace=False)
            direction = points[second] - points[first]
            length = float(np.linalg.norm(direction))
            if length < 0.30:
                continue
            direction /= length
            normal = np.array([-direction[1], direction[0]])
            errors = np.abs((points - points[first]) @ normal)
            inliers = errors <= threshold
            if np.count_nonzero(inliers) < 8:
                continue
            projections = points[inliers] @ direction
            span = float(np.ptp(projections))
            support = int(np.count_nonzero(inliers))
            proximity = max(0.20, float(np.median(np.linalg.norm(
                points[inliers], axis=1
            ))))
            score = span * math.sqrt(support) / proximity
            if score > best_score:
                best_score = score
                best_span = span
                best_inliers = inliers
        if best_inliers is None or best_span < float(
            self.get_parameter("wall_fit_minimum_span").value
        ):
            return None
        vx, vy, px, py = cv2.fitLine(
            points[best_inliers].astype(np.float32),
            cv2.DIST_HUBER,
            0,
            0.01,
            0.01,
        ).reshape(-1)
        yaw = math.atan2(float(vy), float(vx))
        # Choose the direction for which this surface lies on the robot's
        # right. The left-normal signed distance must therefore be negative.
        signed_distance = -float(vy) * float(px) + float(vx) * float(py)
        if signed_distance > 0.0:
            vx, vy = -vx, -vy
            yaw = math.atan2(float(vy), float(vx))
            signed_distance = -signed_distance
        distance = -signed_distance
        return distance, yaw, best_span

    def _wall_is_long_enough(self, wall):
        return wall is not None and wall[2] > float(
            self.get_parameter("minimum_handoff_wall_length").value
        )

    def _wall_is_handoff_eligible(self, wall):
        if not self._wall_is_long_enough(wall):
            return False
        distance, relative_yaw, _ = wall
        target = float(self.get_parameter("target_wall_distance").value)
        return (
            abs(distance - target) <= float(
                self.get_parameter("handoff_distance_tolerance").value
            )
            and abs(relative_yaw) <= float(
                self.get_parameter("handoff_heading_tolerance").value
            )
        )

    def _current_right_wall(self):
        if self._scan is None:
            return None
        ranges = np.asarray(self._scan.ranges, dtype=np.float64)
        angles = (
            self._scan.angle_min
            + np.arange(len(ranges)) * self._scan.angle_increment
        )
        return self._right_wall(ranges, angles)

    def _side_wall_present(self, ranges, angles, left=False):
        lower, upper = ((35, 125) if left else (-125, -35))
        selected = (
            np.isfinite(ranges)
            & (angles > math.radians(lower))
            & (angles < math.radians(upper))
        )
        side = ranges[selected]
        return len(side) >= 8 and float(np.percentile(side, 25.0)) <= float(
            self.get_parameter("dead_end_side_max_distance").value
        )

    def _start_dead_end_escape(self):
        depth = max(
            0.0, self._odometry_distance - self._corridor_entry_distance
        )
        minimum = float(self.get_parameter("dead_end_minimum_depth").value)
        multiplier = float(
            self.get_parameter("dead_end_priority_multiplier").value
        )
        if depth < minimum or multiplier <= 0.0:
            return False
        self._dead_end_escape_remaining = depth * multiplier
        self._dead_end_escape_active = True
        self._set_state("dead_end_turnaround_priority")
        self.get_logger().warn(
            f"Dead end detected {depth:.1f} m into corridor; reserving wall "
            f"control for {self._dead_end_escape_remaining:.1f} m of exit travel."
        )
        return True

    @staticmethod
    def _yaw_from_quaternion(rotation):
        return math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
        )

    def _start_wall_alignment(self, wall, now):
        """Ask Nav2 to establish the wall distance and parallel heading."""
        if not self._navigation.server_is_ready():
            self.get_logger().info(
                "Waiting for Nav2 before wall alignment.",
                throttle_duration_sec=2.0,
            )
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", "base_footprint", Time()
            )
        except TransformException as error:
            self.get_logger().info(
                f"Waiting for map-to-robot transform: {error}",
                throttle_duration_sec=2.0,
            )
            return
        distance, relative_wall_yaw, wall_span = wall
        robot_yaw = self._yaw_from_quaternion(transform.transform.rotation)
        # The fitted wall direction points forward. Its left normal points
        # from a right-side wall toward the robot. Move along that normal until
        # the robot center is exactly target_wall_distance from the wall.
        normal_x = -math.sin(relative_wall_yaw)
        normal_y = math.cos(relative_wall_yaw)
        correction = float(
            self.get_parameter("target_wall_distance").value
        ) - distance
        local_x = normal_x * correction
        local_y = normal_y * correction
        goal_yaw = robot_yaw + relative_wall_yaw
        lead_in = float(self.get_parameter("alignment_lead_in").value)
        straight_distance = float(
            self.get_parameter("nav2_wall_follow_distance").value
        )

        def wall_pose(forward_distance):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now.to_msg()
            route_x = local_x + math.cos(relative_wall_yaw) * forward_distance
            route_y = local_y + math.sin(relative_wall_yaw) * forward_distance
            pose.pose.position.x = (
                transform.transform.translation.x
                + math.cos(robot_yaw) * route_x
                - math.sin(robot_yaw) * route_y
            )
            pose.pose.position.y = (
                transform.transform.translation.y
                + math.sin(robot_yaw) * route_x
                + math.cos(robot_yaw) * route_y
            )
            pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
            pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
            return pose

        spacing = max(
            0.20,
            float(self.get_parameter("alignment_waypoint_spacing").value),
        )
        straight_steps = max(1, int(math.ceil(straight_distance / spacing)))
        route_distances = [lead_in]
        route_distances.extend(
            lead_in + straight_distance * step / straight_steps
            for step in range(1, straight_steps + 1)
        )
        request = NavigateThroughPoses.Goal()
        request.poses = [wall_pose(distance) for distance in route_distances]
        display_path = Path()
        display_path.header.frame_id = "map"
        display_path.header.stamp = now.to_msg()
        display_path.poses = request.poses
        self._alignment_path_pub.publish(display_path)
        self._alignment_pending = True
        self._set_state("nav2_align_parallel_to_right_wall")
        self.get_logger().info(
            f"Nav2 aligning to the nearby right wall at {distance:.2f} m, then "
            f"driving {straight_distance:.1f} m parallel at "
            f"{float(self.get_parameter('target_wall_distance').value):.2f} m "
            f"along a {wall_span:.1f} m LiDAR wall fit."
        )
        future = self._navigation.send_goal_async(request)
        future.add_done_callback(self._alignment_goal_response)

    def _alignment_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("Nav2 rejected wall alignment; retrying.")
            self._alignment_pending = False
            self._alignment_claimed_at = None
            return
        self._alignment_goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(self._alignment_goal_result)

    def _alignment_goal_result(self, future):
        result = future.result()
        self._alignment_goal_handle = None
        self._alignment_pending = False
        if result is not None and result.status == 4 and not self._returning_home:
            wall = self._current_right_wall()
            if self._wall_is_handoff_eligible(wall):
                self._alignment_complete = True
                self._progress_pose = self._pose
                self._progress_time = self.get_clock().now()
                self._set_state("wall_alignment_complete")
                self.get_logger().info(
                    f"Nav2 alignment verified on a {wall[2]:.1f} m wall; "
                    "handing off to wall tracing."
                )
            else:
                self._alignment_claimed_at = None
                self._active.publish(Bool(data=False))
                self._set_state("global_planning_until_long_wall_alignment")
                self.get_logger().info(
                    "Wall handoff withheld: requires >5 m wall, parallel "
                    "heading, and target right-side distance."
                )
        elif not self._returning_home:
            self._alignment_claimed_at = None
            self.get_logger().warn("Nav2 wall alignment failed; retrying.")

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
        if self._returning_home:
            self._cmd.publish(Twist())
            self._active.publish(Bool(data=False))
            return
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
        if (bool(self.get_parameter("align_with_nav2").value)
                and not self._alignment_complete):
            if wall is None:
                self._active.publish(Bool(data=False))
                self._set_state("waiting_for_right_wall_to_align")
                return
            if not self._wall_is_long_enough(wall):
                self._active.publish(Bool(data=False))
                self._set_state("global_planning_until_wall_exceeds_5m")
                return
            self._active.publish(Bool(data=True))
            self._cmd.publish(Twist())
            if self._alignment_claimed_at is None:
                self._alignment_claimed_at = now
                self._corridor_entry_distance = self._odometry_distance
                self._set_state("claiming_nav2_for_wall_alignment")
                return
            settle = float(self.get_parameter("alignment_settle_time").value)
            if (not self._alignment_pending
                    and (now - self._alignment_claimed_at).nanoseconds
                    >= settle * 1e9):
                self._start_wall_alignment(wall, now)
            return
        if wall is not None:
            self._last_wall_time = now
        recently_seen = (
            self._last_wall_time is not None
            and (now - self._last_wall_time).nanoseconds / 1e9
            <= float(self.get_parameter("wall_timeout").value)
        )
        if self._dead_end_escape_active:
            recently_seen = True
        stale_reason = (
            self._exploration_is_stale(now)
            if recently_seen and not self._dead_end_escape_active
            else False
        )
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

        dead_end = (
            not self._dead_end_escape_active
            and front_distance < float(
                self.get_parameter("front_stop_distance").value
            )
            and wall is not None
            and self._side_wall_present(ranges, angles, left=True)
        )
        if dead_end:
            self._start_dead_end_escape()

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
            distance, wall_yaw, _ = wall
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
