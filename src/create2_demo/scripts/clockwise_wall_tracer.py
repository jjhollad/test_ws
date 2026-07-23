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
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class ClockwiseWallTracer(Node):
    # Nav2 footprint expressed in laser_frame.  The LiDAR is 0.24 m forward
    # of base_footprint, so the measured body envelope is x=[-0.26, 0.81],
    # y=[-0.45, 0.35].  Keeping the complete polygon matters while the robot
    # is angled: a front or rear corner can be closer than the right edge.
    _FOOTPRINT_FROM_LASER = (
        (0.81, 0.35),
        (0.81, -0.45),
        (-0.26, -0.45),
        (-0.26, 0.35),
    )

    def __init__(self):
        super().__init__("clockwise_wall_tracer")
        defaults = {
            "target_wall_distance": 1.00,
            "inside_corner_front_clearance": 1.00,
            "front_stop_distance": 0.50,
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
            "alignment_escape_clearance": 0.20,
            "alignment_navigation_timeout": 30.0,
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
            # Outside (convex) right corners need a latched maneuver.  The
            # global line fit may still see the wall behind the robot and
            # otherwise keep commanding straight travel past the corner.
            "outside_corner_loss_margin": 0.65,
            "outside_corner_confirmation_time": 0.35,
            "outside_corner_turn_angle": 1.57,
            "outside_corner_linear_speed": 0.07,
            "outside_corner_turn_speed": 0.38,
            "outside_corner_timeout": 8.0,
            "revisited_right_turn_radius": 1.25,
            "revisited_right_turn_minimum_age": 20.0,
            "revisited_right_turn_clearance": 0.50,
            "right_turn_commitment_time": 2.0,
            "emergency_front_distance": 0.15,
            "minimum_corridor_width": 1.20,
            "corridor_lookahead": 6.0,
            "narrow_corridor_confirmation_time": 0.30,
            "retrace_radius": 0.60,
            "retrace_minimum_age": 20.0,
            "retrace_minimum_length": 2.0,
            "retrace_overlap_ratio": 0.70,
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
        self._alignment_started_at = None
        self._alignment_safety_abort = False
        self._odometry_distance = 0.0
        self._corridor_entry_distance = 0.0
        self._dead_end_escape_remaining = 0.0
        self._dead_end_escape_active = False
        self._backup_start = None
        self._yaw = None
        self._right_side_seen = False
        self._right_side_lost_since = None
        self._outside_corner_start_yaw = None
        self._outside_corner_started_at = None
        self._right_turn_priority_until = None
        self._narrow_corridor_since = None
        self._narrow_corridor_width = None
        self._frontier_handoff_active = False
        self._travel_history = []
        self._last_retrace_check = None
        self._trace_path = Path()
        self._trace_path.header.frame_id = "odom"
        # The coordinator is the only /cmd_vel owner. Wall control has a
        # private input channel and can no longer race Nav2 commands.
        self._cmd = self.create_publisher(Twist, "/cmd_vel_wall", 10)
        self._active = self.create_publisher(Bool, "/wall_tracing_active", 10)
        self._state_pub = self.create_publisher(String, "/wall_behavior_state", 10)
        self._frontier_request_pub = self.create_publisher(
            Bool, "/frontier_handoff_requested", 10
        )
        self._navigation = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses"
        )
        self._tf_buffer = Buffer(
            cache_time=Duration(seconds=120.0)
        )
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
        self.create_subscription(
            Bool, "/frontier_navigation_complete",
            self._frontier_complete_callback, 10,
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

    def _frontier_complete_callback(self, message):
        """Resume wall acquisition only after the requested frontier arrives."""
        if not message.data or not self._frontier_handoff_active:
            return
        self._frontier_handoff_active = False
        self._cooldown_until = None
        self._alignment_complete = False
        self._alignment_pending = False
        self._alignment_claimed_at = None
        self._trace_start_time = None
        self._history.clear()
        self._frontier_request_pub.publish(Bool(data=False))
        self._set_state("frontier_reached_reenable_wall_alignment")
        self.get_logger().info(
            "Nearest frontier reached; wall alignment may take control again."
        )

    def _scan_callback(self, message):
        self._scan = message
        if self._scan_start_time is None:
            self._scan_start_time = self.get_clock().now()

    def _odom_callback(self, message):
        previous_pose = self._pose
        self._pose = (
            message.pose.pose.position.x, message.pose.pose.position.y
        )
        self._yaw = self._yaw_from_quaternion(message.pose.pose.orientation)
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
            self._travel_history.append((
                self.get_clock().now(), self._pose[0], self._pose[1]
            ))
            self._travel_history = self._travel_history[-20000:]

    def _map_callback(self, message):
        values = np.asarray(message.data, dtype=np.int16)
        free = np.count_nonzero((values >= 0) & (values <= 20))
        self._free_area = free * message.info.resolution ** 2

    def _yield_to_global_planner(
        self, reason, now, state="yield_to_global_exploration"
    ):
        cooldown = float(self.get_parameter("global_planning_cooldown").value)
        self._cooldown_until = now + Duration(seconds=cooldown)
        self._trace_start_time = None
        self._history.clear()
        self._alignment_complete = False
        self._alignment_pending = False
        self._alignment_claimed_at = None
        self._frontier_handoff_active = True
        self._frontier_request_pub.publish(Bool(data=True))
        self._set_state(state)
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

    def _current_path_is_retraced(self, now):
        """Require a sustained overlap with older travel, not one crossing."""
        if self._last_retrace_check is not None and (
            now - self._last_retrace_check
        ).nanoseconds < 1e9:
            return False
        self._last_retrace_check = now
        minimum_length = float(
            self.get_parameter("retrace_minimum_length").value
        )
        if len(self._travel_history) < 3:
            return False
        recent = [self._travel_history[-1]]
        length = 0.0
        for point in reversed(self._travel_history[:-1]):
            newest = recent[-1]
            length += math.hypot(point[1] - newest[1], point[2] - newest[2])
            recent.append(point)
            if length >= minimum_length:
                break
        if length < minimum_length:
            return False
        minimum_age = float(self.get_parameter("retrace_minimum_age").value)
        old = np.asarray([
            (x, y) for stamp, x, y in self._travel_history
            if (now - stamp).nanoseconds / 1e9 >= minimum_age
        ], dtype=np.float64)
        if not len(old):
            return False
        radius_squared = float(
            self.get_parameter("retrace_radius").value
        ) ** 2
        matches = 0
        for _, x, y in recent:
            offsets = old - np.asarray((x, y))
            if np.any(np.sum(offsets * offsets, axis=1) <= radius_squared):
                matches += 1
        ratio = matches / len(recent)
        required = float(self.get_parameter("retrace_overlap_ratio").value)
        return ratio >= required

    def _set_state(self, state):
        if state != self._state:
            self._state = state
            self.get_logger().info(f"Clockwise tracing behavior: {state}.")
        self._state_pub.publish(String(data=state))

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

    @classmethod
    def _wall_body_offset(cls, relative_wall_yaw):
        """Distance from LiDAR to the footprint point nearest a right wall."""
        # The fitted wall's left normal points from the wall to the robot.
        normal_x = -math.sin(relative_wall_yaw)
        normal_y = math.cos(relative_wall_yaw)
        nearest_projection = min(
            normal_x * x + normal_y * y
            for x, y in cls._FOOTPRINT_FROM_LASER
        )
        return max(0.0, -nearest_projection)

    @classmethod
    def _wall_clearance(cls, lidar_distance, relative_wall_yaw):
        """Return closest footprint-to-wall clearance for a fitted line."""
        return lidar_distance - cls._wall_body_offset(relative_wall_yaw)

    def _target_lidar_wall_distance(self, relative_wall_yaw):
        """Convert configured body clearance into the equivalent LiDAR range."""
        return (
            float(self.get_parameter("target_wall_distance").value)
            + self._wall_body_offset(relative_wall_yaw)
        )

    def _wall_is_handoff_eligible(self, wall):
        if not self._wall_is_long_enough(wall):
            return False
        distance, relative_yaw, _ = wall
        target = self._target_lidar_wall_distance(relative_yaw)
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

    @staticmethod
    def _sector_distance(ranges, angles, lower_degrees, upper_degrees):
        selected = (
            np.isfinite(ranges)
            & (angles > math.radians(lower_degrees))
            & (angles < math.radians(upper_degrees))
        )
        values = ranges[selected]
        return float(np.percentile(values, 25.0)) if len(values) >= 3 else math.inf

    @classmethod
    def _sector_body_clearance(
        cls, ranges, angles, lower_degrees, upper_degrees, percentile=25.0
    ):
        """Closest outside-footprint clearance within a LiDAR sector."""
        selected = (
            np.isfinite(ranges)
            & (angles > math.radians(lower_degrees))
            & (angles < math.radians(upper_degrees))
        )
        if np.count_nonzero(selected) < 3:
            return math.inf
        sector_angles = angles[selected]
        support = np.asarray([
            max(
                math.cos(angle) * x + math.sin(angle) * y
                for x, y in cls._FOOTPRINT_FROM_LASER
            )
            for angle in sector_angles
        ])
        clearances = ranges[selected] - support
        return float(np.percentile(clearances, percentile))

    @staticmethod
    def _clockwise_angle_from(start, current):
        """Positive clockwise rotation from start to current, in radians."""
        # Normalize to the shortest signed rotation. Tiny counterclockwise
        # odometry noise must not wrap to almost 2*pi and complete a turn in
        # one control tick.
        signed = math.atan2(math.sin(start - current), math.cos(start - current))
        return max(0.0, signed)

    def _outside_corner_detected(self, ranges, angles, front_distance, now):
        """Latch a convex right corner after the nearby side wall ends."""
        # A direct-right wall is parallel to the robot, so include the right
        # footprint extent when converting desired body clearance to range.
        target = self._target_lidar_wall_distance(0.0)
        margin = float(self.get_parameter("outside_corner_loss_margin").value)
        side_distance = self._sector_distance(ranges, angles, -105.0, -75.0)
        side_present = side_distance <= target + margin
        if side_present:
            self._right_side_seen = True
            self._right_side_lost_since = None
            return False
        if not self._right_side_seen or front_distance < float(
            self.get_parameter("front_stop_distance").value
        ):
            self._right_side_lost_since = None
            return False
        if self._right_side_lost_since is None:
            self._right_side_lost_since = now
            return False
        confirmation = float(
            self.get_parameter("outside_corner_confirmation_time").value
        )
        return (now - self._right_side_lost_since).nanoseconds >= confirmation * 1e9

    def _narrow_corridor_ahead(self, ranges, angles, now):
        """Return the entrance distance/width of an unsafe forward passage."""
        valid = np.isfinite(ranges)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        lookahead = float(self.get_parameter("corridor_lookahead").value)
        minimum_width = float(
            self.get_parameter("minimum_corridor_width").value
        )
        # Measure paired wall returns in short forward slices. Requiring two
        # adjacent narrow slices prevents a chair leg or a single noisy beam
        # from becoming a virtual wall.
        slice_depth = 0.35
        estimates = []
        for distance in np.arange(0.70, lookahead + 0.01, 0.35):
            nearby = np.abs(x - distance) <= slice_depth
            left = y[nearby & (y > 0.30)]
            right = y[nearby & (y < -0.30)]
            if len(left) < 2 or len(right) < 2:
                continue
            width = float(np.percentile(left, 20.0)) - float(
                np.percentile(right, 80.0)
            )
            estimates.append((distance, width))
        candidate = None
        for first, second in zip(estimates, estimates[1:]):
            adjacent = second[0] - first[0] <= 0.71
            both_narrow = (
                first[1] < minimum_width and second[1] < minimum_width
            )
            if adjacent and both_narrow:
                candidate = (first[0], max(first[1], second[1]))
                break
        if candidate is None:
            self._narrow_corridor_since = None
            self._narrow_corridor_width = None
            return None
        if self._narrow_corridor_since is None:
            self._narrow_corridor_since = now
            self._narrow_corridor_width = candidate[1]
            return None
        self._narrow_corridor_width = min(
            self._narrow_corridor_width, candidate[1]
        )
        confirmation = float(
            self.get_parameter("narrow_corridor_confirmation_time").value
        )
        elapsed_ns = (now - self._narrow_corridor_since).nanoseconds
        if elapsed_ns < confirmation * 1e9:
            return None
        return candidate[0], self._narrow_corridor_width

    def _approaching_prior_trace(self, now):
        """True near an older recorded pose, excluding the current trace tail."""
        if self._pose is None:
            return False
        radius = float(self.get_parameter("revisited_right_turn_radius").value)
        age_limit = float(
            self.get_parameter("revisited_right_turn_minimum_age").value
        )
        return any(
            (now - stamp).nanoseconds >= age_limit * 1e9
            and math.hypot(self._pose[0] - x, self._pose[1] - y) <= radius
            for stamp, x, y in self._history
        )

    def _start_outside_corner(self, now, reason):
        self._outside_corner_start_yaw = self._yaw
        self._outside_corner_started_at = now
        self._set_state("outside_corner_turn_right_90")
        self.get_logger().info(reason)

    def _run_outside_corner(self, ranges, angles, now):
        """Follow a committed clockwise arc until the new wall is acquired."""
        command = Twist()
        command.linear.x = float(
            self.get_parameter("outside_corner_linear_speed").value
        )
        command.angular.z = -float(
            self.get_parameter("outside_corner_turn_speed").value
        )
        turned = (
            self._clockwise_angle_from(self._outside_corner_start_yaw, self._yaw)
            if self._yaw is not None and self._outside_corner_start_yaw is not None
            else 0.0
        )
        elapsed = (
            (now - self._outside_corner_started_at).nanoseconds / 1e9
            if self._outside_corner_started_at is not None else 0.0
        )
        target_angle = float(
            self.get_parameter("outside_corner_turn_angle").value
        )
        timeout = float(self.get_parameter("outside_corner_timeout").value)
        # Do not accept the old wall behind the robot as reacquisition.  The
        # new continuous face must appear in the direct-right sector after a
        # substantial part of the turn has completed.
        side_distance = self._sector_distance(ranges, angles, -105.0, -75.0)
        target_distance = self._target_lidar_wall_distance(0.0)
        acquired = turned >= 0.80 * target_angle and side_distance <= (
            target_distance
            + float(self.get_parameter("outside_corner_loss_margin").value)
        )
        if acquired or turned >= target_angle or elapsed >= timeout:
            self._outside_corner_start_yaw = None
            self._outside_corner_started_at = None
            self._right_side_lost_since = None
            self._right_side_seen = acquired
            self._progress_pose, self._progress_time = self._pose, now
            commitment = float(
                self.get_parameter("right_turn_commitment_time").value
            )
            self._right_turn_priority_until = now + Duration(seconds=commitment)
            self._set_state("acquire_right_wall")
        return command

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
        # from a right-side wall toward the robot. Project the whole footprint
        # on that normal so Nav2 positions the nearest body point—not the
        # LiDAR or base origin—at the configured clearance.
        normal_x = -math.sin(relative_wall_yaw)
        normal_y = math.cos(relative_wall_yaw)
        target_lidar_distance = self._target_lidar_wall_distance(
            relative_wall_yaw
        )
        correction = target_lidar_distance - distance
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
        self._alignment_started_at = now
        self._alignment_safety_abort = False
        self._set_state("nav2_align_parallel_to_right_wall")
        self.get_logger().info(
            f"Nav2 aligning to a right wall with "
            f"{self._wall_clearance(distance, relative_wall_yaw):.2f} m "
            f"nearest-body clearance, then "
            f"driving {straight_distance:.1f} m parallel at "
            f"{float(self.get_parameter('target_wall_distance').value):.2f} m "
            f"body clearance along a {wall_span:.1f} m LiDAR wall fit. "
            "Nav2 now has exclusive cmd_vel ownership until this goal ends."
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
        if self._alignment_safety_abort:
            handle.cancel_goal_async()

    def _alignment_goal_result(self, future):
        result = future.result()
        self._alignment_goal_handle = None
        self._alignment_pending = False
        self._alignment_started_at = None
        if self._alignment_safety_abort and not self._returning_home:
            self._alignment_safety_abort = False
            self._alignment_complete = True
            self._alignment_claimed_at = None
            self._progress_pose = self._pose
            self._progress_time = self.get_clock().now()
            self._set_state("wall_alignment_aborted_to_safe_direct_control")
            self.get_logger().warn(
                "Nav2 wall alignment aborted; footprint-aware direct wall "
                "control is taking over."
            )
            return
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
            if self._frontier_handoff_active:
                self.get_logger().warn(
                    "Frontier handoff timed out; allowing wall alignment to "
                    "retry so navigation cannot remain idle.",
                    throttle_duration_sec=5.0,
                )
                self._frontier_handoff_active = False
                self._frontier_request_pub.publish(Bool(data=False))
            self._cooldown_until = None
            self._progress_pose, self._progress_time = self._pose, now
        scan = self._scan
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        front_distance = self._sector_body_clearance(
            ranges, angles, -22.5, 22.5, percentile=0.0
        )
        wall = self._right_wall(ranges, angles)
        if (bool(self.get_parameter("align_with_nav2").value)
                and not self._alignment_complete):
            # Once the action is submitted, Nav2 must be the sole velocity
            # owner. Publishing Twist() here used to race Nav2's nonzero
            # commands at 10 Hz, leaving the Gazebo wheels almost stationary
            # until the progress checker aborted alignment and recovery.
            if self._alignment_pending:
                wall_clearance = (
                    self._wall_clearance(wall[0], wall[1])
                    if wall is not None else math.inf
                )
                timeout = float(
                    self.get_parameter("alignment_navigation_timeout").value
                )
                timed_out = (
                    self._alignment_started_at is not None
                    and (now - self._alignment_started_at).nanoseconds / 1e9
                    >= timeout
                )
                unsafe = (
                    front_distance < float(
                        self.get_parameter("front_stop_distance").value
                    )
                    or wall_clearance < float(
                        self.get_parameter("emergency_front_distance").value
                    )
                )
                if unsafe or timed_out:
                    self._alignment_safety_abort = True
                    if self._alignment_goal_handle is not None:
                        self._alignment_goal_handle.cancel_goal_async()
                    self._cmd.publish(Twist())
                    reason = "unsafe body clearance" if unsafe else "30 s timeout"
                    self._set_state("cancel_nav2_alignment_for_safety")
                    self.get_logger().warn(
                        f"Canceling Nav2 wall alignment: {reason}.",
                        throttle_duration_sec=2.0,
                    )
                    return
                self._active.publish(Bool(data=True))
                self._set_state("nav2_align_parallel_to_right_wall")
                return
            if wall is None:
                self._active.publish(Bool(data=False))
                self._set_state("waiting_for_right_wall_to_align")
                return
            if not self._wall_is_long_enough(wall):
                self._active.publish(Bool(data=False))
                self._set_state("global_planning_until_wall_exceeds_5m")
                return
            self._active.publish(Bool(data=True))
            wall_clearance = self._wall_clearance(wall[0], wall[1])
            escape_clearance = float(
                self.get_parameter("alignment_escape_clearance").value
            )
            if wall_clearance < escape_clearance:
                # Nav2 correctly refuses every trajectory when its padded
                # footprint begins in collision.  An in-place turn can remain
                # pinned indefinitely because a rectangular chassis sweeps a
                # corner into the wall.  Crawl forward on a left arc when the
                # complete forward footprint is clear; translation then grows
                # right-side clearance while rotation points the robot away.
                self._alignment_claimed_at = None
                self._set_state("wall_clearance_escape_before_nav2")
                command = Twist()
                if front_distance > float(
                    self.get_parameter("front_stop_distance").value
                ):
                    command.linear.x = min(
                        0.08,
                        float(self.get_parameter("linear_speed").value),
                    )
                command.angular.z = float(
                    self.get_parameter("turn_speed").value
                )
                self._cmd.publish(command)
                self.get_logger().warn(
                    f"Only {wall_clearance:.2f} m body clearance; arcing "
                    f"away from the wall until {escape_clearance:.2f} m "
                    "before asking Nav2 to align.",
                    throttle_duration_sec=2.0,
                )
                return
            # Stop direct wall control only during the short settling period;
            # _start_wall_alignment() transfers command ownership to Nav2.
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
        outside_corner_active = self._outside_corner_start_yaw is not None
        if outside_corner_active:
            recently_seen = True
        if self._dead_end_escape_active:
            recently_seen = True
        stale_reason = (
            self._exploration_is_stale(now)
            if (recently_seen and not self._dead_end_escape_active
                and not outside_corner_active)
            else False
        )
        retraced_path = (
            self._current_path_is_retraced(now)
            if (recently_seen and not self._dead_end_escape_active
                and not outside_corner_active)
            else False
        )
        if retraced_path:
            self._yield_to_global_planner(
                "the current path substantially overlaps older robot travel",
                now,
                state="retrace_detected_request_nearest_frontier",
            )
            self._active.publish(Bool(data=False))
            self._cmd.publish(Twist())
            return
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
                self._set_state("inside_corner_left_90")
            else:
                command.linear.x = -float(
                    self.get_parameter("backup_speed").value
                )
                self._cmd.publish(command)
                return

        right_front = self._sector_body_clearance(
            ranges, angles, -70.0, -15.0
        )
        # An inside corner's front wall becomes the new right wall after the
        # left turn. Begin the maneuver at the requested wall-follow clearance
        # so the completed turn does not leave the chassis pinned too close.
        corner_turn_clearance = max(
            float(self.get_parameter("front_stop_distance").value),
            float(
                self.get_parameter("inside_corner_front_clearance").value
            ),
        )
        open_right_at_front = (
            self._yaw is not None
            and front_distance < 1.5 * corner_turn_clearance
            and front_distance > float(
                self.get_parameter("emergency_front_distance").value
            )
            and right_front >= float(
                self.get_parameter("revisited_right_turn_clearance").value
            )
        )
        if open_right_at_front:
            self._start_outside_corner(
                now,
                "Front wall with footprint-safe open right branch; giving "
                "the clockwise turn priority over the narrow-corridor fallback.",
            )
            self._cmd.publish(self._run_outside_corner(ranges, angles, now))
            return

        narrow_corridor = self._narrow_corridor_ahead(ranges, angles, now)
        if narrow_corridor is not None:
            entrance_distance, measured_width = narrow_corridor
            # Treat the entrance plane exactly like a front obstacle. The
            # normal inside-corner left turn therefore keeps the chassis out.
            # Corridor entrance distance is measured from the LiDAR. Convert
            # it to clearance from the foremost footprint point.
            entrance_clearance = entrance_distance - max(
                x for x, _ in self._FOOTPRINT_FROM_LASER
            )
            front_distance = min(front_distance, entrance_clearance)
            if front_distance < corner_turn_clearance:
                self._outside_corner_start_yaw = None
                self._outside_corner_started_at = None
                self._right_side_lost_since = None
                self._set_state("narrow_corridor_virtual_wall")
                command.linear.x = 0.0
                command.angular.z = float(
                    self.get_parameter("turn_speed").value
                )
                minimum_width = float(
                    self.get_parameter("minimum_corridor_width").value
                )
                self.get_logger().warn(
                    f"Blocking {measured_width:.2f} m corridor at "
                    f"{entrance_distance:.2f} m; minimum safe width is "
                    f"{minimum_width:.2f} m.",
                    throttle_duration_sec=2.0,
                )
                self._cmd.publish(command)
                return

        if outside_corner_active:
            self._set_state("outside_corner_turn_right_90")
            self._cmd.publish(self._run_outside_corner(ranges, angles, now))
            return

        revisited_right_turn = (
            self._yaw is not None
            and self._approaching_prior_trace(now)
            and front_distance < 1.5 * corner_turn_clearance
            and front_distance > float(
                self.get_parameter("emergency_front_distance").value
            )
            and right_front >= float(
                self.get_parameter("revisited_right_turn_clearance").value
            )
        )
        if revisited_right_turn:
            self._start_outside_corner(
                now,
                "Previously traveled approach with an open right branch; "
                "giving the committed clockwise turn priority.",
            )
            self._cmd.publish(self._run_outside_corner(ranges, angles, now))
            return

        if self._outside_corner_detected(
            ranges, angles, front_distance, now
        ) and self._yaw is not None:
            self._start_outside_corner(
                now,
                "Right wall edge confirmed; committing to the continuous "
                "outside-corner turn.",
            )
            self._cmd.publish(self._run_outside_corner(ranges, angles, now))
            return

        dead_end = (
            not self._dead_end_escape_active
            and front_distance < corner_turn_clearance
            and wall is not None
            and self._side_wall_present(ranges, angles, left=True)
        )
        if dead_end:
            self._start_dead_end_escape()
            self._set_state("inside_dead_end_turn_180")
            command.angular.z = float(self.get_parameter("turn_speed").value)
            self._cmd.publish(command)
            return

        if self._update_progress():
            self._backup_start = self._pose
            self._set_state("backup")
            command.linear.x = -float(self.get_parameter("backup_speed").value)
        elif (
            front_distance < corner_turn_clearance
            and self._right_turn_priority_until is not None
            and now < self._right_turn_priority_until
            and front_distance > float(
                self.get_parameter("emergency_front_distance").value
            )
        ):
            self._set_state("right_turn_commitment_guard")
            command.linear.x = float(
                self.get_parameter("outside_corner_linear_speed").value
            )
            command.angular.z = -float(
                self.get_parameter("outside_corner_turn_speed").value
            )
        elif front_distance < corner_turn_clearance:
            self._set_state("inside_corner_left_90")
            command.linear.x = 0.04
            command.angular.z = float(self.get_parameter("turn_speed").value)
        elif wall is None:
            self._set_state("outside_corner_reacquire_right_270")
            command.linear.x = 0.08
            command.angular.z = -float(self.get_parameter("turn_speed").value)
        else:
            distance, wall_yaw, _ = wall
            target = self._target_lidar_wall_distance(wall_yaw)
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
