#!/usr/bin/env python3
"""OpenCV center-biased frontier exploration which sends smooth routes to Nav2."""

import heapq
import math
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from tf2_ros import Buffer, TransformException, TransformListener
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

GridPoint = Tuple[int, int]


class WanderingMapper(Node):
    def __init__(self):
        super().__init__("wandering_mapper")
        defaults = {
            "map_topic": "/map", "map_frame": "map", "robot_frame": "base_footprint",
            "free_threshold": 20, "robot_clearance": 0.58, "goal_clearance": 0.62,
            "minimum_frontier_size": 0.50, "waypoint_spacing": 0.75,
            "path_simplification": 0.22, "frontier_standoff": 0.45,
            "planning_period": 0.50, "maximum_route_poses": 40,
            "clearance_weight": 5.0, "completion_cycles": 4,
            "minimum_goal_distance": 1.80, "minimum_route_length": 2.00,
            "minimum_route_poses": 3, "information_radius": 1.50,
            "information_weight": 4.0, "frontier_bonus": 6.0,
            "travel_weight": 0.20, "revisit_weight": 3.0,
            "visited_radius": 0.75, "visit_record_spacing": 0.25,
            "unavoidable_transit_threshold": 1.0,
            "wall_transit_distance": 1.05, "wall_transit_weight": 4.0,
            "unavoidable_route_horizon": 25.0,
            # A new SLAM map is usually only a small free patch.  Requiring a
            # long first route creates a deadlock: exploration cannot move
            # until the map grows, and the map cannot grow until exploration
            # moves.  These are preferences; shorter safe routes are allowed.
            "startup_grace_cycles": 8, "fallback_goal_distance": 0.80,
            "fallback_route_length": 0.35,
            "route_horizon": 10.00, "continuity_weight": 0.80,
            "forward_weight": 2.0, "reverse_penalty": 6.0,
            "frontier_lateral_penalty": 0.40,
            "corridor_lookahead": 6.0, "corridor_half_width": 4.0,
            "corridor_bin_width": 0.75, "minimum_corridor_width": 1.20,
            "corridor_weight": 8.0,
            "lidar_corridor_lookahead": 10.5,
            "lidar_corridor_margin": 0.55,
            "lidar_corridor_weight": 2.0,
            "long_path_weight": 2.0, "turn_penalty": 1.5,
            # Extend only near the route tail and only after this much stable
            # execution, avoiding the old rapid-preemption deadlock.
            "rolling_replan_poses": 4, "rolling_replan_distance": 4.0,
            "route_extension_period": 3.0,
            "wall_heading_radius": 1.50,
            "progress_timeout": 20.0, "significant_progress": 0.25,
            "discovery_progress_timeout": 30.0,
            "minimum_discovery_area_gain": 1.0,
            "stuck_radius": 2.0, "stuck_timeout": 120.0,
            "recovery_backup_distance": 2.14, "recovery_backup_speed": 0.15,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._map: Optional[OccupancyGrid] = None
        self._scan: Optional[LaserScan] = None
        self._wall_tracing = False
        self._preview_poses = []
        self._busy = False
        self._goal_handle = None
        self._route_generation = 0
        self._rolling_replan_started = False
        self._recovery_requested = False
        self._global_recovery_requested = False
        self._best_distance = None
        self._last_progress_time = None
        self._mapped_free_area = 0.0
        self._discovery_start_area = 0.0
        self._discovery_start_time = None
        self._unavoidable_transit = False
        self._stuck_anchor = None
        self._stuck_anchor_time = None
        self._force_least_explored = False
        self._target = (0.0, 0.0)
        self._has_target = False
        self._blacklist: List[Tuple[float, float]] = []
        self._empty_cycles = 0
        self._origin = None
        self._returning_home = False
        self._visited_world: List[Tuple[float, float]] = []
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._action = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self._backup_action = ActionClient(self, BackUp, "backup")
        self._path_pub = self.create_publisher(Path, "wandering_path", qos)
        self._return_home_pub = self.create_publisher(
            Bool, "/mapping_return_home", 10
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map_topic").value),
            self._map_callback, qos,
        )
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool, "/wall_tracing_active", self._wall_trace_callback, 10,
        )
        self._timer = self.create_timer(
            float(self.get_parameter("planning_period").value), self._plan
        )
        self.get_logger().info("Waiting for map, robot TF, and active Nav2.")

    def _map_callback(self, message):
        self._map = message
        values = np.asarray(message.data, dtype=np.int16)
        threshold = int(self.get_parameter("free_threshold").value)
        free_cells = np.count_nonzero(
            (values >= 0) & (values <= threshold)
        )
        self._mapped_free_area = free_cells * message.info.resolution ** 2

    def _scan_callback(self, message):
        self._scan = message

    def _wall_trace_callback(self, message):
        was_active = self._wall_tracing
        self._wall_tracing = message.data
        if self._wall_tracing and not was_active and self._goal_handle is not None:
            self.get_logger().info(
                "Clockwise wall behavior active; yielding Nav2 route control."
            )
            self._goal_handle.cancel_goal_async()
        elif was_active and not self._wall_tracing:
            self.get_logger().info(
                "Clockwise tracing yielded; refreshing the prepared global route."
            )
            # The timer will also plan, but an immediate refresh minimizes the
            # handoff gap and accounts for the robot's newest pose.
            if not self._busy:
                self._plan()

    def _robot_world(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("robot_frame").value), rclpy.time.Time()
            )
        except TransformException as error:
            self.get_logger().warn(f"Waiting for robot TF: {error}", throttle_duration_sec=5.0)
            return None
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
        )

    def _update_stuck_state(self, x, y, now=None):
        """Return true only after staying within the anchor radius long enough."""
        if now is None:
            now = self.get_clock().now()
        radius = float(self.get_parameter("stuck_radius").value)
        timeout = float(self.get_parameter("stuck_timeout").value)
        if self._stuck_anchor is None or self._stuck_anchor_time is None:
            self._stuck_anchor = (x, y)
            self._stuck_anchor_time = now
            return False
        displacement = math.hypot(
            x - self._stuck_anchor[0], y - self._stuck_anchor[1]
        )
        if displacement >= radius:
            self._stuck_anchor = (x, y)
            self._stuck_anchor_time = now
            return False
        elapsed = (now - self._stuck_anchor_time).nanoseconds / 1e9
        return elapsed >= timeout

    @staticmethod
    def _inside(mask, point):
        return 0 <= point[0] < mask.shape[0] and 0 <= point[1] < mask.shape[1]

    @staticmethod
    def _world_to_grid(message, x, y):
        resolution = message.info.resolution
        return (int((y - message.info.origin.position.y) // resolution),
                int((x - message.info.origin.position.x) // resolution))

    @staticmethod
    def _grid_to_world(message, point):
        resolution = message.info.resolution
        return (message.info.origin.position.x + (point[1] + 0.5) * resolution,
                message.info.origin.position.y + (point[0] + 0.5) * resolution)

    def _masks(self, message, robot):
        values = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width
        )
        threshold = int(self.get_parameter("free_threshold").value)
        free = (values >= 0) & (values <= threshold)
        # Measure clearance from confirmed obstacles, not from unknown cells.
        # Early SLAM maps contain thin free laser rays separated by unknown
        # pixels; treating every unknown pixel as a wall made even the robot's
        # starting cell appear unsafe. Routes are still restricted to confirmed
        # free cells below, so this does not permit motion into unknown space.
        occupied = values > threshold
        clearance = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5)
        radius = float(self.get_parameter("robot_clearance").value) / message.info.resolution
        traversable = free & (clearance >= radius)
        if not self._inside(traversable, robot):
            return None
        if not traversable[robot]:
            cells = np.argwhere(traversable)
            if not len(cells):
                return None
            robot = tuple(cells[np.argmin(np.sum((cells - robot) ** 2, axis=1))])
        count, labels = cv2.connectedComponents(
            traversable.astype(np.uint8), connectivity=8
        )
        if count < 2 or labels[robot] == 0:
            return None
        reachable = labels == labels[robot]
        return values, free, clearance, reachable, robot

    def _frontiers(self, values, free, reachable, resolution):
        unknown = values < 0
        frontier = free & (cv2.dilate(unknown.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0)
        standoff = max(1, int(math.ceil(
            float(self.get_parameter("frontier_standoff").value) / resolution
        )))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * standoff + 1,) * 2)
        frontier &= cv2.dilate(reachable.astype(np.uint8), kernel) > 0
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            frontier.astype(np.uint8), connectivity=8
        )
        minimum = max(2, int(
            float(self.get_parameter("minimum_frontier_size").value) / resolution
        ))
        return [np.argwhere(labels == label) for label in range(1, count)
                if stats[label, cv2.CC_STAT_AREA] >= minimum]

    def _goal_cell(self, message, component, reachable, clearance, robot,
                   minimum_goal_distance=None, robot_yaw=None):
        if robot_yaw is None:
            center = np.mean(component, axis=0)
        else:
            offsets = component - np.asarray(robot)
            forward = (
                offsets[:, 1] * math.cos(robot_yaw)
                + offsets[:, 0] * math.sin(robot_yaw)
            )
            lateral = np.abs(
                -offsets[:, 1] * math.sin(robot_yaw)
                + offsets[:, 0] * math.cos(robot_yaw)
            )
            target_scores = forward - float(
                self.get_parameter("frontier_lateral_penalty").value
            ) * lateral
            center = component[int(np.argmax(target_scores))]
        resolution = message.info.resolution
        search = max(2, int(1.5 / resolution))
        row, col = center.astype(int)
        r0, r1 = max(0, row - search), min(reachable.shape[0], row + search + 1)
        c0, c1 = max(0, col - search), min(reachable.shape[1], col + search + 1)
        cells = np.argwhere(reachable[r0:r1, c0:c1])
        if not len(cells):
            return None
        cells += (r0, c0)
        clear = clearance[cells[:, 0], cells[:, 1]]
        distance = np.linalg.norm(cells - center, axis=1)
        required = float(self.get_parameter("goal_clearance").value) / resolution
        # Stay close to the useful frontier once the footprint clearance has
        # been met. Excess clearance is only a tie-breaker; otherwise goals
        # collapse back toward the already mapped middle of the room.
        scores = -distance + 0.10 * np.minimum(clear, required * 2.0)
        scores[clear < required] -= 1000.0
        if minimum_goal_distance is None:
            minimum_goal_distance = float(
                self.get_parameter("minimum_goal_distance").value
            )
        minimum_distance = minimum_goal_distance / resolution
        robot_distance = np.linalg.norm(cells - np.asarray(robot), axis=1)
        scores[robot_distance < minimum_distance] = -np.inf
        for index in np.argsort(scores)[::-1]:
            if not np.isfinite(scores[index]):
                continue
            point = tuple(int(value) for value in cells[index])
            world = self._grid_to_world(message, point)
            if all(math.hypot(world[0] - x, world[1] - y) > 1.5
                   for x, y in self._blacklist):
                return point
        return None

    def _information_gain(self, values, component, resolution):
        """Return square metres of unknown map observable near a frontier."""
        frontier_mask = np.zeros(values.shape, dtype=np.uint8)
        frontier_mask[component[:, 0], component[:, 1]] = 1
        radius = max(1, int(math.ceil(
            float(self.get_parameter("information_radius").value) / resolution
        )))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1,) * 2)
        observable = cv2.dilate(frontier_mask, kernel) > 0
        return float(np.count_nonzero(observable & (values < 0))) * resolution ** 2

    def _corridor_goal(self, message, values, reachable, clearance,
                       robot_x, robot_y, robot_yaw):
        """Infer a forward hallway center from paired left and right walls."""
        threshold = int(self.get_parameter("free_threshold").value)
        walls = np.argwhere(values > threshold)
        if len(walls) < 2:
            return None
        resolution = message.info.resolution
        wall_x = message.info.origin.position.x + (walls[:, 1] + 0.5) * resolution
        wall_y = message.info.origin.position.y + (walls[:, 0] + 0.5) * resolution
        dx, dy = wall_x - robot_x, wall_y - robot_y
        forward = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        lateral = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
        lookahead = float(self.get_parameter("corridor_lookahead").value)
        half_width = float(self.get_parameter("corridor_half_width").value)
        useful = ((forward > 0.5) & (forward <= lookahead)
                  & (np.abs(lateral) <= half_width))
        forward, lateral = forward[useful], lateral[useful]
        if not len(forward):
            return None
        bin_width = float(self.get_parameter("corridor_bin_width").value)
        minimum_width = float(
            self.get_parameter("minimum_corridor_width").value
        )
        estimates = []
        for distance in np.arange(0.75, lookahead + 0.01, bin_width):
            nearby = np.abs(forward - distance) <= bin_width
            left = lateral[nearby & (lateral > 0.0)]
            right = lateral[nearby & (lateral < 0.0)]
            if not len(left) or not len(right):
                continue
            left_wall, right_wall = float(np.min(left)), float(np.max(right))
            width = left_wall - right_wall
            if width >= minimum_width:
                estimates.append((distance, 0.5 * (left_wall + right_wall), width))
        if not estimates:
            return None
        distance, center_offset, width = estimates[-1]
        target_x = (robot_x + distance * math.cos(robot_yaw)
                    - center_offset * math.sin(robot_yaw))
        target_y = (robot_y + distance * math.sin(robot_yaw)
                    + center_offset * math.cos(robot_yaw))
        target = np.asarray(self._world_to_grid(message, target_x, target_y))
        search = max(2, int(math.ceil(bin_width / resolution)))
        r0, r1 = max(0, target[0] - search), min(values.shape[0], target[0] + search + 1)
        c0, c1 = max(0, target[1] - search), min(values.shape[1], target[1] + search + 1)
        cells = np.argwhere(reachable[r0:r1, c0:c1])
        if not len(cells):
            return None
        cells += (r0, c0)
        offsets = np.linalg.norm(cells - target, axis=1)
        cell_clearance = clearance[cells[:, 0], cells[:, 1]]
        scores = -offsets + 0.25 * cell_clearance
        for index in np.argsort(scores)[::-1]:
            goal = tuple(int(value) for value in cells[index])
            world = self._grid_to_world(message, goal)
            if all(math.hypot(world[0] - x, world[1] - y) > 1.5
                   for x, y in self._blacklist):
                return goal, distance, width
        return None

    def _lidar_corridor(self, message, values, robot_x, robot_y, robot_yaw):
        """Return a temporary free corridor and goal inferred from live scan."""
        if self._scan is None or not self._scan.ranges:
            return None
        scan = self._scan
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        usable = np.isfinite(ranges) & (ranges >= scan.range_min)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        maximum = float(self.get_parameter("lidar_corridor_lookahead").value)
        wall_zone = usable & (x > -0.25) & (x < maximum + 1.0)
        left = np.column_stack((x[wall_zone & (y > 0.35)], y[wall_zone & (y > 0.35)]))
        right = np.column_stack((x[wall_zone & (y < -0.35)], y[wall_zone & (y < -0.35)]))
        if len(left) < 8 or len(right) < 8:
            return None
        forward_ranges = ranges[np.abs(angles) < math.radians(22.5)]
        forward_ranges = np.where(np.isfinite(forward_ranges), forward_ranges, scan.range_max)
        if not len(forward_ranges):
            return None
        distance = min(maximum, float(np.percentile(forward_ranges, 15)) - 1.10)
        if distance < 1.0:
            return None

        def wall_y(points, at_x):
            vx, vy, px, py = cv2.fitLine(
                points.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01
            ).reshape(-1)
            if abs(vx) < 0.10:
                return float(np.median(points[:, 1]))
            return float(py + (at_x - px) * vy / vx)

        left_y, right_y = wall_y(left, distance), wall_y(right, distance)
        width = left_y - right_y
        if width < float(self.get_parameter("minimum_corridor_width").value):
            return None
        center = 0.5 * (left_y + right_y)
        margin = float(self.get_parameter("lidar_corridor_margin").value)
        half_safe = max(0.10, 0.5 * width - margin)
        corridor = np.zeros(values.shape, dtype=np.uint8)
        start = self._world_to_grid(message, robot_x, robot_y)
        candidates = []
        center_angle = math.atan2(center, distance)
        for turn in (-math.radians(25.0), 0.0, math.radians(25.0)):
            heading = center_angle + turn
            angle_error = np.arctan2(
                np.sin(angles - heading), np.cos(angles - heading)
            )
            sector_ranges = ranges[np.abs(angle_error) < math.radians(7.5)]
            sector_ranges = np.where(
                np.isfinite(sector_ranges), sector_ranges, scan.range_max
            )
            if not len(sector_ranges):
                continue
            candidate_distance = min(
                maximum, float(np.percentile(sector_ranges, 20)) - 1.10
            )
            if candidate_distance < 1.5:
                continue
            local_x = candidate_distance * math.cos(heading)
            local_y = candidate_distance * math.sin(heading)
            goal_x = (robot_x + local_x * math.cos(robot_yaw)
                      - local_y * math.sin(robot_yaw))
            goal_y = (robot_y + local_x * math.sin(robot_yaw)
                      + local_y * math.cos(robot_yaw))
            goal = self._world_to_grid(message, goal_x, goal_y)
            thickness = max(3, int(2.0 * half_safe / message.info.resolution))
            cv2.line(
                corridor, (start[1], start[0]), (goal[1], goal[0]),
                1, thickness=thickness,
            )
            candidates.append((goal, candidate_distance, width, abs(turn)))
        corridor[values > int(self.get_parameter("free_threshold").value)] = 0
        return corridor.astype(bool), candidates

    def _astar(
        self, reachable, clearance, start, goal, resolution,
        wall_transit=False,
    ):
        moves = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                 (-1, -1, 1.414), (-1, 1, 1.414),
                 (1, -1, 1.414), (1, 1, 1.414)]
        queue = [(0.0, start)]
        previous: Dict[GridPoint, GridPoint] = {}
        costs = {start: 0.0}
        weight = float(self.get_parameter("clearance_weight").value)
        wall_distance = float(
            self.get_parameter("wall_transit_distance").value
        )
        wall_weight = float(
            self.get_parameter("wall_transit_weight").value
        )
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                break
            for dr, dc, step in moves:
                nxt = current[0] + dr, current[1] + dc
                if not self._inside(reachable, nxt) or not reachable[nxt]:
                    continue
                clearance_m = max(0.05, clearance[nxt] * resolution)
                if wall_transit:
                    # Stay at a safe, wall-following offset instead of drifting
                    # to the center of already mapped corridors.
                    preference = wall_weight * abs(
                        clearance_m - wall_distance
                    )
                else:
                    preference = weight / clearance_m
                cost = costs[current] + step * (1.0 + preference)
                if cost >= costs.get(nxt, float("inf")):
                    continue
                costs[nxt], previous[nxt] = cost, current
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(queue, (cost + heuristic, nxt))
        if goal != start and goal not in previous:
            return []
        path = [goal]
        while path[-1] != start:
            path.append(previous[path[-1]])
        return path[::-1]

    def _line_safe(self, reachable, first, second):
        samples = max(2, int(np.linalg.norm(second - first) * 2))
        for alpha in np.linspace(0, 1, samples):
            cell = tuple(np.rint(first * (1 - alpha) + second * alpha).astype(int))
            if not self._inside(reachable, cell) or not reachable[cell]:
                return False
        return True

    def _smooth(self, raw, reachable, resolution):
        contour = np.asarray([(col, row) for row, col in raw], np.float32).reshape(-1, 1, 2)
        epsilon = float(self.get_parameter("path_simplification").value) / resolution
        xy = cv2.approxPolyDP(contour, epsilon, False).reshape(-1, 2)
        points = np.asarray([(p[1], p[0]) for p in xy], np.float64)
        for _ in range(2):
            rounded = [points[0]]
            for first, second in zip(points[:-1], points[1:]):
                for candidate in (0.75 * first + 0.25 * second,
                                  0.25 * first + 0.75 * second):
                    if self._line_safe(reachable, rounded[-1], candidate):
                        rounded.append(candidate)
            if self._line_safe(reachable, rounded[-1], points[-1]):
                rounded.append(points[-1])
            points = np.asarray(rounded)
        return [tuple(np.rint(point).astype(int)) for point in points]

    def _wall_parallel_yaw(self, message, values, point, travel_yaw):
        """Fit the nearest wall and return its forward-facing parallel yaw."""
        threshold = int(self.get_parameter("free_threshold").value)
        occupied = np.argwhere(values > threshold)
        if len(occupied) < 4:
            return travel_yaw
        point_array = np.asarray(point)
        nearest = occupied[np.argmin(np.sum((occupied - point_array) ** 2, axis=1))]
        radius = float(
            self.get_parameter("wall_heading_radius").value
        ) / message.info.resolution
        wall = occupied[np.sum((occupied - nearest) ** 2, axis=1) <= radius ** 2]
        if len(wall) < 4:
            return travel_yaw
        # OpenCV line coordinates are (map x/column, map y/row), matching the
        # world map axes because OccupancyGrid resolution is isotropic.
        xy = np.column_stack((wall[:, 1], wall[:, 0])).astype(np.float32)
        vx, vy, _, _ = cv2.fitLine(
            xy, cv2.DIST_HUBER, 0, 0.01, 0.01
        ).reshape(-1)
        wall_yaw = math.atan2(float(vy), float(vx))
        reverse_yaw = wall_yaw + math.pi

        def difference(angle):
            return abs(math.atan2(
                math.sin(angle - travel_yaw), math.cos(angle - travel_yaw)
            ))

        return (
            wall_yaw
            if difference(wall_yaw) <= difference(reverse_yaw)
            else reverse_yaw
        )

    def _poses(self, message, path):
        spacing = float(self.get_parameter("waypoint_spacing").value) / message.info.resolution
        points = np.asarray(path, dtype=np.float64)
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        sample_distances = list(np.arange(spacing, cumulative[-1], spacing))
        sample_distances.append(cumulative[-1])
        selected = [path[0]]
        for sample in sample_distances:
            segment = min(
                int(np.searchsorted(cumulative, sample, side="right") - 1),
                len(lengths) - 1,
            )
            alpha = ((sample - cumulative[segment]) / lengths[segment]
                     if lengths[segment] > 0 else 0.0)
            point = points[segment] * (1.0 - alpha) + points[segment + 1] * alpha
            grid_point = tuple(np.rint(point).astype(int))
            if grid_point != selected[-1]:
                selected.append(grid_point)
        limit = int(self.get_parameter("maximum_route_poses").value)
        if limit and len(selected) > limit:
            selected = [selected[i] for i in np.linspace(0, len(selected) - 1, limit).astype(int)]
        poses = []
        values = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width
        )
        for index, point in enumerate(selected):
            other = selected[min(index + 1, len(selected) - 1)]
            if other == point and index:
                other = point
                point_for_angle = selected[index - 1]
                yaw = math.atan2(other[0] - point_for_angle[0], other[1] - point_for_angle[1])
            else:
                yaw = math.atan2(other[0] - point[0], other[1] - point[1])
            yaw = self._wall_parallel_yaw(message, values, point, yaw)
            x, y = self._grid_to_world(message, point)
            pose = PoseStamped()
            pose.header.frame_id = str(self.get_parameter("map_frame").value)
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            poses.append(pose)
        return poses[1:]

    @staticmethod
    def _route_length(path, resolution):
        return resolution * sum(
            math.hypot(second[0] - first[0], second[1] - first[1])
            for first, second in zip(path[:-1], path[1:])
        )

    def _record_visit(self, x, y):
        """Remember physical robot travel without tying it to map indices."""
        spacing = float(self.get_parameter("visit_record_spacing").value)
        if not self._visited_world or math.hypot(
            x - self._visited_world[-1][0], y - self._visited_world[-1][1]
        ) >= spacing:
            self._visited_world.append((x, y))
            self._visited_world = self._visited_world[-20000:]

    def _visited_mask(self, message):
        """Rasterize travel history into the current, possibly growing map."""
        mask = np.zeros(
            (message.info.height, message.info.width), dtype=np.uint8
        )
        for x, y in self._visited_world:
            cell = self._world_to_grid(message, x, y)
            if self._inside(mask, cell):
                mask[cell] = 1
        radius = max(1, int(
            float(self.get_parameter("visited_radius").value)
            / message.info.resolution
        ))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1)
        )
        return cv2.dilate(mask, kernel) > 0

    @staticmethod
    def _route_revisit_distance(path, visited, resolution):
        """Return route metres lying in previously traveled territory."""
        revisited = 0.0
        for first, second in zip(path[:-1], path[1:]):
            segment = resolution * math.hypot(
                second[0] - first[0], second[1] - first[1]
            )
            if visited[second]:
                revisited += segment
        return revisited

    @staticmethod
    def _novelty_score(base_score, revisit, unavoidable_revisit, weight):
        """Penalize only revisit distance for which an alternative exists."""
        return base_score - weight * max(
            0.0, revisit - unavoidable_revisit
        )

    @staticmethod
    def _rolling_horizon(path, resolution, horizon):
        """Return only the next part of a route, for frequent SLAM replans."""
        if horizon <= 0.0 or len(path) < 2:
            return path
        result = [path[0]]
        travelled = 0.0
        for first, second in zip(path[:-1], path[1:]):
            travelled += resolution * math.hypot(
                second[0] - first[0], second[1] - first[1]
            )
            result.append(second)
            if travelled >= horizon:
                break
        return result

    def _plan(self, preempting=False):
        preview_only = self._wall_tracing
        if (self._busy and not preempting) or self._map is None:
            return
        if not self._action.server_is_ready():
            self.get_logger().warn(
                "Waiting for active Nav2 action server.",
                throttle_duration_sec=5.0,
            )
            return
        world = self._robot_world()
        if world is None:
            return
        robot_x, robot_y, robot_yaw = world
        self._record_visit(robot_x, robot_y)
        if self._origin is None:
            self._origin = (robot_x, robot_y, robot_yaw)
            self.get_logger().info(
                f"Recorded mapping origin x={robot_x:.2f}, y={robot_y:.2f}."
            )
        self._update_stuck_state(robot_x, robot_y)
        observed_robot = self._world_to_grid(self._map, robot_x, robot_y)
        masks = self._masks(self._map, observed_robot)
        if masks is None:
            self.get_logger().warn(
                "Robot is not in mapped traversable space.",
                throttle_duration_sec=5.0,
            )
            return
        values, free, clearance, reachable, robot = masks
        visited = self._visited_mask(self._map)
        lidar_corridor = self._lidar_corridor(
            self._map, values, robot_x, robot_y, robot_yaw
        )
        if lidar_corridor is not None:
            corridor_mask, _ = lidar_corridor
            reachable = reachable | corridor_mask
        components = self._frontiers(
            values, free, reachable, self._map.info.resolution
        )
        ranked = []
        preferred_distance = float(
            self.get_parameter("minimum_goal_distance").value
        )
        fallback_distance = float(
            self.get_parameter("fallback_goal_distance").value
        )
        for component in components:
            goal = self._goal_cell(
                self._map, component, reachable, clearance, observed_robot,
                preferred_distance, robot_yaw,
            )
            if goal is None:
                goal = self._goal_cell(
                    self._map, component, reachable, clearance, observed_robot,
                    fallback_distance, robot_yaw,
                )
            if goal:
                distance = self._map.info.resolution * math.hypot(
                    goal[0] - robot[0], goal[1] - robot[1]
                )
                gain = self._information_gain(
                    values, component, self._map.info.resolution
                )
                world_goal = self._grid_to_world(self._map, goal)
                goal_heading = math.atan2(
                    world_goal[1] - robot_y, world_goal[0] - robot_x
                )
                alignment = math.cos(goal_heading - robot_yaw)
                if self._force_least_explored:
                    # Recovery deliberately favors the frontier with the most
                    # observable unknown area. Distance is only a tie-breaker.
                    score = gain * 1000.0 + len(component) - distance
                else:
                    score = (
                        float(self.get_parameter("frontier_bonus").value)
                        + float(self.get_parameter("information_weight").value) * gain
                        - float(self.get_parameter("travel_weight").value) * distance
                        + float(self.get_parameter("forward_weight").value)
                        * max(0.0, alignment)
                        - float(self.get_parameter("reverse_penalty").value)
                        * max(0.0, -alignment)
                    )
                if self._has_target and not self._force_least_explored:
                    score -= float(
                        self.get_parameter("continuity_weight").value
                    ) * math.hypot(
                        world_goal[0] - self._target[0],
                        world_goal[1] - self._target[1],
                    )
                ranked.append(
                    (score, goal, len(component), gain, distance, alignment)
                )
        corridor = None if self._force_least_explored else self._corridor_goal(
            self._map, values, reachable, clearance,
            robot_x, robot_y, robot_yaw,
        )
        if corridor is not None:
            goal, corridor_distance, corridor_width = corridor
            distance = self._map.info.resolution * math.hypot(
                goal[0] - robot[0], goal[1] - robot[1]
            )
            world_goal = self._grid_to_world(self._map, goal)
            goal_heading = math.atan2(
                world_goal[1] - robot_y, world_goal[0] - robot_x
            )
            alignment = math.cos(goal_heading - robot_yaw)
            score = float(self.get_parameter("corridor_weight").value)
            ranked.append((score, goal, 0, 0.0, distance, alignment))
            self.get_logger().info(
                f"Corridor center inferred {corridor_distance:.1f} m ahead "
                f"between walls {corridor_width:.1f} m apart.",
                throttle_duration_sec=5.0,
            )
        if lidar_corridor is not None and not self._force_least_explored:
            _, lidar_candidates = lidar_corridor
            for goal, lidar_distance, lidar_width, turn in lidar_candidates:
                if not self._inside(reachable, goal) or not reachable[goal]:
                    continue
                score = (
                    float(self.get_parameter("lidar_corridor_weight").value)
                    + float(self.get_parameter("long_path_weight").value)
                    * lidar_distance
                    - float(self.get_parameter("turn_penalty").value) * turn
                )
                ranked.append((score, goal, 0, 0.0, lidar_distance, math.cos(turn)))
            if lidar_candidates:
                longest = max(candidate[1] for candidate in lidar_candidates)
                self.get_logger().info(
                    f"Evaluating {len(lidar_candidates)} live lidar paths "
                    f"as far as {longest:.1f} m ahead.",
                    throttle_duration_sec=5.0,
                )
        # If there is useful mapped space ahead, do not consider a target that
        # begins with a turn into the rear half-plane.  Rear targets remain a
        # fallback when obstacles leave no forward frontier at all.
        if not self._force_least_explored:
            forward_ranked = [candidate for candidate in ranked if candidate[5] >= 0.0]
            if forward_ranked:
                ranked = forward_ranked
        ranked.sort(reverse=True)
        chosen = None
        # Prefer a substantial route, then fall back to a short safe step that
        # lets SLAM incorporate another scan and enlarge confirmed free space.
        route_limits = (
            (float(self.get_parameter("minimum_route_length").value),
             int(self.get_parameter("minimum_route_poses").value)),
            (float(self.get_parameter("fallback_route_length").value), 1),
        )
        for minimum_length, minimum_poses in route_limits:
            viable = []
            for base_score, goal, size, gain, distance, alignment in ranked[:12]:
                raw = self._astar(
                    reachable, clearance, robot, goal,
                    self._map.info.resolution,
                )
                route_length = self._route_length(
                    raw, self._map.info.resolution
                )
                if route_length < minimum_length:
                    continue
                revisit_distance = self._route_revisit_distance(
                    raw, visited, self._map.info.resolution
                )
                raw = self._rolling_horizon(
                    raw, self._map.info.resolution,
                    float(self.get_parameter("route_horizon").value),
                )
                route_length = self._route_length(
                    raw, self._map.info.resolution
                )
                poses = self._poses(
                    self._map,
                    self._smooth(raw, reachable, self._map.info.resolution),
                )
                if len(poses) >= minimum_poses:
                    viable.append((
                        base_score, goal, size, poses, gain,
                        route_length, revisit_distance,
                    ))
            if viable:
                unavoidable_revisit = min(
                    candidate[6] for candidate in viable
                )
                transit_threshold = float(
                    self.get_parameter("unavoidable_transit_threshold").value
                )
                unavoidable_transit = (
                    unavoidable_revisit >= transit_threshold
                )
                if unavoidable_transit:
                    wall_viable = []
                    for candidate in viable:
                        base_score, goal, size, _, gain, _, _ = candidate
                        wall_raw = self._astar(
                            reachable, clearance, robot, goal,
                            self._map.info.resolution, wall_transit=True,
                        )
                        wall_revisit = self._route_revisit_distance(
                            wall_raw, visited, self._map.info.resolution
                        )
                        wall_raw = self._rolling_horizon(
                            wall_raw, self._map.info.resolution,
                            float(self.get_parameter(
                                "unavoidable_route_horizon"
                            ).value),
                        )
                        wall_length = self._route_length(
                            wall_raw, self._map.info.resolution
                        )
                        wall_poses = self._poses(
                            self._map,
                            self._smooth(
                                wall_raw, reachable,
                                self._map.info.resolution,
                            ),
                        )
                        if len(wall_poses) >= minimum_poses:
                            wall_viable.append((
                                base_score, goal, size, wall_poses, gain,
                                wall_length, wall_revisit,
                            ))
                    if wall_viable:
                        viable = wall_viable
                        unavoidable_revisit = min(
                            candidate[6] for candidate in viable
                        )
                    self.get_logger().info(
                        "Unavoidable mapped-area transit: using a wall-offset "
                        "route with revisit penalties suspended.",
                        throttle_duration_sec=5.0,
                    )
                revisit_weight = (
                    0.0 if unavoidable_transit else float(
                        self.get_parameter("revisit_weight").value
                    )
                )

                def novelty_score(candidate):
                    return self._novelty_score(
                        candidate[0], candidate[6], unavoidable_revisit,
                        revisit_weight,
                    )

                _, goal, size, poses, gain, route_length, revisit_distance = max(
                    viable, key=novelty_score
                )
                avoidable_revisit = max(
                    0.0, revisit_distance - unavoidable_revisit
                )
                chosen = (
                    goal, size, poses, gain, route_length, revisit_distance,
                    avoidable_revisit, unavoidable_transit,
                )
                break
        if chosen is None:
            if preview_only:
                self.get_logger().info(
                    "Wall tracing active; no global preview route is currently "
                    "reachable.",
                    throttle_duration_sec=5.0,
                )
                return
            if preempting:
                self._busy = True
                self._rolling_replan_started = False
                return
            self._empty_cycles += 1
            required = max(
                int(self.get_parameter("completion_cycles").value),
                int(self.get_parameter("startup_grace_cycles").value),
            )
            self.get_logger().warn(
                f"No usable route: {len(components)} frontier group(s), "
                f"{len(ranked)} candidate goal(s); retry "
                f"{self._empty_cycles}/{required}.",
                throttle_duration_sec=2.0,
            )
            if self._empty_cycles >= required:
                self.get_logger().info(
                    "Mapping complete: no reachable frontiers remain; "
                    "returning to origin."
                )
                self._send_return_home()
            return
        self._empty_cycles = 0
        (
            goal, size, poses, gain, route_length, revisit_distance,
            avoidable_revisit, unavoidable_transit,
        ) = chosen
        preview = Path()
        preview.header.frame_id = str(self.get_parameter("map_frame").value)
        preview.header.stamp = self.get_clock().now().to_msg()
        preview.poses = poses
        self._path_pub.publish(preview)
        self._preview_poses = poses
        if preview_only:
            self.get_logger().info(
                f"Global route preview ready with {len(poses)} poses while "
                "clockwise tracing owns velocity control.",
                throttle_duration_sec=5.0,
            )
            return
        request = NavigateThroughPoses.Goal()
        request.poses = poses
        self._target = self._grid_to_world(self._map, goal)
        self._has_target = True
        self._busy = True
        self._rolling_replan_started = False
        self._recovery_requested = False
        self._global_recovery_requested = False
        self._best_distance = None
        self._last_progress_time = self.get_clock().now()
        self._discovery_start_area = self._mapped_free_area
        self._discovery_start_time = self.get_clock().now()
        self._unavoidable_transit = unavoidable_transit
        if self._force_least_explored:
            self.get_logger().warn(
                "Failure recovery: routing to the least-explored reachable map region."
            )
            self._force_least_explored = False
        self.get_logger().info(
            f"Exploring {size}-cell frontier ({gain:.1f} m^2 unknown) "
            f"over {route_length:.1f} m using {len(poses)} smooth poses; "
            f"{revisit_distance:.1f} m overlaps prior travel, "
            f"{avoidable_revisit:.1f} m is penalized."
        )
        self._route_generation += 1
        generation = self._route_generation
        future = self._action.send_goal_async(
            request,
            feedback_callback=lambda message, route=generation:
                self._feedback(message, route),
        )
        future.add_done_callback(
            lambda result, route=generation: self._goal_response(result, route)
        )

    def _send_return_home(self):
        if self._origin is None or self._returning_home:
            return
        self._returning_home = True
        self._return_home_pub.publish(Bool(data=True))
        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter("map_frame").value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self._origin[0]
        pose.pose.position.y = self._origin[1]
        pose.pose.orientation.z = math.sin(self._origin[2] / 2.0)
        pose.pose.orientation.w = math.cos(self._origin[2] / 2.0)
        request = NavigateThroughPoses.Goal()
        request.poses = [pose]
        self._busy = True
        self._route_generation += 1
        generation = self._route_generation
        self.get_logger().info("Sending final Nav2 route to mapping origin.")
        future = self._action.send_goal_async(request)
        future.add_done_callback(
            lambda result, route=generation: self._goal_response(result, route)
        )

    def _feedback(self, message, generation):
        if generation != self._route_generation:
            return
        feedback = message.feedback
        now = self.get_clock().now()
        distance = feedback.distance_remaining
        progress = float(self.get_parameter("significant_progress").value)
        if self._best_distance is None or distance <= self._best_distance - progress:
            self._best_distance = distance
            self._last_progress_time = now
        timeout = float(self.get_parameter("progress_timeout").value)
        stalled = (
            self._last_progress_time is not None
            and (now - self._last_progress_time).nanoseconds >= timeout * 1e9
        )
        if (stalled
                and not self._recovery_requested
                and not self._global_recovery_requested
                and self._goal_handle is not None):
            world = self._robot_world()
            truly_stuck = False
            if world is not None:
                truly_stuck = self._update_stuck_state(world[0], world[1], now)
            self._recovery_requested = truly_stuck
            self._global_recovery_requested = not truly_stuck
            self._force_least_explored = not truly_stuck
            if truly_stuck:
                stuck_timeout = float(self.get_parameter("stuck_timeout").value)
                stuck_radius = float(self.get_parameter("stuck_radius").value)
                self.get_logger().warn(
                    f"Robot remained within {stuck_radius:.1f} m for "
                    f"{stuck_timeout:.0f} s; canceling route for backup recovery."
                )
            else:
                self.get_logger().warn(
                    f"No {progress:.2f} m route progress for {timeout:.0f} s; "
                    "canceling for least-explored global replanning."
                )
            self._goal_handle.cancel_goal_async()
        discovery_timeout = float(
            self.get_parameter("discovery_progress_timeout").value
        )
        discovery_gain = self._mapped_free_area - self._discovery_start_area
        minimum_gain = float(
            self.get_parameter("minimum_discovery_area_gain").value
        )
        if discovery_gain >= minimum_gain:
            self._discovery_start_area = self._mapped_free_area
            self._discovery_start_time = now
        elif (
            self._discovery_start_time is not None
            and (now - self._discovery_start_time).nanoseconds
            >= discovery_timeout * 1e9
            and not self._recovery_requested
            and not self._global_recovery_requested
            and not self._unavoidable_transit
            and self._goal_handle is not None
        ):
            self._global_recovery_requested = True
            self._force_least_explored = True
            self.get_logger().warn(
                f"Map gained only {discovery_gain:.1f} m^2 in "
                f"{discovery_timeout:.0f} s; canceling for a more productive "
                "frontier route."
            )
            self._goal_handle.cancel_goal_async()
        self.get_logger().info(
            f"Exploring: {distance:.1f} m remaining.",
            throttle_duration_sec=5.0,
        )

        replan_poses = int(self.get_parameter("rolling_replan_poses").value)
        replan_distance = float(
            self.get_parameter("rolling_replan_distance").value
        )
        extension_period = float(
            self.get_parameter("route_extension_period").value
        )
        navigation_seconds = (
            feedback.navigation_time.sec
            + feedback.navigation_time.nanosec / 1e9
        )
        if (replan_poses > 0
                and not self._rolling_replan_started
                and navigation_seconds >= extension_period
                and feedback.number_of_poses_remaining <= replan_poses
                and distance <= replan_distance):
            self._rolling_replan_started = True
            # The current tail has already been handed to Nav2. Excluding its
            # target forces the replacement route to extend farther ahead.
            self._blacklist.append(self._target)
            self._blacklist = self._blacklist[-20:]
            self.get_logger().info(
                f"Extending the Nav2 route beyond its final "
                f"{feedback.number_of_poses_remaining} poses after "
                f"{navigation_seconds:.1f} s of stable execution."
            )
            self._plan(preempting=True)

    def _goal_response(self, future, generation):
        if generation != self._route_generation:
            return
        handle = future.result()
        if handle is None or not handle.accepted:
            if self._returning_home:
                self.get_logger().warn(
                    "Nav2 rejected return-to-origin route; exploration will retry."
                )
                self._returning_home = False
                self._return_home_pub.publish(Bool(data=False))
                self._empty_cycles = 0
                self._busy = False
                return
            self.get_logger().warn("Nav2 rejected route; blacklisting target.")
            self._blacklist.append(self._target)
            self._busy = False
            return
        self._goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(
            lambda completed, route=generation:
                self._goal_result(completed, route)
        )

    def _goal_result(self, future, generation):
        if generation != self._route_generation:
            return
        result = future.result()
        self._goal_handle = None
        if self._returning_home:
            if result is not None and result.status == 4:
                self.get_logger().info(
                    "Origin reached. Mapping mission complete; stopping behaviors."
                )
                self._timer.cancel()
                self.create_timer(0.5, self._shutdown_completed_mission)
            else:
                self.get_logger().warn(
                    "Return-to-origin route failed; exploration will retry."
                )
                self._returning_home = False
                self._return_home_pub.publish(Bool(data=False))
                self._empty_cycles = 0
                self._busy = False
            return
        if self._recovery_requested:
            self._start_backup()
            return
        if self._global_recovery_requested:
            self.get_logger().info(
                "Canceled stalled route; preparing least-explored global path."
            )
            self._blacklist.append(self._target)
            self._blacklist = self._blacklist[-20:]
            self._global_recovery_requested = False
            self._busy = False
            return
        if result is None or result.status != 4:
            self._blacklist.append(self._target)
            self._blacklist = self._blacklist[-20:]
            world = self._robot_world()
            truly_stuck = (
                world is not None
                and self._update_stuck_state(world[0], world[1])
            )
            if truly_stuck:
                stuck_timeout = float(self.get_parameter("stuck_timeout").value)
                stuck_radius = float(self.get_parameter("stuck_radius").value)
                self.get_logger().warn(
                    f"Route failed after the robot remained within "
                    f"{stuck_radius:.1f} m for {stuck_timeout:.0f} s; "
                    "starting backup recovery."
                )
                self._recovery_requested = True
                self._start_backup()
                return
            self.get_logger().warn(
                "Route failed; requesting a least-explored global recovery path."
            )
            self._force_least_explored = True
        else:
            self.get_logger().info("Frontier reached; updating exploration plan.")
            # Do not repeatedly submit a goal that remains on the frontier
            # mask while SLAM catches up.  Treat a reached target as explored
            # and require the next segment to advance at least 1.5 m from it.
            self._blacklist.append(self._target)
            self._blacklist = self._blacklist[-20:]
        self._busy = False

    def _shutdown_completed_mission(self):
        if rclpy.ok():
            rclpy.shutdown()

    def _start_backup(self):
        if not self._backup_action.server_is_ready():
            self.get_logger().error(
                "Nav2 backup action is unavailable; recovery aborted."
            )
            self._recovery_requested = False
            self._busy = False
            return
        distance = float(
            self.get_parameter("recovery_backup_distance").value
        )
        request = BackUp.Goal()
        # Nav2 BackUp defines forward as positive; a negative x target backs
        # up while its behavior server checks the complete chassis footprint.
        request.target.x = -distance
        request.speed = float(
            self.get_parameter("recovery_backup_speed").value
        )
        request.time_allowance.sec = max(20, int(distance / request.speed) + 10)
        self.get_logger().warn(
            f"Backing up {distance:.2f} m (two chassis lengths) for recovery."
        )
        future = self._backup_action.send_goal_async(request)
        future.add_done_callback(self._backup_response)

    def _backup_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Nav2 rejected backup recovery.")
            self._recovery_requested = False
            self._busy = False
            return
        result = handle.get_result_async()
        result.add_done_callback(self._backup_result)

    def _backup_result(self, future):
        result = future.result()
        if result is not None and result.status == 4:
            self.get_logger().info("Backup recovery completed; replanning.")
        else:
            self.get_logger().warn(
                "Backup stopped before two lengths, likely due to an obstacle."
            )
        self._recovery_requested = False
        self._stuck_anchor = None
        self._stuck_anchor_time = None
        self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = WanderingMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node._goal_handle is not None and rclpy.ok():
            node._goal_handle.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
