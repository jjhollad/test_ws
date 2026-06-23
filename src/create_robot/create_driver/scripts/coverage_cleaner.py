#!/usr/bin/env python3

import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    return qz, qw


@dataclass
class CoverageCell:
    rows: List[Tuple[int, int, int]] = field(default_factory=list)

    def append(self, row: int, start_col: int, end_col: int):
        self.rows.append((row, start_col, end_col))

    def centroid(self, grid: OccupancyGrid) -> Tuple[float, float]:
        if not self.rows:
            return (0.0, 0.0)
        total_x = 0.0
        total_y = 0.0
        for row, start_col, end_col in self.rows:
            x, y = self._interval_center(grid, row, start_col, end_col)
            total_x += x
            total_y += y
        count = len(self.rows)
        return (total_x / count, total_y / count)

    def bounds(self) -> Tuple[int, int, int, int]:
        if not self.rows:
            return (0, 0, 0, 0)
        min_row = min(row for row, _, _ in self.rows)
        max_row = max(row for row, _, _ in self.rows)
        min_col = min(start_col for _, start_col, _ in self.rows)
        max_col = max(end_col for _, _, end_col in self.rows)
        return (min_col, max_col, min_row, max_row)

    def is_wider_than_tall(self) -> bool:
        min_col, max_col, min_row, max_row = self.bounds()
        return (max_col - min_col) >= (max_row - min_row)

    def contains(self, col: int, row: int) -> bool:
        for cell_row, start_col, end_col in self.rows:
            if row == cell_row and start_col <= col <= end_col:
                return True
        return False

    @staticmethod
    def _interval_center(
        grid: OccupancyGrid,
        row: int,
        start_col: int,
        end_col: int,
    ) -> Tuple[float, float]:
        col = (start_col + end_col) * 0.5
        x = grid.info.origin.position.x + (col + 0.5) * grid.info.resolution
        y = grid.info.origin.position.y + (row + 0.5) * grid.info.resolution
        return (x, y)


class CoverageCleaner(Node):
    def __init__(self):
        super().__init__('coverage_cleaner')

        self.declare_parameter('auto_start', True)
        self.declare_parameter('preview_only', False)
        self.declare_parameter('coverage_algorithm', 'boustrophedon')
        self.declare_parameter('publish_continuous_path', True)
        self.declare_parameter('preview_republish_period', 2.0)
        self.declare_parameter('startup_delay', 8.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('line_spacing', 0.75)
        self.declare_parameter('waypoint_spacing', 0.75)
        self.declare_parameter('wall_clearance', 0.30)
        self.declare_parameter('edge_passes', 1)
        self.declare_parameter('edge_stepover', 0.80)
        self.declare_parameter('edge_band', 0.10)
        self.declare_parameter('interior_sweep_edge_clearance', 0.65)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('restrict_to_start_region', True)
        self.declare_parameter('min_run_length', 0.80)
        self.declare_parameter('min_cell_height', 0.80)
        self.declare_parameter('align_sweeps_to_cell_axis', True)
        self.declare_parameter('snap_sweeps_to_wall_axes', True)
        self.declare_parameter('wall_axis_override_deg', 999.0)
        self.declare_parameter('max_waypoints', 250)
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('initial_pose_repeats', 8)
        self.declare_parameter('preview_line_width', 0.08)
        self.declare_parameter('preview_dot_size', 0.08)
        self.declare_parameter('preview_show_dots', True)
        self.declare_parameter('preview_show_arrows', True)
        self.declare_parameter('preview_arrow_stride', 10)
        self.declare_parameter('preview_label_stride', 25)
        self.declare_parameter('preview_text_height', 0.35)

        self._map: Optional[OccupancyGrid] = None
        self._preview_waypoints: List[PoseStamped] = []
        self._preview_segments: List[List[PoseStamped]] = []
        self._wall_axis_angle: Optional[float] = None
        self._sent_goal = False
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10,
        )
        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self._path_publisher = self.create_publisher(Path, 'coverage_path', path_qos)
        self._marker_publisher = self.create_publisher(Marker, 'coverage_marker', path_qos)
        self._marker_array_publisher = self.create_publisher(
            MarkerArray,
            'coverage_marker_array',
            path_qos,
        )
        self._pose_array_publisher = self.create_publisher(PoseArray, 'coverage_poses', path_qos)
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            map_qos,
        )
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        if self.get_parameter('publish_initial_pose').value:
            repeats = int(self.get_parameter('initial_pose_repeats').value)
            self._initial_pose_repeats_left = max(0, repeats)
            self._initial_pose_timer = self.create_timer(0.5, self._publish_initial_pose)
        else:
            self._initial_pose_repeats_left = 0
            self._initial_pose_timer = None

        startup_delay = float(self.get_parameter('startup_delay').value)
        self._start_timer = self.create_timer(startup_delay, self._maybe_start_cleaning)

    def _map_callback(self, msg: OccupancyGrid):
        self._map = msg

    def _publish_initial_pose(self):
        if self._initial_pose_repeats_left <= 0:
            self._initial_pose_timer.cancel()
            return

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.get_parameter('map_frame').value
        pose.pose.pose.position.x = float(self.get_parameter('start_x').value)
        pose.pose.pose.position.y = float(self.get_parameter('start_y').value)
        qz, qw = yaw_to_quaternion(float(self.get_parameter('start_yaw').value))
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.0685

        self._initial_pose_publisher.publish(pose)
        self._initial_pose_repeats_left -= 1

    def _maybe_start_cleaning(self):
        self._start_timer.cancel()

        if not bool(self.get_parameter('auto_start').value):
            self.get_logger().info('Coverage cleaner is loaded; auto_start is false.')
            return

        if self._map is None:
            self.get_logger().warn('Waiting for /map before generating cleaning waypoints.')
            self._start_timer = self.create_timer(1.0, self._maybe_start_cleaning)
            return

        waypoints = self._generate_waypoints(self._map)
        if not waypoints:
            self.get_logger().error('No cleaning waypoints were generated from the map.')
            return

        self._publish_path_preview(waypoints)

        if bool(self.get_parameter('preview_only').value):
            self.get_logger().info(
                f'Published {len(waypoints)} coverage preview poses on /coverage_path.'
            )
            republish_period = float(self.get_parameter('preview_republish_period').value)
            if republish_period > 0.0:
                self._preview_waypoints = waypoints
                self.create_timer(republish_period, self._republish_preview)
            return

        max_waypoints = int(self.get_parameter('max_waypoints').value)
        if max_waypoints > 0 and len(waypoints) > max_waypoints:
            self.get_logger().warn(
                f'Generated {len(waypoints)} waypoints; limiting to {max_waypoints}.'
            )
            waypoints = waypoints[:max_waypoints]

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 follow_waypoints action server is not available.')
            return

        goal = FollowWaypoints.Goal()
        goal.poses = waypoints
        self.get_logger().info(f'Sending {len(waypoints)} cleaning waypoints to Nav2.')
        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _republish_preview(self):
        if self._preview_waypoints:
            self._publish_path_preview(self._preview_waypoints)

    def _publish_path_preview(self, waypoints: List[PoseStamped]):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.get_parameter('map_frame').value
        path.poses = waypoints
        if bool(self.get_parameter('publish_continuous_path').value):
            self._path_publisher.publish(path)

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL

        markers = [delete_marker, self._line_marker(path.header, self._preview_segments)]
        if bool(self.get_parameter('preview_show_dots').value):
            markers.append(self._dot_marker(path.header, waypoints))
        if bool(self.get_parameter('preview_show_arrows').value):
            markers.extend(self._direction_markers(path.header, self._preview_segments))
        markers.extend(self._step_label_markers(path.header, waypoints))

        self._marker_publisher.publish(markers[1])
        self._marker_array_publisher.publish(MarkerArray(markers=markers))

        poses = PoseArray()
        poses.header = path.header
        poses.poses = [waypoint.pose for waypoint in waypoints]
        self._pose_array_publisher.publish(poses)

    def _line_marker(self, header, segments: List[List[PoseStamped]]) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = 'coverage_cleaner'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = float(self.get_parameter('preview_line_width').value)
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        for segment in segments:
            for index in range(len(segment) - 1):
                start = segment[index].pose.position
                end = segment[index + 1].pose.position
                marker.points.append(Point(x=start.x, y=start.y, z=0.35))
                marker.points.append(Point(x=end.x, y=end.y, z=0.35))
        return marker

    def _dot_marker(self, header, waypoints: List[PoseStamped]) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = 'coverage_cleaner'
        marker.id = 1
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        dot_size = float(self.get_parameter('preview_dot_size').value)
        marker.scale.x = dot_size
        marker.scale.y = dot_size
        marker.scale.z = dot_size
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.points = [
            Point(x=pose.pose.position.x, y=pose.pose.position.y, z=0.45)
            for pose in waypoints
        ]
        return marker

    def _direction_markers(self, header, segments: List[List[PoseStamped]]) -> List[Marker]:
        stride = max(1, int(self.get_parameter('preview_arrow_stride').value))
        line_width = float(self.get_parameter('preview_line_width').value)
        markers = []
        step = 0

        for segment in segments:
            for index in range(len(segment) - 1):
                if step % stride == 0:
                    start = segment[index].pose.position
                    end = segment[index + 1].pose.position
                    marker = Marker()
                    marker.header = header
                    marker.ns = 'coverage_direction'
                    marker.id = 10000 + step
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD
                    marker.pose.orientation.w = 1.0
                    marker.points = [
                        Point(x=start.x, y=start.y, z=0.75),
                        Point(x=end.x, y=end.y, z=0.75),
                    ]
                    marker.scale.x = max(0.04, line_width * 1.5)
                    marker.scale.y = max(0.14, line_width * 4.0)
                    marker.scale.z = max(0.22, line_width * 6.0)
                    marker.color.r = 0.0
                    marker.color.g = 0.85
                    marker.color.b = 1.0
                    marker.color.a = 1.0
                    markers.append(marker)
                step += 1

        return markers

    def _step_label_markers(self, header, waypoints: List[PoseStamped]) -> List[Marker]:
        stride = int(self.get_parameter('preview_label_stride').value)
        if stride <= 0:
            return []

        text_height = float(self.get_parameter('preview_text_height').value)
        markers = []
        for index, waypoint in enumerate(waypoints):
            step_number = index + 1
            if index != 0 and step_number % stride != 0:
                continue

            marker = Marker()
            marker.header = header
            marker.ns = 'coverage_steps'
            marker.id = 20000 + step_number
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.pose.position.x
            marker.pose.position.y = waypoint.pose.position.y
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0
            marker.scale.z = text_height
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = str(step_number)
            markers.append(marker)

        return markers

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Cleaning waypoint goal was rejected.')
            return

        self.get_logger().info('Cleaning waypoint goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint
        self.get_logger().debug(f'Current cleaning waypoint: {current}')

    def _result_callback(self, future):
        result = future.result().result
        missed = list(result.missed_waypoints)
        if missed:
            self.get_logger().warn(f'Cleaning finished with missed waypoints: {missed}')
        else:
            self.get_logger().info('Cleaning waypoint route completed.')

    def _generate_waypoints(self, grid: OccupancyGrid) -> List[PoseStamped]:
        resolution = grid.info.resolution
        height = grid.info.height

        line_step = max(1, round(float(self.get_parameter('line_spacing').value) / resolution))
        point_step = max(1, round(float(self.get_parameter('waypoint_spacing').value) / resolution))
        min_run_cells = max(1, round(float(self.get_parameter('min_run_length').value) / resolution))
        allowed_cells = self._allowed_cells(grid)

        algorithm = str(self.get_parameter('coverage_algorithm').value).lower()
        if algorithm in ('center_loop', 'middle_loop'):
            segments = self._generate_center_loop_segments(grid, allowed_cells, point_step)
        else:
            sweep_allowed_cells = self._interior_sweep_cells(grid, allowed_cells)

            if algorithm in ('boustrophedon', 'bcd'):
                segments = self._generate_boustrophedon_segments(
                    grid,
                    sweep_allowed_cells,
                    line_step,
                    point_step,
                    min_run_cells,
                )
            else:
                segments = self._generate_lawnmower_segments(
                    grid,
                    sweep_allowed_cells,
                    line_step,
                    point_step,
                    min_run_cells,
                    height,
                )

        if algorithm not in ('center_loop', 'middle_loop'):
            edge_segments = self._generate_edge_segments(grid, allowed_cells, point_step)
            segments = edge_segments + segments

        self._preview_segments = self._orient_segments(segments)
        return [waypoint for segment in self._preview_segments for waypoint in segment]

    def _interior_sweep_cells(
        self,
        grid: OccupancyGrid,
        allowed_cells: Optional[List[bool]],
    ) -> Optional[List[bool]]:
        edge_passes = int(self.get_parameter('edge_passes').value)
        clearance = float(self.get_parameter('interior_sweep_edge_clearance').value)
        if edge_passes <= 0 or clearance <= 0.0:
            return allowed_cells

        passable = self._passable_mask(grid, allowed_cells)
        edge_distances = self._edge_distance_field(grid, passable)
        clearance_cells = max(1, round(clearance / grid.info.resolution))
        sweep_allowed = [False] * len(passable)

        for index, is_passable in enumerate(passable):
            sweep_allowed[index] = is_passable and edge_distances[index] >= clearance_cells

        if not any(sweep_allowed):
            self.get_logger().warn(
                'Interior sweep edge clearance removed all cells; using full reachable area.'
            )
            return allowed_cells

        return sweep_allowed

    def _generate_edge_segments(
        self,
        grid: OccupancyGrid,
        allowed_cells: Optional[List[bool]],
        point_step: int,
    ) -> List[List[PoseStamped]]:
        edge_passes = int(self.get_parameter('edge_passes').value)
        if edge_passes <= 0:
            return []

        passable = self._passable_mask(grid, allowed_cells)
        edge_distances = self._edge_distance_field(grid, passable)
        step_cells = max(
            1,
            round(float(self.get_parameter('edge_stepover').value) / grid.info.resolution),
        )
        band_cells = max(
            1,
            round(float(self.get_parameter('edge_band').value) / grid.info.resolution),
        )

        segments: List[List[PoseStamped]] = []
        current = (
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value),
        )

        for edge_pass in range(edge_passes):
            target_distance = edge_pass * step_cells
            ring_cells = [
                index
                for index, distance in enumerate(edge_distances)
                if distance >= 0 and abs(distance - target_distance) <= band_cells
            ]
            ring_segments = self._ring_segments_from_cells(
                grid,
                ring_cells,
                current,
                point_step,
            )
            if ring_segments:
                current = (
                    ring_segments[-1][-1].pose.position.x,
                    ring_segments[-1][-1].pose.position.y,
                )
                segments.extend(ring_segments)

        self.get_logger().info(
            f'Edge preview generated {len(segments)} segments across {edge_passes} passes.'
        )
        return segments

    def _generate_center_loop_segments(
        self,
        grid: OccupancyGrid,
        allowed_cells: Optional[List[bool]],
        point_step: int,
    ) -> List[List[PoseStamped]]:
        passable = self._passable_mask(grid, allowed_cells)
        center_cells = self._hallway_centerline_cells(grid, passable)
        if not center_cells:
            self.get_logger().warn(
                'Center loop found no hallway centerline cells; try lowering min_run_length.'
            )
            return []

        loop_cells = self._largest_connected_component(grid, center_cells)
        cycle_cells = self._prune_dead_end_cells(grid, loop_cells)
        if len(cycle_cells) >= max(8, len(loop_cells) // 4):
            loop_cells = self._largest_connected_component(grid, cycle_cells)

        segment = self._trace_loop_segment(grid, loop_cells, point_step)
        segments = [segment] if len(segment) >= 2 else []

        self.get_logger().info(
            f'Center loop preview generated {len(segment)} poses in one closed path.'
        )
        return segments

    def _hallway_centerline_cells(
        self,
        grid: OccupancyGrid,
        passable: List[bool],
    ) -> List[int]:
        width = grid.info.width
        height = grid.info.height
        min_run_cells = max(
            3,
            round(float(self.get_parameter('min_run_length').value) / grid.info.resolution),
        )
        cells = set()

        for row in range(height):
            for start_col, end_col in self._free_runs_in_row_mask(
                passable,
                width,
                row,
                min_run_cells,
            ):
                mid_col = (start_col + end_col) // 2
                cells.add(row * width + mid_col)

        for col in range(width):
            for start_row, end_row in self._free_runs_in_col_mask(
                passable,
                width,
                height,
                col,
                min_run_cells,
            ):
                mid_row = (start_row + end_row) // 2
                cells.add(mid_row * width + col)

        return list(cells)

    def _free_runs_in_row_mask(
        self,
        passable: List[bool],
        width: int,
        row: int,
        min_run_cells: int,
    ) -> List[Tuple[int, int]]:
        runs = []
        start_col = None

        for col in range(width):
            is_passable = passable[row * width + col]
            if is_passable and start_col is None:
                start_col = col
            elif not is_passable and start_col is not None:
                end_col = col - 1
                if end_col - start_col + 1 >= min_run_cells:
                    runs.append((start_col, end_col))
                start_col = None

        if start_col is not None:
            end_col = width - 1
            if end_col - start_col + 1 >= min_run_cells:
                runs.append((start_col, end_col))

        return runs

    def _free_runs_in_col_mask(
        self,
        passable: List[bool],
        width: int,
        height: int,
        col: int,
        min_run_cells: int,
    ) -> List[Tuple[int, int]]:
        runs = []
        start_row = None

        for row in range(height):
            is_passable = passable[row * width + col]
            if is_passable and start_row is None:
                start_row = row
            elif not is_passable and start_row is not None:
                end_row = row - 1
                if end_row - start_row + 1 >= min_run_cells:
                    runs.append((start_row, end_row))
                start_row = None

        if start_row is not None:
            end_row = height - 1
            if end_row - start_row + 1 >= min_run_cells:
                runs.append((start_row, end_row))

        return runs

    def _prune_dead_end_cells(
        self,
        grid: OccupancyGrid,
        cells: List[int],
    ) -> List[int]:
        width = grid.info.width
        height = grid.info.height
        remaining = set(cells)
        queue = deque(
            index for index in remaining if self._cell_graph_degree(width, height, remaining, index) <= 1
        )

        while queue:
            index = queue.popleft()
            if index not in remaining:
                continue
            if self._cell_graph_degree(width, height, remaining, index) > 1:
                continue

            remaining.remove(index)
            col = index % width
            row = index // width
            for next_col, next_row in self._neighbors8(col, row):
                if next_col < 0 or next_row < 0 or next_col >= width or next_row >= height:
                    continue
                next_index = next_row * width + next_col
                if next_index in remaining and self._cell_graph_degree(width, height, remaining, next_index) <= 1:
                    queue.append(next_index)

        return list(remaining)

    def _cell_graph_degree(
        self,
        width: int,
        height: int,
        cells: set,
        index: int,
    ) -> int:
        col = index % width
        row = index // width
        degree = 0
        for next_col, next_row in self._neighbors8(col, row):
            if next_col < 0 or next_row < 0 or next_col >= width or next_row >= height:
                continue
            if next_row * width + next_col in cells:
                degree += 1
        return degree

    def _trace_loop_segment(
        self,
        grid: OccupancyGrid,
        cells: List[int],
        point_step: int,
    ) -> List[PoseStamped]:
        if not cells:
            return []

        width = grid.info.width
        height = grid.info.height
        cell_set = set(cells)
        start = (
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value),
        )
        start_cell = min(
            cell_set,
            key=lambda index: self._distance_squared(
                start,
                self._cell_to_world(grid, index % width, index // width),
            ),
        )

        ordered_cells = [start_cell]
        visited = {start_cell}
        previous = None
        current = start_cell
        max_steps = max(1, len(cell_set) + 1)

        for _ in range(max_steps):
            candidates = [
                index
                for index in self._neighbor_cell_indices(width, height, current)
                if index in cell_set and index != previous
            ]
            if not candidates:
                break

            unvisited = [index for index in candidates if index not in visited]
            if not unvisited and start_cell in candidates and len(ordered_cells) > 3:
                ordered_cells.append(start_cell)
                break
            if not unvisited:
                break

            if previous is None:
                next_cell = self._best_initial_loop_neighbor(width, start_cell, unvisited)
            else:
                next_cell = self._best_continuation_cell(width, previous, current, unvisited)

            ordered_cells.append(next_cell)
            visited.add(next_cell)
            previous = current
            current = next_cell

        if ordered_cells[-1] != start_cell:
            neighbors = self._neighbor_cell_indices(width, height, ordered_cells[-1])
            if start_cell in neighbors:
                ordered_cells.append(start_cell)
            else:
                self.get_logger().warn(
                    'Center loop could not close cleanly; publishing the longest traced path.'
                )

        spacing = max(grid.info.resolution, point_step * grid.info.resolution)
        segment = []
        last_point: Optional[Tuple[float, float]] = None
        for cell in ordered_cells:
            point = self._cell_to_world(grid, cell % width, cell // width)
            if last_point is None or math.hypot(point[0] - last_point[0], point[1] - last_point[1]) >= spacing:
                segment.append(self._pose(point[0], point[1], 0.0))
                last_point = point

        if segment:
            first = segment[0].pose.position
            last = segment[-1].pose.position
            if math.hypot(first.x - last.x, first.y - last.y) >= grid.info.resolution:
                segment.append(self._pose(first.x, first.y, 0.0))

        return segment

    def _neighbor_cell_indices(self, width: int, height: int, index: int) -> List[int]:
        col = index % width
        row = index // width
        neighbors = []
        for next_col, next_row in self._neighbors8(col, row):
            if next_col < 0 or next_row < 0 or next_col >= width or next_row >= height:
                continue
            neighbors.append(next_row * width + next_col)
        return neighbors

    def _best_initial_loop_neighbor(
        self,
        width: int,
        current: int,
        candidates: List[int],
    ) -> int:
        current_col = current % width
        current_row = current // width
        return min(
            candidates,
            key=lambda index: math.atan2((index // width) - current_row, (index % width) - current_col),
        )

    def _best_continuation_cell(
        self,
        width: int,
        previous: int,
        current: int,
        candidates: List[int],
    ) -> int:
        prev_col = previous % width
        prev_row = previous // width
        current_col = current % width
        current_row = current // width
        incoming_x = current_col - prev_col
        incoming_y = current_row - prev_row

        def turn_cost(index: int) -> float:
            next_col = index % width
            next_row = index // width
            outgoing_x = next_col - current_col
            outgoing_y = next_row - current_row
            cross = incoming_x * outgoing_y - incoming_y * outgoing_x
            dot = incoming_x * outgoing_x + incoming_y * outgoing_y
            angle = abs(math.atan2(cross, dot))
            distance_penalty = 0.01 * (outgoing_x * outgoing_x + outgoing_y * outgoing_y)
            return angle + distance_penalty

        return min(candidates, key=turn_cost)

    def _largest_connected_component(
        self,
        grid: OccupancyGrid,
        cells: List[int],
    ) -> List[int]:
        width = grid.info.width
        height = grid.info.height
        remaining = set(cells)
        largest: List[int] = []

        while remaining:
            start = remaining.pop()
            component = [start]
            queue = deque([start])
            while queue:
                index = queue.popleft()
                col = index % width
                row = index // width
                for next_col, next_row in self._neighbors8(col, row):
                    if next_col < 0 or next_row < 0 or next_col >= width or next_row >= height:
                        continue
                    next_index = next_row * width + next_col
                    if next_index not in remaining:
                        continue
                    remaining.remove(next_index)
                    component.append(next_index)
                    queue.append(next_index)

            if len(component) > len(largest):
                largest = component

        return largest

    def _passable_mask(
        self,
        grid: OccupancyGrid,
        allowed_cells: Optional[List[bool]],
    ) -> List[bool]:
        if allowed_cells is not None:
            return allowed_cells

        passable = [False] * (grid.info.width * grid.info.height)
        for row in range(grid.info.height):
            for col in range(grid.info.width):
                passable[row * grid.info.width + col] = self._is_clear_cell(grid, col, row)
        return passable

    def _edge_distance_field(self, grid: OccupancyGrid, passable: List[bool]) -> List[int]:
        width = grid.info.width
        height = grid.info.height
        distances = [-1] * (width * height)
        queue = deque()

        for row in range(height):
            for col in range(width):
                index = row * width + col
                if not passable[index]:
                    continue
                if self._is_passable_edge(width, height, passable, col, row):
                    distances[index] = 0
                    queue.append((col, row))

        while queue:
            col, row = queue.popleft()
            distance = distances[row * width + col]
            for next_col, next_row in self._neighbors4(col, row):
                if next_col < 0 or next_row < 0 or next_col >= width or next_row >= height:
                    continue
                next_index = next_row * width + next_col
                if not passable[next_index] or distances[next_index] >= 0:
                    continue
                distances[next_index] = distance + 1
                queue.append((next_col, next_row))

        return distances

    def _is_passable_edge(
        self,
        width: int,
        height: int,
        passable: List[bool],
        col: int,
        row: int,
    ) -> bool:
        for next_col, next_row in self._neighbors4(col, row):
            if next_col < 0 or next_row < 0 or next_col >= width or next_row >= height:
                return True
            if not passable[next_row * width + next_col]:
                return True
        return False

    def _ring_segments_from_cells(
        self,
        grid: OccupancyGrid,
        ring_cells: List[int],
        current: Tuple[float, float],
        point_step: int,
    ) -> List[List[PoseStamped]]:
        width = grid.info.width
        height = grid.info.height
        ring_set = set(ring_cells)
        components: List[List[int]] = []

        while ring_set:
            start = ring_set.pop()
            component = [start]
            queue = deque([start])
            while queue:
                index = queue.popleft()
                col = index % width
                row = index // width
                for next_col, next_row in self._neighbors8(col, row):
                    if next_col < 0 or next_row < 0 or next_col >= width or next_row >= height:
                        continue
                    next_index = next_row * width + next_col
                    if next_index not in ring_set:
                        continue
                    ring_set.remove(next_index)
                    component.append(next_index)
                    queue.append(next_index)
            components.append(component)

        ordered_components = self._order_components(grid, components, current)
        segments: List[List[PoseStamped]] = []
        min_points = max(3, round(float(self.get_parameter('min_run_length').value) / grid.info.resolution))

        for component in ordered_components:
            if len(component) < min_points:
                continue
            component_segments = self._ordered_component_segments(grid, component, point_step)
            segments.extend(component_segments)

        return segments

    def _order_components(
        self,
        grid: OccupancyGrid,
        components: List[List[int]],
        current: Tuple[float, float],
    ) -> List[List[int]]:
        remaining = components[:]
        ordered = []
        cursor = current

        while remaining:
            best_index = min(
                range(len(remaining)),
                key=lambda index: self._distance_squared(cursor, self._component_centroid(grid, remaining[index])),
            )
            component = remaining.pop(best_index)
            ordered.append(component)
            cursor = self._component_centroid(grid, component)

        return ordered

    def _component_centroid(self, grid: OccupancyGrid, component: List[int]) -> Tuple[float, float]:
        total_x = 0.0
        total_y = 0.0
        for index in component:
            x, y = self._cell_to_world(grid, index % grid.info.width, index // grid.info.width)
            total_x += x
            total_y += y
        count = max(1, len(component))
        return (total_x / count, total_y / count)

    def _ordered_component_segments(
        self,
        grid: OccupancyGrid,
        component: List[int],
        point_step: int,
    ) -> List[List[PoseStamped]]:
        width = grid.info.width
        height = grid.info.height
        unvisited = set(component)
        segments: List[List[PoseStamped]] = []

        while unvisited:
            start = min(unvisited)
            unvisited.remove(start)
            ordered = [start]
            current = start

            while unvisited:
                col = current % width
                row = current // width
                neighbors = []
                for next_col, next_row in self._neighbors8(col, row):
                    if next_col < 0 or next_row < 0 or next_col >= width or next_row >= height:
                        continue
                    next_index = next_row * width + next_col
                    if next_index in unvisited:
                        neighbors.append(next_index)

                if not neighbors:
                    break

                next_index = min(
                    neighbors,
                    key=lambda index: self._cell_distance_squared(width, current, index),
                )
                unvisited.remove(next_index)
                ordered.append(next_index)
                current = next_index

            segment = self._cells_to_downsampled_segment(grid, ordered, point_step)
            if len(segment) >= 2:
                segments.append(segment)

        return segments

    def _cells_to_downsampled_segment(
        self,
        grid: OccupancyGrid,
        cells: List[int],
        point_step: int,
    ) -> List[PoseStamped]:
        segment = []
        last_col = None
        last_row = None

        for index in cells:
            col = index % grid.info.width
            row = index // grid.info.width
            if last_col is None:
                should_add = True
            else:
                d_col = col - last_col
                d_row = row - last_row
                should_add = d_col * d_col + d_row * d_row >= point_step * point_step

            if should_add:
                x, y = self._cell_to_world(grid, col, row)
                segment.append(self._pose(x, y, 0.0))
                last_col = col
                last_row = row

        if cells:
            final_col = cells[-1] % grid.info.width
            final_row = cells[-1] // grid.info.width
            if last_col != final_col or last_row != final_row:
                x, y = self._cell_to_world(grid, final_col, final_row)
                segment.append(self._pose(x, y, 0.0))

        return segment

    @staticmethod
    def _neighbors4(col: int, row: int) -> List[Tuple[int, int]]:
        return [
            (col + 1, row),
            (col - 1, row),
            (col, row + 1),
            (col, row - 1),
        ]

    @staticmethod
    def _neighbors8(col: int, row: int) -> List[Tuple[int, int]]:
        return [
            (col + 1, row),
            (col - 1, row),
            (col, row + 1),
            (col, row - 1),
            (col + 1, row + 1),
            (col + 1, row - 1),
            (col - 1, row + 1),
            (col - 1, row - 1),
        ]

    @staticmethod
    def _cell_distance_squared(width: int, first: int, second: int) -> int:
        first_col = first % width
        first_row = first // width
        second_col = second % width
        second_row = second // width
        d_col = first_col - second_col
        d_row = first_row - second_row
        return d_col * d_col + d_row * d_row

    def _generate_lawnmower_segments(
        self,
        grid: OccupancyGrid,
        allowed_cells: Optional[List[bool]],
        line_step: int,
        point_step: int,
        min_run_cells: int,
        height: int,
    ) -> List[List[PoseStamped]]:
        rows = range(0, height, line_step)
        segments: List[List[PoseStamped]] = []
        reverse = False

        for row in rows:
            runs = self._free_runs_for_row(grid, row, min_run_cells, allowed_cells)
            if reverse:
                runs = list(reversed(runs))

            for start_col, end_col in runs:
                cols = range(start_col, end_col + 1, point_step)
                if reverse:
                    cols = range(end_col, start_col - 1, -point_step)
                segment = []
                for col in cols:
                    x, y = self._cell_to_world(grid, col, row)
                    segment.append(self._pose(x, y, 0.0))
                if len(segment) >= 2:
                    segments.append(segment)

            reverse = not reverse

        return segments

    def _generate_boustrophedon_segments(
        self,
        grid: OccupancyGrid,
        allowed_cells: Optional[List[bool]],
        line_step: int,
        point_step: int,
        min_run_cells: int,
    ) -> List[List[PoseStamped]]:
        cells = self._decompose_boustrophedon_cells(
            grid,
            allowed_cells,
            line_step,
            min_run_cells,
        )
        ordered_cells = self._order_cells_from_start(grid, cells)
        segments: List[List[PoseStamped]] = []

        reverse_cell = False
        for cell in ordered_cells:
            if bool(self.get_parameter('align_sweeps_to_cell_axis').value):
                segments.extend(
                    self._oriented_cell_sweep_segments(
                        grid,
                        cell,
                        line_step,
                        point_step,
                        reverse_cell,
                    )
                )
            elif cell.is_wider_than_tall():
                segments.extend(
                    self._horizontal_cell_sweep_segments(
                        grid,
                        cell,
                        point_step,
                        reverse_cell,
                    )
                )
            else:
                segments.extend(
                    self._vertical_cell_sweep_segments(
                        grid,
                        cell,
                        line_step,
                        point_step,
                        reverse_cell,
                    )
                )
            reverse_cell = not reverse_cell

        self.get_logger().info(
            f'Boustrophedon preview generated {len(ordered_cells)} cells.'
        )
        return segments

    def _oriented_cell_sweep_segments(
        self,
        grid: OccupancyGrid,
        cell: CoverageCell,
        line_step: int,
        point_step: int,
        reverse_cell: bool,
    ) -> List[List[PoseStamped]]:
        samples = self._cell_sample_points(grid, cell, max(1, point_step // 2))
        if len(samples) < 2:
            return []

        center_x = sum(point[0] for point in samples) / len(samples)
        center_y = sum(point[1] for point in samples) / len(samples)
        cov_xx = sum((point[0] - center_x) ** 2 for point in samples)
        cov_yy = sum((point[1] - center_y) ** 2 for point in samples)
        cov_xy = sum((point[0] - center_x) * (point[1] - center_y) for point in samples)

        axis_angle = 0.5 * math.atan2(2.0 * cov_xy, cov_xx - cov_yy)
        if bool(self.get_parameter('snap_sweeps_to_wall_axes').value):
            wall_axis = self._dominant_wall_axis(grid)
            if wall_axis is not None:
                axis_angle = self._wall_aligned_cell_axis(samples, wall_axis)

        axis_x = math.cos(axis_angle)
        axis_y = math.sin(axis_angle)
        cross_x = -axis_y
        cross_y = axis_x

        line_spacing = max(grid.info.resolution, line_step * grid.info.resolution)
        waypoint_spacing = max(grid.info.resolution, point_step * grid.info.resolution)
        min_run_length = float(self.get_parameter('min_run_length').value)

        bins = {}
        for x, y in samples:
            rel_x = x - center_x
            rel_y = y - center_y
            along = rel_x * axis_x + rel_y * axis_y
            across = rel_x * cross_x + rel_y * cross_y
            bin_index = round(across / line_spacing)
            bins.setdefault(bin_index, []).append((along, across))

        ordered_bins = sorted(bins.items(), key=lambda item: item[0], reverse=reverse_cell)
        segments: List[List[PoseStamped]] = []
        reverse_line = reverse_cell
        max_gap = waypoint_spacing * 1.8

        for _, values in ordered_bins:
            values = sorted(values, key=lambda item: item[0], reverse=reverse_line)
            run: List[Tuple[float, float]] = []
            previous_along = None
            for along, across in values:
                if previous_along is not None and abs(along - previous_along) > max_gap:
                    segment = self._oriented_run_to_segment(
                        center_x,
                        center_y,
                        axis_x,
                        axis_y,
                        cross_x,
                        cross_y,
                        run,
                        waypoint_spacing,
                    )
                    if self._segment_length(segment) >= min_run_length:
                        segments.append(segment)
                    run = []
                run.append((along, across))
                previous_along = along

            segment = self._oriented_run_to_segment(
                center_x,
                center_y,
                axis_x,
                axis_y,
                cross_x,
                cross_y,
                run,
                waypoint_spacing,
            )
            if self._segment_length(segment) >= min_run_length:
                segments.append(segment)

            reverse_line = not reverse_line

        return segments

    def _wall_aligned_cell_axis(
        self,
        samples: List[Tuple[float, float]],
        wall_axis: float,
    ) -> float:
        axis0_x = math.cos(wall_axis)
        axis0_y = math.sin(wall_axis)
        axis1_x = -axis0_y
        axis1_y = axis0_x

        projections0 = [x * axis0_x + y * axis0_y for x, y in samples]
        projections1 = [x * axis1_x + y * axis1_y for x, y in samples]
        extent0 = max(projections0) - min(projections0)
        extent1 = max(projections1) - min(projections1)

        if extent0 >= extent1:
            return wall_axis
        return wall_axis + math.pi / 2.0

    def _dominant_wall_axis(self, grid: OccupancyGrid) -> Optional[float]:
        if self._wall_axis_angle is not None:
            return self._wall_axis_angle

        override_deg = float(self.get_parameter('wall_axis_override_deg').value)
        if abs(override_deg) <= 180.0:
            self._wall_axis_angle = math.radians(override_deg)
            self.get_logger().info(
                f'Using wall axis override {override_deg:.1f} degrees.'
            )
            return self._wall_axis_angle

        occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        occupied_cells = []
        for row in range(0, grid.info.height, 2):
            for col in range(0, grid.info.width, 2):
                value = grid.data[row * grid.info.width + col]
                if value < occupied_threshold:
                    continue
                occupied_cells.append((col, row))

        if len(occupied_cells) < 2:
            return None

        occupied_set = set(occupied_cells)
        best_angle = 0.0
        best_score = -1
        radii = (4, 7, 10)

        for degree in range(0, 180):
            angle = math.radians(degree)
            cos_angle = math.cos(angle)
            sin_angle = math.sin(angle)
            score = 0

            for col, row in occupied_cells:
                for radius in radii:
                    d_col = int(round(cos_angle * radius))
                    d_row = int(round(sin_angle * radius))
                    if d_col == 0 and d_row == 0:
                        continue
                    if (col + d_col, row + d_row) in occupied_set:
                        score += 1
                    if (col - d_col, row - d_row) in occupied_set:
                        score += 1

            if score > best_score:
                best_score = score
                best_angle = angle

        self._wall_axis_angle = best_angle
        self.get_logger().info(
            f'Detected wall tangent axis {math.degrees(self._wall_axis_angle):.1f} degrees.'
        )
        return self._wall_axis_angle

    def _nearest_equivalent_axis(self, axis_angle: float, wall_axis: float) -> float:
        candidates = [
            wall_axis,
            wall_axis + math.pi / 2.0,
            wall_axis + math.pi,
            wall_axis - math.pi / 2.0,
        ]
        return min(
            candidates,
            key=lambda candidate: abs(math.atan2(math.sin(axis_angle - candidate), math.cos(axis_angle - candidate))),
        )

    def _cell_sample_points(
        self,
        grid: OccupancyGrid,
        cell: CoverageCell,
        sample_step: int,
    ) -> List[Tuple[float, float]]:
        points = []
        for row, start_col, end_col in cell.rows:
            for col in range(start_col, end_col + 1, sample_step):
                points.append(self._cell_to_world(grid, col, row))
            if (end_col - start_col) % sample_step != 0:
                points.append(self._cell_to_world(grid, end_col, row))
        return points

    def _oriented_run_to_segment(
        self,
        center_x: float,
        center_y: float,
        axis_x: float,
        axis_y: float,
        cross_x: float,
        cross_y: float,
        run: List[Tuple[float, float]],
        waypoint_spacing: float,
    ) -> List[PoseStamped]:
        if len(run) < 2:
            return []

        across = sum(value[1] for value in run) / len(run)
        start_along = run[0][0]
        end_along = run[-1][0]
        distance = end_along - start_along
        step = waypoint_spacing if distance >= 0.0 else -waypoint_spacing
        count = max(1, int(abs(distance) / waypoint_spacing))

        segment = []
        for index in range(count + 1):
            along = start_along + step * index
            if (step > 0 and along > end_along) or (step < 0 and along < end_along):
                along = end_along
            x = center_x + axis_x * along + cross_x * across
            y = center_y + axis_y * along + cross_y * across
            segment.append(self._pose(x, y, 0.0))

        return segment

    @staticmethod
    def _segment_length(segment: List[PoseStamped]) -> float:
        if len(segment) < 2:
            return 0.0
        total = 0.0
        for index in range(len(segment) - 1):
            start = segment[index].pose.position
            end = segment[index + 1].pose.position
            total += math.hypot(end.x - start.x, end.y - start.y)
        return total

    def _horizontal_cell_sweep_segments(
        self,
        grid: OccupancyGrid,
        cell: CoverageCell,
        point_step: int,
        reverse_cell: bool,
    ) -> List[List[PoseStamped]]:
        cell_rows = sorted(cell.rows, key=lambda item: item[0])
        if reverse_cell:
            cell_rows = list(reversed(cell_rows))

        segments: List[List[PoseStamped]] = []
        reverse_row = reverse_cell
        for row, start_col, end_col in cell_rows:
            cols = range(start_col, end_col + 1, point_step)
            if reverse_row:
                cols = range(end_col, start_col - 1, -point_step)

            segment = []
            for col in cols:
                x, y = self._cell_to_world(grid, col, row)
                segment.append(self._pose(x, y, 0.0))
            if len(segment) >= 2:
                segments.append(segment)

            reverse_row = not reverse_row

        return segments

    def _vertical_cell_sweep_segments(
        self,
        grid: OccupancyGrid,
        cell: CoverageCell,
        line_step: int,
        point_step: int,
        reverse_cell: bool,
    ) -> List[List[PoseStamped]]:
        min_col, max_col, min_row, max_row = cell.bounds()
        cols = range(min_col, max_col + 1, line_step)
        if reverse_cell:
            cols = range(max_col, min_col - 1, -line_step)

        segments: List[List[PoseStamped]] = []
        reverse_col = reverse_cell
        for col in cols:
            runs = self._cell_row_runs_for_column(cell, col, min_row, max_row)
            if reverse_col:
                runs = list(reversed(runs))

            for start_row, end_row in runs:
                rows = range(start_row, end_row + 1, point_step)
                if reverse_col:
                    rows = range(end_row, start_row - 1, -point_step)

                segment = []
                for row in rows:
                    x, y = self._cell_to_world(grid, col, row)
                    segment.append(self._pose(x, y, 0.0))
                if len(segment) >= 2:
                    segments.append(segment)

            reverse_col = not reverse_col

        return segments

    def _cell_row_runs_for_column(
        self,
        cell: CoverageCell,
        col: int,
        min_row: int,
        max_row: int,
    ) -> List[Tuple[int, int]]:
        rows = sorted(row for row in range(min_row, max_row + 1) if cell.contains(col, row))
        if not rows:
            return []

        runs = []
        run_start = rows[0]
        previous = rows[0]
        for row in rows[1:]:
            if row == previous + 1:
                previous = row
                continue
            runs.append((run_start, previous))
            run_start = row
            previous = row

        runs.append((run_start, previous))
        return runs

    def _decompose_boustrophedon_cells(
        self,
        grid: OccupancyGrid,
        allowed_cells: Optional[List[bool]],
        line_step: int,
        min_run_cells: int,
    ) -> List[CoverageCell]:
        cells: List[CoverageCell] = []
        active: List[Tuple[int, Tuple[int, int]]] = []
        min_cell_rows = max(
            1,
            round(float(self.get_parameter('min_cell_height').value) / (
                grid.info.resolution * line_step
            )),
        )

        for row in range(0, grid.info.height, line_step):
            intervals = self._free_runs_for_row(grid, row, min_run_cells, allowed_cells)
            if not intervals:
                active = []
                continue

            prev_overlap_counts = []
            for _, prev_interval in active:
                count = sum(1 for interval in intervals if self._interval_overlaps(prev_interval, interval))
                prev_overlap_counts.append(count)

            new_active: List[Tuple[int, Tuple[int, int]]] = []
            for interval in intervals:
                overlaps = [
                    (index, cell_index)
                    for index, (cell_index, prev_interval) in enumerate(active)
                    if self._interval_overlaps(prev_interval, interval)
                ]

                if len(overlaps) == 1 and prev_overlap_counts[overlaps[0][0]] == 1:
                    cell_index = overlaps[0][1]
                else:
                    cell_index = len(cells)
                    cells.append(CoverageCell())

                cells[cell_index].append(row, interval[0], interval[1])
                new_active.append((cell_index, interval))

            active = new_active

        filtered_cells = [cell for cell in cells if len(cell.rows) >= min_cell_rows]
        if not filtered_cells:
            return cells
        return filtered_cells

    def _order_cells_from_start(
        self,
        grid: OccupancyGrid,
        cells: List[CoverageCell],
    ) -> List[CoverageCell]:
        remaining = cells[:]
        ordered = []
        current = (
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value),
        )

        while remaining:
            best_index = min(
                range(len(remaining)),
                key=lambda index: self._distance_squared(current, remaining[index].centroid(grid)),
            )
            cell = remaining.pop(best_index)
            ordered.append(cell)
            current = cell.centroid(grid)

        return ordered

    @staticmethod
    def _interval_overlaps(first: Tuple[int, int], second: Tuple[int, int]) -> bool:
        return first[0] <= second[1] and second[0] <= first[1]

    @staticmethod
    def _distance_squared(first: Tuple[float, float], second: Tuple[float, float]) -> float:
        dx = first[0] - second[0]
        dy = first[1] - second[1]
        return dx * dx + dy * dy

    def _free_runs_for_row(
        self,
        grid: OccupancyGrid,
        row: int,
        min_run_cells: int,
        allowed_cells: Optional[List[bool]],
    ) -> List[Tuple[int, int]]:
        runs = []
        run_start = None

        for col in range(grid.info.width):
            if self._is_allowed_cell(grid, col, row, allowed_cells):
                if run_start is None:
                    run_start = col
            elif run_start is not None:
                if col - run_start >= min_run_cells:
                    runs.append((run_start, col - 1))
                run_start = None

        if run_start is not None and grid.info.width - run_start >= min_run_cells:
            runs.append((run_start, grid.info.width - 1))

        return runs

    def _allowed_cells(self, grid: OccupancyGrid) -> Optional[List[bool]]:
        if not bool(self.get_parameter('restrict_to_start_region').value):
            return None

        width = grid.info.width
        height = grid.info.height
        start_x = float(self.get_parameter('start_x').value)
        start_y = float(self.get_parameter('start_y').value)
        start_col, start_row = self._world_to_cell(grid, start_x, start_y)

        if not self._is_clear_cell(grid, start_col, start_row):
            nearest = self._nearest_clear_cell(grid, start_col, start_row)
            if nearest is None:
                self.get_logger().warn(
                    'Start pose is not in a clear map cell and no clear cell was found.'
                )
                return None
            start_col, start_row = nearest
            self.get_logger().warn(
                f'Start pose is not clear; using nearest clear cell ({start_col}, {start_row}).'
            )

        allowed = [False] * (width * height)
        queue = deque([(start_col, start_row)])
        allowed[start_row * width + start_col] = True

        while queue:
            col, row = queue.popleft()
            for next_col, next_row in (
                (col + 1, row),
                (col - 1, row),
                (col, row + 1),
                (col, row - 1),
            ):
                if next_col < 0 or next_row < 0 or next_col >= width or next_row >= height:
                    continue
                index = next_row * width + next_col
                if allowed[index] or not self._is_clear_cell(grid, next_col, next_row):
                    continue
                allowed[index] = True
                queue.append((next_col, next_row))

        return allowed

    def _is_allowed_cell(
        self,
        grid: OccupancyGrid,
        col: int,
        row: int,
        allowed_cells: Optional[List[bool]],
    ) -> bool:
        if allowed_cells is not None:
            return allowed_cells[row * grid.info.width + col]
        return self._is_clear_cell(grid, col, row)

    def _is_clear_cell(self, grid: OccupancyGrid, col: int, row: int) -> bool:
        clearance_cells = max(
            0,
            round(float(self.get_parameter('wall_clearance').value) / grid.info.resolution),
        )
        occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)

        for check_row in range(row - clearance_cells, row + clearance_cells + 1):
            for check_col in range(col - clearance_cells, col + clearance_cells + 1):
                if check_col < 0 or check_row < 0:
                    return False
                if check_col >= grid.info.width or check_row >= grid.info.height:
                    return False

                value = grid.data[check_row * grid.info.width + check_col]
                if value < 0 and unknown_is_obstacle:
                    return False
                if value >= occupied_threshold:
                    return False

        return True

    def _cell_to_world(self, grid: OccupancyGrid, col: int, row: int) -> Tuple[float, float]:
        x = grid.info.origin.position.x + (col + 0.5) * grid.info.resolution
        y = grid.info.origin.position.y + (row + 0.5) * grid.info.resolution
        return x, y

    def _world_to_cell(self, grid: OccupancyGrid, x: float, y: float) -> Tuple[int, int]:
        col = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        col = min(max(col, 0), grid.info.width - 1)
        row = min(max(row, 0), grid.info.height - 1)
        return col, row

    def _nearest_clear_cell(
        self,
        grid: OccupancyGrid,
        start_col: int,
        start_row: int,
    ) -> Optional[Tuple[int, int]]:
        width = grid.info.width
        height = grid.info.height
        visited = [False] * (width * height)
        queue = deque([(start_col, start_row)])
        visited[start_row * width + start_col] = True

        while queue:
            col, row = queue.popleft()
            if self._is_clear_cell(grid, col, row):
                return col, row
            for next_col, next_row in (
                (col + 1, row),
                (col - 1, row),
                (col, row + 1),
                (col, row - 1),
            ):
                if next_col < 0 or next_row < 0 or next_col >= width or next_row >= height:
                    continue
                index = next_row * width + next_col
                if visited[index]:
                    continue
                visited[index] = True
                queue.append((next_col, next_row))

        return None

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.get_parameter('map_frame').value
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.05
        qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _orient_segments(self, segments: List[List[PoseStamped]]) -> List[List[PoseStamped]]:
        for segment in segments:
            for index, waypoint in enumerate(segment):
                if index < len(segment) - 1:
                    next_waypoint = segment[index + 1]
                    dx = next_waypoint.pose.position.x - waypoint.pose.position.x
                    dy = next_waypoint.pose.position.y - waypoint.pose.position.y
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = float(self.get_parameter('start_yaw').value)

                qz, qw = yaw_to_quaternion(yaw)
                waypoint.pose.orientation.z = qz
                waypoint.pose.orientation.w = qw

        return segments

    def _orient_waypoints(self, waypoints: List[PoseStamped]) -> List[PoseStamped]:
        for index, waypoint in enumerate(waypoints):
            if index < len(waypoints) - 1:
                next_waypoint = waypoints[index + 1]
                dx = next_waypoint.pose.position.x - waypoint.pose.position.x
                dy = next_waypoint.pose.position.y - waypoint.pose.position.y
                yaw = math.atan2(dy, dx)
            else:
                yaw = float(self.get_parameter('start_yaw').value)

            qz, qw = yaw_to_quaternion(yaw)
            waypoint.pose.orientation.z = qz
            waypoint.pose.orientation.w = qw

        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = CoverageCleaner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
