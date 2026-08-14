#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np
import yaml


Point = Tuple[float, float]
PathEntry = Tuple[int, float, List[Point]]


def load_ros_map(map_yaml: Path):
    with map_yaml.open("r", encoding="utf-8") as stream:
        info = yaml.safe_load(stream)

    image_path = Path(info["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path

    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Could not read map image: {image_path}")

    return info, image


def free_space_mask(image: np.ndarray, info: Dict) -> np.ndarray:
    negate = int(info.get("negate", 0))
    free_thresh = float(info.get("free_thresh", 0.25))

    if negate:
        probability = image.astype(np.float32) / 255.0
    else:
        probability = (255.0 - image.astype(np.float32)) / 255.0

    free = probability <= free_thresh
    return free.astype(np.uint8)


def white_pixel_mask(image: np.ndarray, threshold: int) -> np.ndarray:
    return (image >= threshold).astype(np.uint8)


def keep_largest_component(mask: np.ndarray) -> np.ndarray:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return mask

    largest_label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return (labels == largest_label).astype(np.uint8)


def keep_component_containing(mask: np.ndarray, point: Point) -> np.ndarray:
    col = int(round(point[0]))
    row = int(round(point[1]))
    if row < 0 or col < 0 or row >= mask.shape[0] or col >= mask.shape[1] or mask[row, col] == 0:
        return keep_largest_component(mask)

    count, labels, _, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return mask

    label = int(labels[row, col])
    if label <= 0:
        return keep_largest_component(mask)
    return (labels == label).astype(np.uint8)


def chaikin_closed(points: Sequence[Point], iterations: int) -> List[Point]:
    smoothed = list(points)
    for _ in range(max(0, iterations)):
        if len(smoothed) < 3:
            return smoothed

        next_points: List[Point] = []
        for index, point in enumerate(smoothed):
            following = smoothed[(index + 1) % len(smoothed)]
            q = (
                0.75 * point[0] + 0.25 * following[0],
                0.75 * point[1] + 0.25 * following[1],
            )
            r = (
                0.25 * point[0] + 0.75 * following[0],
                0.25 * point[1] + 0.75 * following[1],
            )
            next_points.extend((q, r))
        smoothed = next_points
    return smoothed


def closed_length(points: Sequence[Point]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(
        math.hypot(
            points[(index + 1) % len(points)][0] - points[index][0],
            points[(index + 1) % len(points)][1] - points[index][1],
        )
        for index in range(len(points))
    )


def signed_area(points: Sequence[Point]) -> float:
    if len(points) < 3:
        return 0.0
    return 0.5 * sum(
        (points[index][0] * points[(index + 1) % len(points)][1])
        - (points[(index + 1) % len(points)][0] * points[index][1])
        for index in range(len(points))
    )


def orient_first_loop_for_right_edge_cleaning(paths: Sequence[List[Point]]) -> List[List[Point]]:
    oriented = [list(path) for path in paths]
    if not oriented:
        return oriented

    # Image Y points downward, so negative image area means counter-clockwise in map/world.
    # For the outer wall loop, world counter-clockwise keeps the wall on the robot's right.
    if signed_area(oriented[0]) > 0.0:
        oriented[0].reverse()
    return oriented


def resample_closed(points: Sequence[Point], spacing: float) -> List[Point]:
    if len(points) < 2:
        return list(points)

    spacing = max(1.0, spacing)
    perimeter = closed_length(points)
    if perimeter <= spacing:
        return list(points)

    samples = max(3, int(round(perimeter / spacing)))
    resampled: List[Point] = []
    target_distance = 0.0
    accumulated = 0.0
    segment_index = 0

    while len(resampled) < samples:
        start = points[segment_index % len(points)]
        end = points[(segment_index + 1) % len(points)]
        segment_length = math.hypot(end[0] - start[0], end[1] - start[1])

        if segment_length <= 1e-6:
            segment_index += 1
            continue

        if accumulated + segment_length >= target_distance:
            ratio = (target_distance - accumulated) / segment_length
            resampled.append(
                (
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                )
            )
            target_distance += perimeter / samples
        else:
            accumulated += segment_length
            segment_index += 1

    return resampled


def rotate_to_nearest(points: Sequence[Point], target: Point) -> List[Point]:
    if not points:
        return []
    nearest = min(
        range(len(points)),
        key=lambda index: (points[index][0] - target[0]) ** 2 + (points[index][1] - target[1]) ** 2,
    )
    return list(points[nearest:]) + list(points[:nearest])


def order_paths(paths: Sequence[List[Point]], start: Point) -> List[List[Point]]:
    remaining = [list(path) for path in paths]
    ordered: List[List[Point]] = []
    cursor = start

    while remaining:
        best_index = min(
            range(len(remaining)),
            key=lambda index: min(
                (point[0] - cursor[0]) ** 2 + (point[1] - cursor[1]) ** 2
                for point in remaining[index]
            ),
        )
        path = rotate_to_nearest(remaining.pop(best_index), cursor)
        ordered.append(path)
        cursor = path[-1]

    return ordered


def image_to_world(point: Point, info: Dict, height: int) -> Point:
    resolution = float(info["resolution"])
    origin_x, origin_y, _ = info["origin"]
    col, row = point
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (height - row - 0.5) * resolution
    return x, y


def world_to_image(point: Point, info: Dict, height: int) -> Point:
    resolution = float(info["resolution"])
    origin_x, origin_y, _ = info["origin"]
    x, y = point
    col = (x - origin_x) / resolution - 0.5
    row = height - ((y - origin_y) / resolution) - 0.5
    return col, row


def contour_depth(index: int, hierarchy: np.ndarray) -> int:
    depth = 0
    parent = int(hierarchy[index][3])
    while parent >= 0:
        depth += 1
        parent = int(hierarchy[parent][3])
    return depth


def contour_path_entries(
    mask: np.ndarray,
    resolution: float,
    min_length_m: float,
    simplify_m: float,
    smoothing_iterations: int,
    waypoint_spacing_m: float,
) -> List[PathEntry]:
    contours, hierarchy = cv2.findContours(
        (mask * 255).astype(np.uint8),
        cv2.RETR_TREE,
        cv2.CHAIN_APPROX_NONE,
    )

    if hierarchy is None:
        return []

    path_entries: List[PathEntry] = []
    simplify_px = max(0.0, simplify_m / resolution)
    waypoint_spacing_px = max(1.0, waypoint_spacing_m / resolution)
    min_length_px = min_length_m / resolution
    hierarchy = hierarchy[0]

    for contour_index, contour in enumerate(contours):
        if len(contour) < 12:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter < min_length_px:
            continue

        simplified = cv2.approxPolyDP(contour, simplify_px, True)
        points = [(float(point[0][0]), float(point[0][1])) for point in simplified]
        if len(points) < 3:
            continue

        points = chaikin_closed(points, smoothing_iterations)
        points = resample_closed(points, waypoint_spacing_px)
        if len(points) >= 3:
            depth = contour_depth(contour_index, hierarchy)
            area = abs(cv2.contourArea(contour))
            path_entries.append((depth, area, points))

    return path_entries


def contour_paths(
    safe_mask: np.ndarray,
    resolution: float,
    min_length_m: float,
    simplify_m: float,
    smoothing_iterations: int,
    waypoint_spacing_m: float,
) -> List[List[Point]]:
    path_entries = contour_path_entries(
        safe_mask,
        resolution,
        min_length_m,
        simplify_m,
        smoothing_iterations,
        waypoint_spacing_m,
    )
    outer_paths = sorted(
        (entry for entry in path_entries if entry[0] == 0),
        key=lambda entry: entry[1],
        reverse=True,
    )
    inner_paths = sorted(
        (entry for entry in path_entries if entry[0] > 0),
        key=lambda entry: (entry[0], entry[1]),
        reverse=True,
    )

    prioritized = outer_paths[:1] + inner_paths[:1]
    used_path_ids = {id(entry[2]) for entry in prioritized}
    remaining = sorted(
        (entry for entry in path_entries if id(entry[2]) not in used_path_ids),
        key=lambda entry: closed_length(entry[2]),
        reverse=True,
    )

    return [entry[2] for entry in prioritized + remaining]


def _neighbors8(col: int, row: int):
    for row_delta in (-1, 0, 1):
        for col_delta in (-1, 0, 1):
            if col_delta == 0 and row_delta == 0:
                continue
            yield col + col_delta, row + row_delta


def _neighbor_indices(width: int, height: int, index: int) -> List[int]:
    col = index % width
    row = index // width
    neighbors: List[int] = []
    for next_col, next_row in _neighbors8(col, row):
        if 0 <= next_col < width and 0 <= next_row < height:
            neighbors.append(next_row * width + next_col)
    return neighbors


def _cell_graph_degree(width: int, height: int, cells: set, index: int) -> int:
    return sum(1 for neighbor in _neighbor_indices(width, height, index) if neighbor in cells)


def _largest_component_indices(width: int, height: int, cells: Sequence[int]) -> List[int]:
    remaining = set(cells)
    largest: List[int] = []

    while remaining:
        start = remaining.pop()
        component = [start]
        stack = [start]
        while stack:
            current = stack.pop()
            for neighbor in _neighbor_indices(width, height, current):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
                    component.append(neighbor)

        if len(component) > len(largest):
            largest = component

    return largest


def _prune_dead_end_indices(width: int, height: int, cells: Sequence[int]) -> List[int]:
    remaining = set(cells)
    queue = [
        index
        for index in remaining
        if _cell_graph_degree(width, height, remaining, index) <= 1
    ]

    while queue:
        index = queue.pop()
        if index not in remaining:
            continue
        if _cell_graph_degree(width, height, remaining, index) > 1:
            continue

        remaining.remove(index)
        for neighbor in _neighbor_indices(width, height, index):
            if neighbor in remaining and _cell_graph_degree(width, height, remaining, neighbor) <= 1:
                queue.append(neighbor)

    return list(remaining)


def _best_continuation_cell(width: int, previous: int, current: int, candidates: Sequence[int]) -> int:
    prev_col = previous % width
    prev_row = previous // width
    current_col = current % width
    current_row = current // width
    heading_x = current_col - prev_col
    heading_y = current_row - prev_row

    def score(index: int) -> Tuple[float, float]:
        next_col = index % width
        next_row = index // width
        step_x = next_col - current_col
        step_y = next_row - current_row
        dot = heading_x * step_x + heading_y * step_y
        cross = abs(heading_x * step_y - heading_y * step_x)
        return (-dot, cross)

    return min(candidates, key=score)


def _shortest_path_excluding(
    width: int,
    height: int,
    cells: set,
    start: int,
    goal: int,
    excluded: int,
) -> List[int]:
    queue = [start]
    parents = {start: None}
    cursor = 0

    while cursor < len(queue):
        current = queue[cursor]
        cursor += 1
        if current == goal:
            break

        for neighbor in _neighbor_indices(width, height, current):
            if neighbor == excluded or neighbor not in cells or neighbor in parents:
                continue
            parents[neighbor] = current
            queue.append(neighbor)

    if goal not in parents:
        return []

    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = parents[current]
    path.reverse()
    return path


def _cycle_through_node(width: int, height: int, cells: set, node: int) -> List[int]:
    neighbors = [index for index in _neighbor_indices(width, height, node) if index in cells]
    if len(neighbors) < 2:
        return []

    best_cycle: List[int] = []
    for first_index, first in enumerate(neighbors):
        for second in neighbors[first_index + 1:]:
            path = _shortest_path_excluding(width, height, cells, first, second, node)
            if path and len(path) + 2 > len(best_cycle):
                best_cycle = [node] + path + [node]

    return best_cycle


def _free_runs_in_row(mask: np.ndarray, row: int, min_run_cells: int) -> List[Tuple[int, int]]:
    width = mask.shape[1]
    runs: List[Tuple[int, int]] = []
    start_col = None

    for col in range(width):
        is_free = bool(mask[row, col])
        if is_free and start_col is None:
            start_col = col
        elif not is_free and start_col is not None:
            end_col = col - 1
            if end_col - start_col + 1 >= min_run_cells:
                runs.append((start_col, end_col))
            start_col = None

    if start_col is not None:
        end_col = width - 1
        if end_col - start_col + 1 >= min_run_cells:
            runs.append((start_col, end_col))

    return runs


def _free_runs_in_col(mask: np.ndarray, col: int, min_run_cells: int) -> List[Tuple[int, int]]:
    height = mask.shape[0]
    runs: List[Tuple[int, int]] = []
    start_row = None

    for row in range(height):
        is_free = bool(mask[row, col])
        if is_free and start_row is None:
            start_row = row
        elif not is_free and start_row is not None:
            end_row = row - 1
            if end_row - start_row + 1 >= min_run_cells:
                runs.append((start_row, end_row))
            start_row = None

    if start_row is not None:
        end_row = height - 1
        if end_row - start_row + 1 >= min_run_cells:
            runs.append((start_row, end_row))

    return runs


def hallway_centerline_cells(mask: np.ndarray, resolution: float, min_run_m: float) -> List[int]:
    height, width = mask.shape
    min_run_cells = max(3, int(round(min_run_m / resolution)))
    cells = set()

    for row in range(height):
        for start_col, end_col in _free_runs_in_row(mask, row, min_run_cells):
            cells.add(row * width + ((start_col + end_col) // 2))

    for col in range(width):
        for start_row, end_row in _free_runs_in_col(mask, col, min_run_cells):
            cells.add(((start_row + end_row) // 2) * width + col)

    return list(cells)


def skeleton_cells(mask: np.ndarray) -> List[int]:
    image = (mask > 0).astype(np.uint8) * 255
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        skeleton = cv2.ximgproc.thinning(image)
    else:
        skeleton = np.zeros_like(image)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

        while cv2.countNonZero(image) > 0:
            eroded = cv2.erode(image, element)
            opened = cv2.dilate(eroded, element)
            skeleton = cv2.bitwise_or(skeleton, cv2.subtract(image, opened))
            image = eroded

    rows, cols = np.nonzero(skeleton > 0)
    width = mask.shape[1]
    return [int(row) * width + int(col) for row, col in zip(rows, cols)]


def trace_center_loop(
    mask: np.ndarray,
    resolution: float,
    start: Point,
    min_run_m: float,
    waypoint_spacing_m: float,
) -> List[Point]:
    height, width = mask.shape
    cells = skeleton_cells(mask)
    if not cells:
        cells = hallway_centerline_cells(mask, resolution, min_run_m)
    if not cells:
        return []

    loop_cells = _largest_component_indices(width, height, cells)
    cycle_cells = _prune_dead_end_indices(width, height, loop_cells)
    if len(cycle_cells) >= max(8, len(loop_cells) // 4):
        loop_cells = _largest_component_indices(width, height, cycle_cells)

    if not loop_cells:
        return []

    cell_set = set(loop_cells)
    cycle_start_candidates = [
        index for index in cell_set if _cell_graph_degree(width, height, cell_set, index) >= 2
    ]
    if not cycle_start_candidates:
        cycle_start_candidates = list(cell_set)

    start_cell = min(
        cycle_start_candidates,
        key=lambda index: (
            ((index % width) + 0.5 - start[0]) ** 2
            + ((index // width) + 0.5 - start[1]) ** 2
        ),
    )

    ordered_cells = _cycle_through_node(width, height, cell_set, start_cell)
    if not ordered_cells:
        ordered_cells = [start_cell]
        visited = {start_cell}
        previous = None
        current = start_cell
        max_steps = max(1, len(cell_set) + 1)

        for _ in range(max_steps):
            candidates = [
                index
                for index in _neighbor_indices(width, height, current)
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
                current_col = current % width
                current_row = current // width
                next_cell = min(
                    unvisited,
                    key=lambda index: math.atan2((index // width) - current_row, (index % width) - current_col),
                )
            else:
                next_cell = _best_continuation_cell(width, previous, current, unvisited)

            ordered_cells.append(next_cell)
            visited.add(next_cell)
            previous = current
            current = next_cell

        if ordered_cells[-1] != start_cell and start_cell in _neighbor_indices(width, height, ordered_cells[-1]):
            ordered_cells.append(start_cell)

    spacing_px = max(1.0, waypoint_spacing_m / resolution)
    path: List[Point] = []
    last_point = None
    for cell in ordered_cells:
        point = ((cell % width) + 0.5, (cell // width) + 0.5)
        if last_point is None or math.hypot(point[0] - last_point[0], point[1] - last_point[1]) >= spacing_px:
            path.append(point)
            last_point = point

    if (
        ordered_cells[-1] == start_cell
        and len(path) >= 2
        and math.hypot(path[0][0] - path[-1][0], path[0][1] - path[-1][1]) >= 1.0
    ):
        path.append(path[0])

    return path


def coverage_loop_paths(
    distance: np.ndarray,
    resolution: float,
    wall_clearance_m: float,
    loop_spacing_m: float,
    min_length_m: float,
    simplify_m: float,
    smoothing_iterations: int,
    waypoint_spacing_m: float,
) -> List[List[Point]]:
    paths: List[List[Point]] = []
    max_distance_m = float(distance.max()) * resolution
    offset_m = max(resolution, wall_clearance_m)
    loop_spacing_m = max(resolution, loop_spacing_m)

    while offset_m <= max_distance_m + (0.5 * resolution):
        offset_px = max(1, int(round(offset_m / resolution)))
        mask = (distance >= offset_px).astype(np.uint8)
        path_entries = contour_path_entries(
            mask,
            resolution,
            min_length_m,
            simplify_m,
            smoothing_iterations,
            waypoint_spacing_m,
        )
        if not path_entries:
            break

        outer_paths = sorted(
            (entry for entry in path_entries if entry[0] == 0),
            key=lambda entry: entry[1],
            reverse=True,
        )
        inner_paths = sorted(
            (entry for entry in path_entries if entry[0] > 0),
            key=lambda entry: (entry[0], entry[1]),
            reverse=True,
        )

        for entries in (outer_paths, inner_paths):
            if entries:
                paths.append(entries[0][2])

        offset_m += loop_spacing_m

    return paths


def rotate_paths_to_nearest(paths: Sequence[List[Point]], start: Point) -> List[List[Point]]:
    rotated: List[List[Point]] = []
    cursor = start
    for path in paths:
        rotated_path = rotate_to_nearest(path, cursor)
        rotated.append(rotated_path)
        if rotated_path:
            cursor = rotated_path[-1]
    return rotated


def draw_preview(
    image: np.ndarray,
    free: np.ndarray,
    safe: np.ndarray,
    paths: Sequence[Sequence[Point]],
    output_path: Path,
):
    preview = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    preview[free == 0] = (70, 70, 70)
    preview[safe > 0] = (
        0.65 * preview[safe > 0] + np.array([45.0, 105.0, 45.0])
    ).astype(np.uint8)

    colors = [
        (40, 220, 255),  # orange in RGB viewer
        (255, 170, 40),  # blue in RGB viewer
        (80, 255, 120),
        (230, 90, 255),
        (80, 160, 255),
    ]

    for path_index, path in enumerate(paths):
        color = colors[path_index % len(colors)]
        pixels = np.array([[round(x), round(y)] for x, y in path], dtype=np.int32)
        if len(pixels) >= 2:
            closed = np.linalg.norm(pixels[0].astype(float) - pixels[-1].astype(float)) <= 2.0
            cv2.polylines(preview, [pixels], closed, color, 2, lineType=cv2.LINE_AA)
            cv2.circle(preview, tuple(pixels[0]), 4, (0, 0, 255), -1)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), preview)


def write_path_csv(paths: Sequence[Sequence[Point]], info: Dict, height: int, output_path: Path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["segment", "index", "x", "y", "yaw"])
        for segment_index, path in enumerate(paths):
            world_points = [image_to_world(point, info, height) for point in path]
            is_closed = (
                len(world_points) >= 3
                and math.hypot(
                    world_points[0][0] - world_points[-1][0],
                    world_points[0][1] - world_points[-1][1],
                )
                <= float(info["resolution"]) * 2.0
            )
            for index, point in enumerate(world_points):
                if index + 1 < len(world_points):
                    following = world_points[index + 1]
                elif is_closed:
                    following = world_points[0]
                else:
                    following = world_points[index - 1]
                yaw = math.atan2(following[1] - point[1], following[0] - point[0])
                writer.writerow(
                    [
                        segment_index,
                        index,
                        f"{point[0]:.4f}",
                        f"{point[1]:.4f}",
                        f"{yaw:.4f}",
                    ]
                )


def main():
    parser = argparse.ArgumentParser(
        description="OpenCV wall-tracing coverage path demo for ROS occupancy maps."
    )
    parser.add_argument(
        "--map",
        default="/home/user/maps/EERCsB/EERCsBsTRAIT.yaml",
        type=Path,
        help="ROS map YAML to read.",
    )
    parser.add_argument(
        "--output-dir",
        default="/home/user/test_ws/opencv_wall_trace_demo",
        type=Path,
        help="Directory for generated preview/path files.",
    )
    parser.add_argument("--start-x", default=0.22198301553726196, type=float)
    parser.add_argument("--start-y", default=-0.06558487564325333, type=float)
    parser.add_argument(
        "--robot-width",
        default=0.3485,
        type=float,
        help="Robot cleaning width in meters. Loop spacing is two thirds of this width.",
    )
    parser.add_argument(
        "--wall-clearance",
        default=None,
        type=float,
        help="Meters to offset path from walls/unknown cells. Defaults to half the robot width.",
    )
    parser.add_argument(
        "--waypoint-spacing",
        default=0.15,
        type=float,
        help="Meters between output points on the smooth path.",
    )
    parser.add_argument(
        "--path-mode",
        choices=("coverage_loops", "center_loop"),
        default="coverage_loops",
        help="coverage_loops traces offset wall loops; center_loop makes one hallway middle practice loop.",
    )
    parser.add_argument(
        "--centerline-min-run",
        default=1.20,
        type=float,
        help="Minimum free-space run length in meters used to identify hallway centerline cells.",
    )
    parser.add_argument(
        "--simplify",
        default=0.04,
        type=float,
        help="Meters of contour simplification before smoothing. Higher means fewer turns.",
    )
    parser.add_argument(
        "--smooth-iterations",
        default=1,
        type=int,
        help="Chaikin smoothing passes. Higher means rounder corners.",
    )
    parser.add_argument(
        "--min-length",
        default=0.80,
        type=float,
        help="Ignore contours shorter than this many meters.",
    )
    parser.add_argument(
        "--all-components",
        action="store_true",
        help="Use every free-space component instead of only the largest one.",
    )
    parser.add_argument(
        "--mask-source",
        choices=("white", "ros"),
        default="white",
        help="Use literal white map pixels or ROS occupancy thresholds as the cleanable area.",
    )
    parser.add_argument(
        "--white-threshold",
        default=250,
        type=int,
        help="Pixel value treated as cleanable white floor when --mask-source white.",
    )
    args = parser.parse_args()

    info, image = load_ros_map(args.map)
    resolution = float(info["resolution"])
    start_image = world_to_image((args.start_x, args.start_y), info, image.shape[0])
    if args.mask_source == "white":
        free = white_pixel_mask(image, args.white_threshold)
    else:
        free = free_space_mask(image, info)
    if not args.all_components:
        free = keep_component_containing(free, start_image)

    wall_clearance = args.wall_clearance
    if wall_clearance is None:
        wall_clearance = args.robot_width / 2.0

    clearance_px = max(1, int(round(wall_clearance / resolution)))
    loop_spacing = (2.0 / 3.0) * args.robot_width
    loop_spacing_px = max(1, int(round(loop_spacing / resolution)))
    distance = cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    safe = (distance >= clearance_px).astype(np.uint8)

    if args.path_mode == "center_loop":
        center_path = trace_center_loop(
            safe,
            resolution,
            start_image,
            args.centerline_min_run,
            args.waypoint_spacing,
        )
        paths = [center_path] if len(center_path) >= 2 else []
        preview_path = args.output_dir / "center_loop_preview.png"
        csv_path = args.output_dir / "center_loop_path.csv"
    else:
        paths = coverage_loop_paths(
            distance,
            resolution,
            wall_clearance,
            loop_spacing,
            args.min_length,
            args.simplify,
            args.smooth_iterations,
            args.waypoint_spacing,
        )
        paths = orient_first_loop_for_right_edge_cleaning(paths)
        paths = rotate_paths_to_nearest(paths, start_image)
        preview_path = args.output_dir / "wall_trace_preview.png"
        csv_path = args.output_dir / "wall_trace_path.csv"

    draw_preview(image, free, safe, paths, preview_path)
    write_path_csv(paths, info, image.shape[0], csv_path)

    total_points = sum(len(path) for path in paths)
    total_length_m = sum(closed_length(path) * resolution for path in paths)
    print(f"map: {args.map}")
    print(f"mask source: {args.mask_source}")
    if args.mask_source == "white":
        print(f"white threshold: {args.white_threshold}")
    print(f"resolution: {resolution:.3f} m/cell")
    print(f"clearance: {wall_clearance:.2f} m ({clearance_px} px)")
    print(f"robot width: {args.robot_width:.3f} m")
    print(f"path mode: {args.path_mode}")
    if args.path_mode == "coverage_loops":
        print(f"loop spacing: {loop_spacing:.3f} m ({loop_spacing_px} px)")
        print("first loop direction: outer wall kept on robot right")
    else:
        print(f"centerline min run: {args.centerline_min_run:.2f} m")
    print(f"segments: {len(paths)}")
    print(f"points: {total_points}")
    print(f"path length: {total_length_m:.2f} m")
    print(f"preview: {preview_path}")
    print(f"csv: {csv_path}")


if __name__ == "__main__":
    main()
