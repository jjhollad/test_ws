# Copyright 2026 jjhollad
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import sys

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


def frontier_cells(grid, clearance_cells=3):
    """Return free cells beside unknown space and away from occupied cells."""
    width = grid.info.width
    height = grid.info.height
    data = grid.data
    candidates = []
    for row in range(1, height - 1):
        for col in range(1, width - 1):
            index = row * width + col
            if data[index] != 0:
                continue
            neighbors = [
                data[index - 1], data[index + 1],
                data[index - width], data[index + width],
            ]
            if -1 not in neighbors:
                continue
            clear = True
            for check_row in range(
                max(0, row - clearance_cells),
                min(height, row + clearance_cells + 1),
            ):
                start = check_row * width + max(0, col - clearance_cells)
                end = check_row * width + min(width, col + clearance_cells + 1)
                if any(value >= 65 for value in data[start:end]):
                    clear = False
                    break
            if clear:
                candidates.append((col, row))
    return candidates


class FrontierMapper(Node):
    """Send conservative frontier goals to Nav2 while SLAM grows the map."""

    def __init__(self):
        super().__init__('frontier_mapper')
        self.declare_parameter('clearance_cells', 4)
        self.declare_parameter('minimum_goal_distance', 0.75)
        self.declare_parameter('goal_timeout_seconds', 90.0)
        self.map = None
        self.goal_handle = None
        self.goal_started_ns = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, qos)
        self.create_timer(2.0, self._tick)
        self.get_logger().info('Frontier mapper waiting for SLAM map and Nav2.')

    def _map_callback(self, message):
        self.map = message

    def _robot_xy(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except TransformException:
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _cell_xy(self, col, row):
        resolution = self.map.info.resolution
        origin = self.map.info.origin.position
        return (
            origin.x + (col + 0.5) * resolution,
            origin.y + (row + 0.5) * resolution,
        )

    def _tick(self):
        if self.map is None or self.goal_handle is not None:
            self._cancel_timed_out_goal()
            return
        robot = self._robot_xy()
        if robot is None or not self.navigator.wait_for_server(timeout_sec=0.1):
            return
        cells = frontier_cells(
            self.map,
            int(self.get_parameter('clearance_cells').value),
        )
        minimum = float(self.get_parameter('minimum_goal_distance').value)
        choices = []
        for col, row in cells:
            x, y = self._cell_xy(col, row)
            distance = math.hypot(x - robot[0], y - robot[1])
            if distance >= minimum:
                choices.append((distance, x, y))
        if not choices:
            self.get_logger().info('No reachable frontier candidate found.')
            return
        _distance, x, y = max(choices)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        future = self.navigator.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self.goal_started_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f'Sending frontier goal x={x:.2f}, y={y:.2f}.')

    def _goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.goal_started_ns = None
            return
        self.goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(self._goal_finished)

    def _goal_finished(self, _future):
        self.goal_handle = None
        self.goal_started_ns = None

    def _cancel_timed_out_goal(self):
        if self.goal_handle is None or self.goal_started_ns is None:
            return
        elapsed = (self.get_clock().now().nanoseconds - self.goal_started_ns) / 1e9
        timeout = float(self.get_parameter('goal_timeout_seconds').value)
        if elapsed >= timeout:
            self.get_logger().warn('Frontier goal timed out; cancelling it.')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.goal_started_ns = None


def main(args=None):
    rclpy.init(args=args)
    node = FrontierMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
