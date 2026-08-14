#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class OpenCvHallwayMappingDriver(Node):
    """Reactive hallway-center driver for live SLAM mapping."""

    def __init__(self):
        super().__init__('opencv_hallway_mapping_driver')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel_opencv_mapping')
        self.declare_parameter('enabled', True)
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('crawl_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.45)
        self.declare_parameter('lookahead_distance', 1.2)
        self.declare_parameter('min_forward_distance', 0.25)
        self.declare_parameter('side_range', 1.8)
        self.declare_parameter('front_arc_deg', 26.0)
        self.declare_parameter('slow_distance', 0.85)
        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('desired_single_wall_clearance', 0.75)
        self.declare_parameter('center_gain', 0.85)
        self.declare_parameter('heading_gain', 1.25)
        self.declare_parameter('min_wall_points', 5)
        self.declare_parameter('search_forward_when_uncertain', True)
        self.declare_parameter('search_angular_speed', 0.0)
        self.declare_parameter('command_timeout', 0.3)
        self.declare_parameter('debug_log_period', 1.0)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.crawl_speed = float(self.get_parameter('crawl_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.min_forward_distance = float(self.get_parameter('min_forward_distance').value)
        self.side_range = float(self.get_parameter('side_range').value)
        self.front_arc = math.radians(float(self.get_parameter('front_arc_deg').value))
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.desired_single_wall_clearance = float(
            self.get_parameter('desired_single_wall_clearance').value
        )
        self.center_gain = float(self.get_parameter('center_gain').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.min_wall_points = int(self.get_parameter('min_wall_points').value)
        self.search_forward_when_uncertain = bool(
            self.get_parameter('search_forward_when_uncertain').value
        )
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.debug_log_period = float(self.get_parameter('debug_log_period').value)

        scan_topic = str(self.get_parameter('scan_topic').value)
        cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.create_timer(self.command_timeout, self.watchdog_callback)

        self.last_scan_time = None
        self.last_debug_time = self.get_clock().now()
        self.last_cmd = Twist()

        self.get_logger().info(
            f'OpenCV hallway mapping driver listening on {scan_topic}, publishing {cmd_topic}'
        )

    def scan_callback(self, scan: LaserScan):
        self.last_scan_time = self.get_clock().now()
        cmd, status = self.compute_command(scan)
        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)

        now = self.get_clock().now()
        if (now - self.last_debug_time).nanoseconds * 1e-9 >= self.debug_log_period:
            self.last_debug_time = now
            self.get_logger().info(status)

    def watchdog_callback(self):
        if self.last_scan_time is None:
            return
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
        if age > self.command_timeout:
            self.cmd_pub.publish(Twist())

    def compute_command(self, scan: LaserScan) -> Tuple[Twist, str]:
        cmd = Twist()
        if not self.enabled:
            return cmd, 'disabled'

        points = self.scan_to_points(scan)
        front_min = self.front_clearance(scan)
        if front_min < self.stop_distance:
            return cmd, f'stopped: front obstacle {front_min:.2f} m'

        left = points[points[:, 1] > 0.10]
        right = points[points[:, 1] < -0.10]
        left_line = self.fit_wall_line(left)
        right_line = self.fit_wall_line(right)

        target_x = min(max(self.lookahead_distance * 0.75, 0.35), self.lookahead_distance)
        center_y = 0.0
        heading_error = 0.0
        confidence = 0

        if left_line and right_line:
            left_y = self.line_y_at_x(left_line, target_x)
            right_y = self.line_y_at_x(right_line, target_x)
            center_y = 0.5 * (left_y + right_y)
            heading_error = self.average_heading(left_line[2], right_line[2])
            confidence = 2
        elif left_line:
            left_y = self.line_y_at_x(left_line, target_x)
            center_y = left_y - self.desired_single_wall_clearance
            heading_error = left_line[2]
            confidence = 1
        elif right_line:
            right_y = self.line_y_at_x(right_line, target_x)
            center_y = right_y + self.desired_single_wall_clearance
            heading_error = right_line[2]
            confidence = 1
        else:
            if self.search_forward_when_uncertain and front_min > self.slow_distance:
                cmd.linear.x = self.crawl_speed
                cmd.angular.z = float(
                    np.clip(
                        self.search_angular_speed,
                        -self.max_angular_speed,
                        self.max_angular_speed,
                    )
                )
                return (
                    cmd,
                    f'searching: no hallway walls detected, front={front_min:.2f} m '
                    f'cmd=({cmd.linear.x:.2f}, {cmd.angular.z:+.2f})',
                )
            return cmd, 'stopped: no hallway walls detected'

        angular = self.center_gain * center_y + self.heading_gain * heading_error
        angular = float(np.clip(angular, -self.max_angular_speed, self.max_angular_speed))

        linear = self.linear_speed
        if front_min < self.slow_distance or confidence < 2:
            linear = min(linear, self.crawl_speed)
        if abs(angular) > 0.32:
            linear = min(linear, self.crawl_speed)

        cmd.linear.x = linear
        cmd.angular.z = angular
        return (
            cmd,
            f'center_y={center_y:+.2f} m heading={heading_error:+.2f} rad '
            f'front={front_min:.2f} m cmd=({linear:.2f}, {angular:+.2f}) confidence={confidence}',
        )

    def scan_to_points(self, scan: LaserScan) -> np.ndarray:
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(ranges.size, dtype=np.float32) * scan.angle_increment
        valid = np.isfinite(ranges)
        valid &= ranges >= max(scan.range_min, 0.05)
        valid &= ranges <= min(scan.range_max, self.side_range + 1.0)

        ranges = ranges[valid]
        angles = angles[valid]
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        roi = x >= self.min_forward_distance
        roi &= x <= self.lookahead_distance
        roi &= np.abs(y) <= self.side_range
        if not np.any(roi):
            return np.zeros((0, 2), dtype=np.float32)

        return np.column_stack((x[roi], y[roi])).astype(np.float32)

    def front_clearance(self, scan: LaserScan) -> float:
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(ranges.size, dtype=np.float32) * scan.angle_increment
        front = np.abs(angles) <= self.front_arc
        valid = front & np.isfinite(ranges)
        valid &= ranges >= max(scan.range_min, 0.05)
        valid &= ranges <= scan.range_max
        if not np.any(valid):
            return float('inf')
        return float(np.min(ranges[valid]))

    def fit_wall_line(self, points: np.ndarray) -> Optional[Tuple[float, float, float]]:
        if points.shape[0] < self.min_wall_points:
            return None

        fit = cv2.fitLine(points, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(-1)
        vx, vy, x0, y0 = (float(fit[0]), float(fit[1]), float(fit[2]), float(fit[3]))
        if abs(vx) < 1e-4:
            return None
        if vx < 0.0:
            vx = -vx
            vy = -vy
        slope = vy / vx
        heading = math.atan2(vy, vx)

        residual = points[:, 1] - (y0 + slope * (points[:, 0] - x0))
        keep = np.abs(residual) < 0.18
        if np.count_nonzero(keep) >= self.min_wall_points and np.count_nonzero(keep) < points.shape[0]:
            fit = cv2.fitLine(points[keep], cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(-1)
            vx, vy, x0, y0 = (float(fit[0]), float(fit[1]), float(fit[2]), float(fit[3]))
            if abs(vx) < 1e-4:
                return None
            if vx < 0.0:
                vx = -vx
                vy = -vy
            slope = vy / vx
            heading = math.atan2(vy, vx)

        return x0, y0, heading

    @staticmethod
    def line_y_at_x(line: Tuple[float, float, float], x: float) -> float:
        x0, y0, heading = line
        slope = math.tan(heading)
        return y0 + slope * (x - x0)

    @staticmethod
    def average_heading(a: float, b: float) -> float:
        return math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))


def main(args=None):
    rclpy.init(args=args)
    node = OpenCvHallwayMappingDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
