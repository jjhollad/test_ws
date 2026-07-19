#!/usr/bin/env python3
"""Remove robot self-hits and make a finite clearing scan for SLAM/Nav2."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan


class ScanClearer(Node):
    def __init__(self):
        super().__init__("scan_clearer")
        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("filtered_topic", "/scan")
        self.declare_parameter("clearing_topic", "/scan_clear")
        self.declare_parameter("clearing_margin", 0.05)
        self.declare_parameter("self_filter_margin", 0.02)
        self.declare_parameter("maximum_scan_age", 0.50)
        self.declare_parameter("stamp_offset_seconds", 0.0)
        # A sensor stream should favor the newest measurement. Reliable depth
        # ten could replay an old scan after CPU starvation, when its TF data
        # has already aged out of Nav2's cache.
        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        output_qos.durability = DurabilityPolicy.VOLATILE
        self._filtered_publisher = self.create_publisher(
            LaserScan, str(self.get_parameter("filtered_topic").value),
            output_qos,
        )
        self._clearing_publisher = self.create_publisher(
            LaserScan, str(self.get_parameter("clearing_topic").value),
            output_qos,
        )
        self.create_subscription(
            LaserScan, str(self.get_parameter("input_topic").value),
            self._scan, qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Filtering robot self-hits and generating finite clearing rays."
        )

    @staticmethod
    def _copy_scan(message):
        output = LaserScan()
        output.header = message.header
        output.angle_min = message.angle_min
        output.angle_max = message.angle_max
        output.angle_increment = message.angle_increment
        output.time_increment = message.time_increment
        output.scan_time = message.scan_time
        output.range_min = message.range_min
        output.range_max = message.range_max
        output.intensities = list(message.intensities)
        return output

    def _inside_robot(self, distance, angle):
        if not math.isfinite(distance):
            return False
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        margin = float(self.get_parameter("self_filter_margin").value)

        # Coordinates are in laser_frame. The lidar is at x=0.12 m in
        # base_link; the chassis spans x=[0, 0.8], y=[-0.3, 0.3].
        chassis = (-0.12 - margin <= x <= 0.68 + margin
                   and -0.30 - margin <= y <= 0.30 + margin)
        # Rear drive wheels are centered beneath the lidar at y=+/-0.285 m.
        left_wheel = math.hypot(x, y - 0.285) <= 0.12 + margin
        right_wheel = math.hypot(x, y + 0.285) <= 0.12 + margin
        # The side brush is centered 0.68 m forward and 0.275 m right.
        side_brush = math.hypot(x - 0.68, y + 0.275) <= 0.15 + margin
        return chassis or left_wheel or right_wheel or side_brush

    def _scan(self, message):
        stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        maximum_age = float(self.get_parameter("maximum_scan_age").value)
        if age > maximum_age:
            self.get_logger().warn(
                f"Rejecting LiDAR sample already {age:.3f} s old; newest data "
                "will be used instead.",
                throttle_duration_sec=2.0,
            )
            return
        filtered = self._copy_scan(message)
        # Gazebo's ray sensor callback precedes the matching 50 Hz odometry
        # publication. A small simulation-only offset lets TF message filters
        # wait for that transform instead of rejecting the scan as older than
        # their entire cache. Real-robot launches retain the zero default.
        stamp_offset = float(
            self.get_parameter("stamp_offset_seconds").value
        )
        if stamp_offset:
            corrected_ns = stamp_ns + int(stamp_offset * 1e9)
            filtered.header.stamp.sec = corrected_ns // 1_000_000_000
            filtered.header.stamp.nanosec = corrected_ns % 1_000_000_000
        filtered.ranges = []
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            filtered.ranges.append(
                math.inf if self._inside_robot(value, angle) else value
            )
        self._filtered_publisher.publish(filtered)

        limit = max(
            filtered.range_min,
            filtered.range_max
            - float(self.get_parameter("clearing_margin").value),
        )
        clearing = self._copy_scan(filtered)
        # Positive infinity means the beam saw no obstacle.  The paired SLAM
        # configuration has an 11.90 m raster threshold, below this synthetic
        # 11.95 m reading.  Karto therefore ray-traces it as clear but does not
        # create an occupied endpoint.  NaNs remain invalid measurements.
        clearing.ranges = [
            limit if math.isinf(value) and value > 0 else value
            for value in filtered.ranges
        ]
        self._clearing_publisher.publish(clearing)


def main(args=None):
    rclpy.init(args=args)
    node = ScanClearer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
