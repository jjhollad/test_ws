#!/usr/bin/env python3
"""Record compact navigation trial metrics as CSV."""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class TrialMonitor(Node):
    def __init__(self):
        super().__init__("trial_monitor")
        self.declare_parameter("output", "/tmp/navigation_trial.csv")
        self.declare_parameter("period", 60.0)
        self._output = str(self.get_parameter("output").value)
        self._start = self.get_clock().now()
        self._pose = None
        self._start_pose = None
        self._last_pose = None
        self._travel = 0.0
        self._free_area = 0.0
        self._occupied_area = 0.0
        self._wall_active = False
        self._wall_seconds = 0.0
        self._last_tick = self._start
        with open(self._output, "w", encoding="utf-8") as stream:
            stream.write("seconds,x,y,net_m,travel_m,free_m2,occupied_m2,wall_seconds\n")
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(OccupancyGrid, "/map", self._map, 10)
        self.create_subscription(Bool, "/wall_tracing_active", self._wall, 10)
        self.create_timer(float(self.get_parameter("period").value), self._record)

    def _odom(self, message):
        pose = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self._start_pose is None:
            self._start_pose = pose
        if self._last_pose is not None:
            step = math.hypot(pose[0] - self._last_pose[0], pose[1] - self._last_pose[1])
            if step < 0.25:
                self._travel += step
        self._pose, self._last_pose = pose, pose

    def _map(self, message):
        values = np.asarray(message.data, dtype=np.int16)
        scale = message.info.resolution ** 2
        self._free_area = np.count_nonzero((values >= 0) & (values <= 20)) * scale
        self._occupied_area = np.count_nonzero(values > 20) * scale

    def _wall(self, message):
        self._wall_active = message.data

    def _record(self):
        now = self.get_clock().now()
        elapsed_tick = (now - self._last_tick).nanoseconds / 1e9
        if self._wall_active:
            self._wall_seconds += elapsed_tick
        self._last_tick = now
        if self._pose is None or self._start_pose is None:
            return
        elapsed = (now - self._start).nanoseconds / 1e9
        net = math.hypot(
            self._pose[0] - self._start_pose[0],
            self._pose[1] - self._start_pose[1],
        )
        with open(self._output, "a", encoding="utf-8") as stream:
            stream.write(
                f"{elapsed:.1f},{self._pose[0]:.3f},{self._pose[1]:.3f},"
                f"{net:.3f},{self._travel:.3f},{self._free_area:.3f},"
                f"{self._occupied_area:.3f},{self._wall_seconds:.1f}\n"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TrialMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
