#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class NavCmdPathMonitor(Node):
    def __init__(self):
        super().__init__('nav_cmd_path_monitor')
        self.samples = {
            '/cmd_vel_nav': None,
            '/cmd_vel': None,
        }
        self.create_subscription(Twist, '/cmd_vel_nav', self._callback('/cmd_vel_nav'), 10)
        self.create_subscription(Twist, '/cmd_vel', self._callback('/cmd_vel'), 10)
        self.create_timer(1.0, self._report)

    def _callback(self, topic):
        def cb(msg):
            self.samples[topic] = (self.get_clock().now(), msg.linear.x, msg.angular.z)
        return cb

    def _format(self, topic):
        sample = self.samples[topic]
        if sample is None:
            return f'{topic}: no messages'

        stamp, linear_x, angular_z = sample
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        moving = math.fabs(linear_x) > 1e-3 or math.fabs(angular_z) > 1e-3
        state = 'nonzero' if moving else 'zero'
        return f'{topic}: {state} lin.x={linear_x:.3f} ang.z={angular_z:.3f} age={age:.1f}s'

    def _report(self):
        self.get_logger().info(f'{self._format("/cmd_vel_nav")} | {self._format("/cmd_vel")}')


def main():
    rclpy.init()
    node = NavCmdPathMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
