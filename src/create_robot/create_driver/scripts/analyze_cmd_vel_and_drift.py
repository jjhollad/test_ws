#!/usr/bin/env python3
"""
Analyze /cmd_vel stream quality and idle drift indicators.

What it reports periodically:
- message rate stats (avg/min/max/std)
- max inter-message gap and timeout risk
- percentage of exact-zero vs tiny non-zero commands
- bias direction (forward/backward, clockwise/counter-clockwise)
- optional odom drift while commands are near zero
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CmdVelDriftAnalyzer(Node):
    def __init__(self):
        super().__init__('cmd_vel_drift_analyzer')

        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('use_odom', True)
        self.declare_parameter('report_period_sec', 5.0)
        self.declare_parameter('window_size', 4000)
        self.declare_parameter('cmd_zero_epsilon', 1e-3)
        self.declare_parameter('cmd_tiny_epsilon', 2e-2)
        self.declare_parameter('odom_zero_linear', 5e-3)
        self.declare_parameter('odom_zero_angular', 1e-2)
        self.declare_parameter('timeout_suspect_gap_sec', 0.25)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.use_odom = bool(self.get_parameter('use_odom').value)
        self.report_period = float(self.get_parameter('report_period_sec').value)
        self.window_size = int(self.get_parameter('window_size').value)
        self.cmd_zero_eps = float(self.get_parameter('cmd_zero_epsilon').value)
        self.cmd_tiny_eps = float(self.get_parameter('cmd_tiny_epsilon').value)
        self.odom_zero_lin = float(self.get_parameter('odom_zero_linear').value)
        self.odom_zero_ang = float(self.get_parameter('odom_zero_angular').value)
        self.timeout_gap = float(self.get_parameter('timeout_suspect_gap_sec').value)

        self.dt_window = deque(maxlen=self.window_size)
        self.lin_window = deque(maxlen=self.window_size)
        self.ang_window = deque(maxlen=self.window_size)
        self.last_cmd_time_ns = None
        self.max_gap = 0.0
        self.total_cmd_msgs = 0

        self.idle_odom_lin = deque(maxlen=self.window_size)
        self.idle_odom_ang = deque(maxlen=self.window_size)
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0

        self.create_subscription(Twist, self.cmd_topic, self._cmd_cb, 50)
        if self.use_odom:
            self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 50)

        self.timer = self.create_timer(self.report_period, self._report)

        self.get_logger().info('=' * 72)
        self.get_logger().info('CMD_VEL + Drift Analyzer')
        self.get_logger().info(f'cmd_topic={self.cmd_topic} | use_odom={self.use_odom} | odom_topic={self.odom_topic}')
        self.get_logger().info(
            f'eps: cmd_zero={self.cmd_zero_eps}, cmd_tiny={self.cmd_tiny_eps}, '
            f'odom_zero_lin={self.odom_zero_lin}, odom_zero_ang={self.odom_zero_ang}'
        )
        self.get_logger().info('=' * 72)

    def _cmd_cb(self, msg: Twist):
        now_ns = self.get_clock().now().nanoseconds
        self.total_cmd_msgs += 1

        if self.last_cmd_time_ns is not None:
            dt = (now_ns - self.last_cmd_time_ns) / 1e9
            if dt > 0.0:
                self.dt_window.append(dt)
                if dt > self.max_gap:
                    self.max_gap = dt
        self.last_cmd_time_ns = now_ns

        self.last_cmd_linear = msg.linear.x
        self.last_cmd_angular = msg.angular.z
        self.lin_window.append(self.last_cmd_linear)
        self.ang_window.append(self.last_cmd_angular)

    def _odom_cb(self, msg: Odometry):
        # Only collect odom "idle drift" samples when command is near zero.
        if abs(self.last_cmd_linear) <= self.cmd_zero_eps and abs(self.last_cmd_angular) <= self.cmd_zero_eps:
            self.idle_odom_lin.append(msg.twist.twist.linear.x)
            self.idle_odom_ang.append(msg.twist.twist.angular.z)

    @staticmethod
    def _mean(values):
        if not values:
            return 0.0
        return sum(values) / len(values)

    @staticmethod
    def _std(values, mean):
        if len(values) < 2:
            return 0.0
        var = sum((v - mean) ** 2 for v in values) / (len(values) - 1)
        return math.sqrt(var)

    def _report(self):
        n = len(self.dt_window)
        if n == 0:
            self.get_logger().info('Waiting for enough /cmd_vel messages to compute stats...')
            return

        hz_values = [1.0 / dt for dt in self.dt_window if dt > 0.0]
        hz_mean = self._mean(hz_values)
        hz_min = min(hz_values) if hz_values else 0.0
        hz_max = max(hz_values) if hz_values else 0.0
        hz_std = self._std(hz_values, hz_mean) if hz_values else 0.0

        lin = list(self.lin_window)
        ang = list(self.ang_window)
        m = len(lin)
        near_zero_count = sum(
            1 for i in range(m)
            if abs(lin[i]) <= self.cmd_zero_eps and abs(ang[i]) <= self.cmd_zero_eps
        )
        tiny_nonzero_count = sum(
            1 for i in range(m)
            if (abs(lin[i]) <= self.cmd_tiny_eps and abs(ang[i]) <= self.cmd_tiny_eps)
            and not (abs(lin[i]) <= self.cmd_zero_eps and abs(ang[i]) <= self.cmd_zero_eps)
        )
        lin_mean = self._mean(lin)
        ang_mean = self._mean(ang)

        backward_bias = lin_mean < -self.cmd_zero_eps
        clockwise_bias = ang_mean < -self.cmd_zero_eps
        timeout_risk = self.max_gap >= self.timeout_gap

        msg = []
        msg.append('-' * 72)
        msg.append(f'/cmd_vel msgs={self.total_cmd_msgs} | window={m}')
        msg.append(
            f'rate avg={hz_mean:.2f}Hz min={hz_min:.2f} max={hz_max:.2f} std={hz_std:.2f} | max_gap={self.max_gap:.3f}s'
        )
        msg.append(
            f'near_zero={100.0*near_zero_count/max(1,m):.1f}% | tiny_nonzero={100.0*tiny_nonzero_count/max(1,m):.1f}%'
        )
        msg.append(f'cmd mean: linear.x={lin_mean:+.5f} m/s | angular.z={ang_mean:+.5f} rad/s')
        msg.append(
            'bias flags: '
            f'backward_bias={backward_bias} | clockwise_bias={clockwise_bias} | timeout_risk={timeout_risk}'
        )

        if self.use_odom and len(self.idle_odom_lin) > 20:
            idle_lin_mean = self._mean(self.idle_odom_lin)
            idle_ang_mean = self._mean(self.idle_odom_ang)
            idle_lin_flag = abs(idle_lin_mean) > self.odom_zero_lin
            idle_ang_flag = abs(idle_ang_mean) > self.odom_zero_ang
            msg.append(
                f'idle odom mean: linear.x={idle_lin_mean:+.5f} m/s | angular.z={idle_ang_mean:+.5f} rad/s'
            )
            msg.append(
                f'idle drift flags: linear={idle_lin_flag} (>{self.odom_zero_lin}) | '
                f'angular={idle_ang_flag} (>{self.odom_zero_ang})'
            )

        self.get_logger().info('\n'.join(msg))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDriftAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down cmd_vel drift analyzer.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
