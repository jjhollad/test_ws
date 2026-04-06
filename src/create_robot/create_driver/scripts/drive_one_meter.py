#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DriveOneMeter(Node):
    def __init__(self):
        super().__init__("drive_one_meter")
        self.target = self.declare_parameter("target_distance", 1.0).value
        self.speed = self.declare_parameter("speed", 0.15).value
        # Proportional heading correction gain (higher = stronger/faster correction, may oscillate).
        self.kp = self.declare_parameter("heading_kp", 3.0).value
        # Max heading correction command (rad/s); caps steering to avoid over-correction.
        self.max_w = self.declare_parameter("max_angular_z", 2.0).value
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.odom_cb, 10)
        self.create_timer(0.05, self.tick)
        self.p0 = None
        self.yaw0 = None
        self.yaw = 0.0
        self.start = None
        self.d = 0.0
        self.done = False
        self.last_log_sec = -1
        self.get_logger().info(f"Driving {self.target:.2f}m at {self.speed:.2f}m/s")

    def odom_cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.start is None:
            self.p0, self.start = (p.x, p.y), self.get_clock().now()
            self.yaw0 = self.yaw
            return
        self.d = math.hypot(p.x - self.p0[0], p.y - self.p0[1])

    def tick(self):
        cmd = Twist()
        if self.done or self.start is None:
            self.pub.publish(cmd)
            return
        t = (self.get_clock().now() - self.start).nanoseconds / 1e9
        if self.d >= self.target:
            self.done = True
            self.get_logger().info(f"Done: {self.d:.3f}m in {t:.2f}s (avg {self.d/max(t,1e-6):.3f}m/s)")
            self.pub.publish(cmd)
            self.create_timer(0.5, lambda: rclpy.shutdown())
            return
        cmd.linear.x = self.speed
        err = 0.0
        if self.yaw0 is not None:
            err = math.atan2(math.sin(self.yaw0 - self.yaw), math.cos(self.yaw0 - self.yaw))
            cmd.angular.z = max(-self.max_w, min(self.max_w, self.kp * err))
        sec = int(t)
        if sec != self.last_log_sec:
            self.last_log_sec = sec
            self.get_logger().info(
                f"t={t:4.1f}s d={self.d:.3f}/{self.target:.3f}m yaw_err={err:.3f}rad wz={cmd.angular.z:.3f}"
            )
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    n = DriveOneMeter()
    try:
        rclpy.spin(n)
    finally:
        n.pub.publish(Twist())
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

