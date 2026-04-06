#!/usr/bin/env python3
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class LeftWheelCmdVelTest(Node):
    def __init__(self):
        super().__init__("left_wheel_cmdvel_test")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.wheel_base = self.declare_parameter("wheel_base", 0.61).value

    def run(self, left_wheel_speed=0.2, duration=3.0):
        # Differential-drive mapping:
        # v_l = linear + angular*base/2, v_r = linear - angular*base/2
        # Set v_l=left_wheel_speed and v_r=0.
        cmd = Twist()
        cmd.linear.x = left_wheel_speed / 2.0
        cmd.angular.z = left_wheel_speed / self.wheel_base
        stop = Twist()

        self.get_logger().info(
            f"Publishing LEFT-only cmd_vel for {duration:.2f}s "
            f"(v_l={left_wheel_speed:.3f} m/s, v_r=0)"
        )
        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(cmd)
            time.sleep(0.05)

        for _ in range(8):
            self.pub.publish(stop)
            time.sleep(0.05)
        self.get_logger().info("Done.")


def main(args=None):
    rclpy.init(args=args)
    node = LeftWheelCmdVelTest()
    try:
        v = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
        t = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
        node.run(v, t)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
