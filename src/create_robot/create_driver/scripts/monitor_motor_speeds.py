#!/usr/bin/env python3
"""
Monitor motor speed values sent over serial by generic_motor_driver.

This listens to the /motor_speeds topic published by the driver. Those values are the
exact integers used in the $spd:M1,M2,0,0# command.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class MotorSpeedsMonitor(Node):
    def __init__(self):
        super().__init__("motor_speeds_monitor")

        self.declare_parameter("topic", "/motor_speeds")
        topic = self.get_parameter("topic").get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Int32MultiArray,
            topic,
            self._callback,
            10,
        )

        self.message_count = 0
        self.last_msg_time = None

        self.get_logger().info("=" * 60)
        self.get_logger().info("MOTOR SPEEDS Monitor")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Monitoring topic: {topic}")
        self.get_logger().info("Expected payload: data=[M1, M2] (ints sent in $spd:...)")
        self.get_logger().info("Waiting for messages...")
        self.get_logger().info("=" * 60)

    def _callback(self, msg: Int32MultiArray):
        self.message_count += 1
        now = self.get_clock().now()

        if self.last_msg_time is not None:
            dt = (now - self.last_msg_time).nanoseconds / 1e9
            freq = 1.0 / dt if dt > 0 else 0.0
        else:
            freq = 0.0

        self.last_msg_time = now

        if len(msg.data) < 2:
            self.get_logger().warn(f"[{self.message_count:4d}] Bad message: data has {len(msg.data)} elements")
            return

        m1 = int(msg.data[0])
        m2 = int(msg.data[1])

        # Log non-zero commands; otherwise sample occasionally.
        if abs(m1) > 0 or abs(m2) > 0:
            self.get_logger().info(
                f"[{self.message_count:4d}] $spd m1={m1:6d}, m2={m2:6d} | Freq: {freq:5.1f} Hz"
            )
        elif self.message_count % 50 == 0:
            self.get_logger().info(f"[{self.message_count:4d}] $spd m1=0, m2=0 | Freq: {freq:5.1f} Hz")


def main(args=None):
    rclpy.init(args=args)
    node = MotorSpeedsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("=" * 60)
        node.get_logger().info(f"Total messages received: {node.message_count}")
        node.get_logger().info("=" * 60)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

