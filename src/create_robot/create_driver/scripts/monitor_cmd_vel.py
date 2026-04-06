#!/usr/bin/env python3
"""
Monitor cmd_vel topic to verify velocity commands are being published.
Shows real-time velocity commands with timestamps.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class CmdVelMonitor(Node):
    def __init__(self):
        super().__init__('cmd_vel_monitor')
        
        self.declare_parameter('topic', '/cmd_vel')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Twist,
            topic,
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('CMD_VEL Monitor')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Monitoring topic: {topic}')
        self.get_logger().info('Waiting for messages...')
        self.get_logger().info('=' * 60)
        
        self.message_count = 0
        self.last_msg_time = None
        
    def cmd_vel_callback(self, msg):
        self.message_count += 1
        current_time = self.get_clock().now()
        
        # Calculate time since last message
        if self.last_msg_time is not None:
            dt = (current_time - self.last_msg_time).nanoseconds / 1e9
            freq = 1.0 / dt if dt > 0 else 0.0
        else:
            dt = 0.0
            freq = 0.0
        
        self.last_msg_time = current_time
        
        # Format output
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Only log if there's actual movement (non-zero velocities)
        if abs(linear) > 0.001 or abs(angular) > 0.001:
            self.get_logger().info(
                f'[{self.message_count:4d}] Linear: {linear:6.3f} m/s | '
                f'Angular: {angular:6.3f} rad/s | '
                f'Freq: {freq:5.1f} Hz'
            )
        elif self.message_count % 50 == 0:  # Log every 50th zero message
            self.get_logger().info(
                f'[{self.message_count:4d}] No movement (linear: {linear:.3f}, angular: {angular:.3f}) | '
                f'Freq: {freq:5.1f} Hz'
            )

def main(args=None):
    rclpy.init(args=args)
    
    node = CmdVelMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('=' * 60)
        node.get_logger().info(f'Total messages received: {node.message_count}')
        node.get_logger().info('=' * 60)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





