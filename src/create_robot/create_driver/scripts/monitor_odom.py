#!/usr/bin/env python3
"""
Monitor odometry messages to check calibration of linear positions.

This script subscribes to the /odom topic and displays:
- Position (x, y, theta)
- Linear and angular velocities
- Useful for calibrating wheel positions and odometry accuracy
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


class OdometryMonitor(Node):
    def __init__(self):
        super().__init__('odom_monitor')
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odometry Monitor started. Listening to /odom topic...')
        self.get_logger().info('Press Ctrl+C to stop.')
        
        # Track previous position for distance calculation
        self.last_x = None
        self.last_y = None
        self.total_distance = 0.0
        
    def odom_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract orientation (quaternion to yaw)
        orientation = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(orientation)
        
        # Extract velocities
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        
        # Calculate distance traveled since last update
        if self.last_x is not None and self.last_y is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance += distance
        else:
            distance = 0.0
        
        self.last_x = x
        self.last_y = y
        
        # Display information
        print(f"\n{'='*80}")
        print(f"ODOMETRY DATA")
        print(f"{'='*80}")
        print(f"Position:")
        print(f"  X:        {x:10.6f} m")
        print(f"  Y:        {y:10.6f} m")
        print(f"  Theta:    {math.degrees(yaw):10.4f} deg  ({yaw:10.6f} rad)")
        print(f"")
        print(f"Velocities:")
        print(f"  Linear:   {linear_vel:10.6f} m/s")
        print(f"  Angular: {angular_vel:10.6f} rad/s  ({math.degrees(angular_vel):10.4f} deg/s)")
        print(f"")
        print(f"Distance Traveled:")
        print(f"  Since start: {self.total_distance:10.6f} m")
        print(f"  Last step:   {distance:10.6f} m")
        print(f"{'='*80}")
        
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle (rotation around z-axis)."""
        # Extract quaternion components
        w = quaternion.w
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        
        # Convert to yaw (rotation around z-axis)
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw


def main(args=None):
    rclpy.init(args=args)
    
    monitor = OdometryMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\nStopping odometry monitor...")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





