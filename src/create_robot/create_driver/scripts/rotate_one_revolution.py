#!/usr/bin/env python3
"""
Rotate robot exactly 360 degrees (2π radians) using odometry data for rotation calibration.
This script reads odometry, tracks rotation, and stops when 360 degrees is reached.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf_transformations

class RotateOneRevolution(Node):
    def __init__(self):
        super().__init__('rotate_one_revolution')
        
        # Get parameters
        self.declare_parameter('target_rotation', 2.0 * math.pi)  # 360 degrees in radians
        self.declare_parameter('angular_speed', 0.5)  # rad/s
        
        self.target_rotation_ = self.get_parameter('target_rotation').get_parameter_value().double_value
        self.angular_speed_ = self.get_parameter('angular_speed').get_parameter_value().double_value
        
        # State variables
        self.initial_theta_ = None
        self.rotation_traveled_ = 0.0  # in radians
        self.start_time_ = None
        self.odom_initialized_ = False
        self.last_log_time_ = None
        
        # Publishers and subscribers
        self.cmd_vel_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub_ = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Timer to send velocity commands
        self.timer_ = self.create_timer(0.1, self.update)  # 10 Hz
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Rotate One Revolution Calibration Script')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Target rotation: {math.degrees(self.target_rotation_):.1f} degrees ({self.target_rotation_:.4f} radians)')
        self.get_logger().info(f'Angular speed: {self.angular_speed_} rad/s ({math.degrees(self.angular_speed_):.1f} deg/s)')
        self.get_logger().info('Waiting for odometry data...')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Mark the robot orientation before starting!')
        self.get_logger().info('=' * 60)
        
    def odom_callback(self, msg):
        """Callback to receive odometry data"""
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        # Normalize yaw to [-π, π]
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        
        # Initialize on first reading
        if not self.odom_initialized_:
            self.initial_theta_ = yaw
            self.odom_initialized_ = True
            self.start_time_ = self.get_clock().now()
            self.last_log_time_ = self.get_clock().now()
            self.get_logger().info('Odometry data received! Starting rotation...')
            self.get_logger().info(f'Initial orientation: {math.degrees(self.initial_theta_):.1f} degrees ({self.initial_theta_:.4f} radians)')
            return
        
        # Calculate rotation traveled
        # Handle wrap-around at ±π boundary
        delta_theta = yaw - self.initial_theta_
        
        # Normalize to [-π, π]
        if delta_theta > math.pi:
            delta_theta -= 2.0 * math.pi
        elif delta_theta < -math.pi:
            delta_theta += 2.0 * math.pi
        
        self.rotation_traveled_ = delta_theta
        
    def update(self):
        """Main update loop - sends velocity commands and checks rotation"""
        if not self.odom_initialized_:
            return
        
        # Check if target rotation reached
        if abs(self.rotation_traveled_) >= abs(self.target_rotation_):
            # Stop the robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub_.publish(cmd)
            
            # Log results
            elapsed_time = (self.get_clock().now() - self.start_time_).nanoseconds / 1e9
            rotation_degrees = math.degrees(self.rotation_traveled_)
            target_degrees = math.degrees(self.target_rotation_)
            
            self.get_logger().info('=' * 60)
            self.get_logger().info('ROTATION COMPLETE!')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'Target rotation: {target_degrees:.1f} degrees ({self.target_rotation_:.4f} radians)')
            self.get_logger().info(f'Rotation traveled (odom): {rotation_degrees:.1f} degrees ({self.rotation_traveled_:.4f} radians)')
            self.get_logger().info(f'Time elapsed: {elapsed_time:.2f} seconds')
            self.get_logger().info(f'Average angular speed: {rotation_degrees/elapsed_time:.2f} deg/s')
            self.get_logger().info('=' * 60)
            self.get_logger().info('Now measure the actual rotation the robot turned!')
            self.get_logger().info('Compare odom rotation vs real rotation to calibrate rotation_scale.')
            self.get_logger().info('=' * 60)
            
            # Shutdown after a delay
            self.create_timer(2.0, lambda: rclpy.shutdown())
            return
        
        # Determine rotation direction
        rotation_direction = 1.0 if self.target_rotation_ >= 0 else -1.0
        
        # Send rotation command (in place - no linear velocity)
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = rotation_direction * self.angular_speed_
        self.cmd_vel_pub_.publish(cmd)
        
        # Log progress with percentage every 0.5 seconds
        rotation_degrees = math.degrees(self.rotation_traveled_)
        target_degrees = math.degrees(self.target_rotation_)
        percentage = (abs(self.rotation_traveled_) / abs(self.target_rotation_)) * 100.0
        
        elapsed_since_log = (self.get_clock().now() - self.last_log_time_).nanoseconds / 1e9
        if elapsed_since_log >= 0.5:  # Log every 0.5 seconds
            self.get_logger().info(f'[{percentage:5.1f}%] Rotation: {rotation_degrees:7.1f}° / {target_degrees:.1f}° '
                                  f'({self.rotation_traveled_:.4f} rad / {self.target_rotation_:.4f} rad)')
            self.last_log_time_ = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    
    node = RotateOneRevolution()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot before shutdown
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.cmd_vel_pub_.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
