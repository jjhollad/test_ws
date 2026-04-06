#!/usr/bin/env python3
"""
Script to rotate a wheel exactly 360 degrees (one full rotation).
This helps compare the real robot wheel rotation with RViz visualization.

Usage:
    ros2 run generic_motor_driver rotate_wheel_one_turn.py [left|right] [duration]
    
    left|right: Which wheel to rotate (default: left)
    duration: Time in seconds to complete one rotation (default: 2.0)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time

class WheelRotator(Node):
    def __init__(self):
        super().__init__('wheel_rotator')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Robot parameters (match your launch file)
        # These should match the values in complete_robot.launch.py
        self.wheel_radius = 0.12  # meters (default from launch file)
        self.wheel_base = 0.57    # meters (default from launch file)
        
        # Calculate angular velocity for one rotation
        # For one full rotation: distance = 2 * PI * radius
        # linear_velocity = distance / time
        # For pure rotation (one wheel): angular_vel = linear_vel / radius
        
    def rotate_wheel(self, side='left', duration=2.0):
        """
        Rotate one wheel exactly 360 degrees.
        
        Args:
            side: 'left' or 'right'
            duration: Time in seconds to complete one rotation
        """
        # Calculate linear velocity needed for one rotation in given time
        # One rotation = 2 * PI * radius meters
        distance_per_rotation = 2.0 * 3.141592653589793 * self.wheel_radius
        linear_vel = distance_per_rotation / duration
        
        # Calculate angular velocity to rotate only one wheel
        # For differential drive: v_left = linear - angular*base/2
        #                        v_right = linear + angular*base/2
        # To rotate only left wheel: v_left = linear_vel, v_right = 0
        # To rotate only right wheel: v_left = 0, v_right = linear_vel
        
        twist = Twist()
        
        if side == 'left':
            # Rotate left wheel: v_left = linear_vel, v_right = 0
            # linear = (v_left + v_right) / 2 = linear_vel / 2
            # angular = (v_left - v_right) / base = linear_vel / base
            twist.linear.x = linear_vel / 2.0
            twist.angular.z = linear_vel / self.wheel_base
            self.get_logger().info(f'Rotating LEFT wheel one full rotation in {duration}s')
            self.get_logger().info(f'  Linear velocity: {twist.linear.x:.3f} m/s')
            self.get_logger().info(f'  Angular velocity: {twist.angular.z:.3f} rad/s')
        else:
            # Rotate right wheel: v_left = 0, v_right = linear_vel
            # linear = (v_left + v_right) / 2 = linear_vel / 2
            # angular = (v_left - v_right) / base = -linear_vel / base
            twist.linear.x = linear_vel / 2.0
            twist.angular.z = -linear_vel / self.wheel_base
            self.get_logger().info(f'Rotating RIGHT wheel one full rotation in {duration}s')
            self.get_logger().info(f'  Linear velocity: {twist.linear.x:.3f} m/s')
            self.get_logger().info(f'  Angular velocity: {twist.angular.z:.3f} rad/s')
        
        self.get_logger().info(f'  Wheel radius: {self.wheel_radius} m')
        self.get_logger().info(f'  Distance per rotation: {distance_per_rotation:.4f} m')
        self.get_logger().info(f'  Publishing cmd_vel for {duration} seconds...')
        
        # Publish command for the specified duration
        start_time = time.time()
        rate = self.create_rate(20)  # 20 Hz
        
        interrupted = False
        try:
            while (time.time() - start_time) < duration:
                try:
                    self.publisher.publish(twist)
                    rate.sleep()
                except KeyboardInterrupt:
                    interrupted = True
                    break
                except Exception as e:
                    self.get_logger().warn(f'Error publishing: {e}')
                    break
        except KeyboardInterrupt:
            interrupted = True
        finally:
            # Stop the robot (publish multiple times to ensure it's received)
            if interrupted:
                self.get_logger().info('Interrupted. Stopping robot...')
            else:
                self.get_logger().info('Rotation complete. Stopping robot...')
            
            stop_twist = Twist()
            try:
                for _ in range(5):
                    try:
                        self.publisher.publish(stop_twist)
                        time.sleep(0.1)
                    except (KeyboardInterrupt, Exception):
                        # Context may be invalid, just break
                        break
            except (KeyboardInterrupt, Exception):
                pass

def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    side = 'left'
    duration = 2.0
    
    if len(sys.argv) > 1:
        side = sys.argv[1].lower()
        if side not in ['left', 'right']:
            print(f"Error: Side must be 'left' or 'right', got '{side}'")
            print("Usage: ros2 run generic_motor_driver rotate_wheel_one_turn.py [left|right] [duration]")
            return
    
    if len(sys.argv) > 2:
        try:
            duration = float(sys.argv[2])
        except ValueError:
            print(f"Error: Duration must be a number, got '{sys.argv[2]}'")
            return
    
    rotator = WheelRotator()
    
    try:
        rotator.rotate_wheel(side, duration)
    except KeyboardInterrupt:
        pass  # Already handled in rotate_wheel
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean shutdown
        try:
            rotator.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()





