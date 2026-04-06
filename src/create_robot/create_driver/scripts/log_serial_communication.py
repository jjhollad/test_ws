#!/usr/bin/env python3
"""
Log serial communication between ROS2 driver and motor controller.

Subscribes to:
- /motor_speeds: SPD values being sent (Int32MultiArray)
- /serial_tx: Raw commands sent over serial (String)
- /serial_rx: Raw data received from serial (String)

Logs them side-by-side with timestamps to a file.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import sys
import os
from datetime import datetime


class SerialCommunicationLogger(Node):
    def __init__(self):
        super().__init__("serial_comm_logger")
        
        # Parameters
        self.declare_parameter("log_file", "motor_serial_comm.log")
        self.declare_parameter("motor_speeds_topic", "/motor_speeds")
        self.declare_parameter("serial_tx_topic", "/serial_tx")
        self.declare_parameter("serial_rx_topic", "/serial_rx")
        
        log_file = self.get_parameter("log_file").get_parameter_value().string_value
        motor_speeds_topic = self.get_parameter("motor_speeds_topic").get_parameter_value().string_value
        serial_tx_topic = self.get_parameter("serial_tx_topic").get_parameter_value().string_value
        serial_rx_topic = self.get_parameter("serial_rx_topic").get_parameter_value().string_value
        
        # Open log file
        self.log_file_path = log_file
        self.log_file = open(self.log_file_path, "w")
        self.log_file.write("# Motor Serial Communication Log\n")
        self.log_file.write(f"# Started: {datetime.now().isoformat()}\n")
        self.log_file.write("# Format: TIMESTAMP | TX_COMMAND | RX_DATA | SPD_VALUES\n")
        self.log_file.write("#" + "=" * 100 + "\n\n")
        self.log_file.flush()
        
        # Track latest values
        self.latest_spd = None
        self.latest_tx = None
        self.latest_rx = None
        self.last_log_time = None
        
        # Subscriptions
        self.create_subscription(
            Int32MultiArray,
            motor_speeds_topic,
            self.motor_speeds_callback,
            10
        )
        
        self.create_subscription(
            String,
            serial_tx_topic,
            self.serial_tx_callback,
            10
        )
        
        self.create_subscription(
            String,
            serial_rx_topic,
            self.serial_rx_callback,
            10
        )
        
        self.get_logger().info("=" * 80)
        self.get_logger().info("Serial Communication Logger")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"Logging to: {self.log_file_path}")
        self.get_logger().info(f"Monitoring:")
        self.get_logger().info(f"  - Motor speeds: {motor_speeds_topic}")
        self.get_logger().info(f"  - Serial TX: {serial_tx_topic}")
        self.get_logger().info(f"  - Serial RX: {serial_rx_topic}")
        self.get_logger().info("Press Ctrl+C to stop and close log file")
        self.get_logger().info("=" * 80)
        
    def motor_speeds_callback(self, msg):
        """Callback for motor speeds (SPD values)."""
        if len(msg.data) >= 2:
            self.latest_spd = (int(msg.data[0]), int(msg.data[1]))
            self._log_line()
    
    def serial_tx_callback(self, msg):
        """Callback for serial commands sent."""
        self.latest_tx = msg.data.strip()
        self._log_line()
    
    def serial_rx_callback(self, msg):
        """Callback for serial data received."""
        self.latest_rx = msg.data.strip()
        self._log_line()
    
    def _log_line(self):
        """Log a line with current values."""
        now = self.get_clock().now()
        timestamp = now.nanoseconds / 1e9  # Convert to seconds
        
        # Format values
        spd_str = f"M1={self.latest_spd[0]:6d},M2={self.latest_spd[1]:6d}" if self.latest_spd else "N/A"
        tx_str = self.latest_tx if self.latest_tx else "N/A"
        rx_str = self.latest_rx if self.latest_rx else "N/A"
        
        # Write to log file
        log_line = f"{timestamp:15.6f} | TX: {tx_str:30s} | RX: {rx_str:30s} | SPD: {spd_str}\n"
        self.log_file.write(log_line)
        self.log_file.flush()  # Ensure immediate write
        
        # Also print to console occasionally (every 10th message to avoid spam)
        if self.last_log_time is None or (timestamp - self.last_log_time) > 1.0:
            self.get_logger().info(f"Logged: TX={tx_str[:30]} | RX={rx_str[:30]} | SPD={spd_str}")
            self.last_log_time = timestamp
    
    def __del__(self):
        """Close log file on destruction."""
        if hasattr(self, 'log_file') and self.log_file:
            self.log_file.write(f"\n# Log ended: {datetime.now().isoformat()}\n")
            self.log_file.close()


def main(args=None):
    rclpy.init(args=args)
    
    node = SerialCommunicationLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nShutting down logger...")
    finally:
        if hasattr(node, 'log_file') and node.log_file:
            node.log_file.write(f"\n# Log ended: {datetime.now().isoformat()}\n")
            node.log_file.close()
            node.get_logger().info(f"Log file closed: {node.log_file_path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
