#!/bin/bash
# Fix script for Yahboom motor controller after LiDAR connection

echo "=========================================="
echo "Fixing Motor Controller Connection"
echo "=========================================="
echo ""

# Check if Arduino serial monitor is running
if pgrep -f "serial-monitor" > /dev/null; then
    echo "Found Arduino Serial Monitor running - this is blocking the motor controller!"
    echo ""
    read -p "Kill Arduino Serial Monitor? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pkill -f "serial-monitor"
        pkill -f "serial-discovery"
        sleep 1
        echo "✓ Arduino Serial Monitor stopped"
    else
        echo "Please close Arduino IDE Serial Monitor manually"
    fi
    echo ""
fi

# Check if device is still locked
if lsof /dev/ttyUSB0 2>/dev/null | grep -q .; then
    echo "WARNING: /dev/ttyUSB0 is still in use:"
    lsof /dev/ttyUSB0
    echo ""
    echo "To free it, you can:"
    echo "  1. Close Arduino IDE Serial Monitor"
    echo "  2. Kill the process: sudo kill <PID>"
    echo "  3. Or restart the system"
else
    echo "✓ /dev/ttyUSB0 is available"
fi

echo ""
echo "Testing motor controller communication..."
python3 /home/user/test_ws/src/create_robot/create_driver/scripts/test_motor_controller.py

echo ""
echo "If motor controller is working, you can now launch ROS2:"
echo "  ros2 launch generic_motor_driver complete_robot.launch.py dev:=/dev/ttyUSB0"






