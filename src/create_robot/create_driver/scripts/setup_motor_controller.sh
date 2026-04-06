#!/bin/bash
# Setup script for Yahboom motor controller
# Sets the correct baud rate (1152000) before launching ROS2

MOTOR_DEV=${1:-/dev/ttyUSB0}

echo "Setting up Yahboom motor controller at $MOTOR_DEV..."

# Check if device exists
if [ ! -e "$MOTOR_DEV" ]; then
    echo "Error: $MOTOR_DEV not found!"
    exit 1
fi

# Set baud rate to 1152000
echo "Setting baud rate to 1152000..."
stty -F "$MOTOR_DEV" 1152000

if [ $? -eq 0 ]; then
    echo "✓ Baud rate set successfully"
    echo ""
    echo "You can now launch ROS2:"
    echo "  ros2 launch generic_motor_driver complete_robot.launch.py dev:=$MOTOR_DEV baud:=1152000"
else
    echo "✗ Failed to set baud rate"
    echo "Try: sudo stty -F $MOTOR_DEV 1152000"
    exit 1
fi










