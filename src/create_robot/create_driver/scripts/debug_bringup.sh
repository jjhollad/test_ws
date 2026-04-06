#!/bin/bash
# Debug script for complete robot bringup

echo "=========================================="
echo "Complete Robot Bringup Debugging Script"
echo "=========================================="
echo ""

# 1. Check serial devices
echo "1. Checking Serial Devices:"
echo "---------------------------"
echo "Available USB devices:"
ls -la /dev/ttyUSB* 2>/dev/null || echo "  No ttyUSB devices found"
ls -la /dev/ttyACM* 2>/dev/null || echo "  No ttyACM devices found"
echo ""

# 2. Check device permissions
echo "2. Checking Device Permissions:"
echo "-------------------------------"
if [ -e "/dev/ttyUSB0" ]; then
    ls -la /dev/ttyUSB0
    echo "  Motor controller: /dev/ttyUSB0"
else
    echo "  Motor controller: /dev/ttyUSB0 NOT FOUND"
fi

if [ -e "/dev/ttyACM0" ]; then
    ls -la /dev/ttyACM0
    echo "  Relay controller: /dev/ttyACM0"
else
    echo "  Relay controller: /dev/ttyACM0 NOT FOUND"
fi
echo ""

# 3. Check ROS2 nodes
echo "3. Checking ROS2 Nodes:"
echo "-----------------------"
ros2 node list 2>/dev/null || echo "  No ROS2 nodes running (or ROS2 not sourced)"
echo ""

# 4. Check ROS2 topics
echo "4. Checking ROS2 Topics:"
echo "------------------------"
ros2 topic list 2>/dev/null || echo "  No topics available (or ROS2 not sourced)"
echo ""

# 5. Check motor driver topics
echo "5. Checking Motor Driver Topics:"
echo "--------------------------------"
echo "cmd_vel:"
timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null || echo "  No messages on /cmd_vel"
echo ""
echo "joint_states:"
timeout 1 ros2 topic echo /joint_states --once 2>/dev/null || echo "  No messages on /joint_states"
echo ""
echo "odom:"
timeout 1 ros2 topic echo /odom --once 2>/dev/null || echo "  No messages on /odom"
echo ""

# 6. Check relay controller topics
echo "6. Checking Relay Controller Topics:"
echo "-------------------------------------"
echo "relay_status:"
timeout 1 ros2 topic echo /relay_status --once 2>/dev/null || echo "  No messages on /relay_status"
echo ""

# 7. Check TF tree
echo "7. Checking TF Tree:"
echo "--------------------"
ros2 run tf2_tools view_frames 2>/dev/null
if [ -f "frames.pdf" ]; then
    echo "  TF tree saved to frames.pdf"
    rm frames.pdf 2>/dev/null
fi
echo ""

# 8. Check node info
echo "8. Checking Node Information:"
echo "------------------------------"
echo "Motor Driver:"
ros2 node info /generic_motor_driver 2>/dev/null || echo "  Motor driver node not running"
echo ""
echo "Relay Controller:"
ros2 node info /relay_controller 2>/dev/null || echo "  Relay controller node not running"
echo ""

# 9. Check parameters
echo "9. Checking Parameters:"
echo "-----------------------"
echo "Motor Driver Parameters:"
ros2 param list /generic_motor_driver 2>/dev/null || echo "  Motor driver node not running"
echo ""
echo "Relay Controller Parameters:"
ros2 param list /relay_controller 2>/dev/null || echo "  Relay controller node not running"
echo ""

# 10. Test serial communication
echo "10. Testing Serial Communication:"
echo "----------------------------------"
echo "Testing motor controller (sending stop command):"
echo '$spd:0,0,0,0#' > /dev/ttyUSB0 2>/dev/null && echo "  Command sent to /dev/ttyUSB0" || echo "  Failed to send to /dev/ttyUSB0"
echo ""

echo "=========================================="
echo "Debugging Complete"
echo "=========================================="



