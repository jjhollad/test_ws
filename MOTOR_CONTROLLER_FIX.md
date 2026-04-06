# Motor Controller Fix Guide

## Problem
After connecting LiDAR, motor controller stopped responding. Port assignments can change when devices are unplugged/replugged.

## Solution: Create udev Rules

### Step 1: Install udev rules

```bash
sudo cp /tmp/99-motor-controller.rules /etc/udev/rules.d/99-motor-controller.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Step 2: Unplug and replug USB devices

After replugging, you should see:
- `/dev/motor_controller` → Motor controller (always)
- `/dev/lidar` → LiDAR (always)

Check with:
```bash
ls -la /dev/motor_controller /dev/lidar
```

### Step 3: Update launch file

Use the consistent port name:
```bash
ros2 launch generic_motor_driver complete_robot.launch.py dev:=/dev/motor_controller
```

## If Motor Controller Still Doesn't Respond

### Check 1: Power Issues
- LiDAR may be drawing too much current
- Try: Unplug LiDAR, test motor controller alone
- Use powered USB hub if needed
- Check motor controller power LED

### Check 2: Connection Issues
- Verify USB cable is data-capable (not power-only)
- Try different USB port on computer
- Check wiring connections

### Check 3: Test Communication
```bash
# Simple test
python3 /home/user/test_ws/src/create_robot/create_driver/scripts/test_motor_controller.py

# Advanced test (tries multiple baud rates)
python3 /home/user/test_ws/src/create_robot/create_driver/scripts/test_motor_advanced.py
```

### Check 4: Verify Device
```bash
# Check if device is accessible
ls -la /dev/motor_controller

# Check if anything is using it
lsof /dev/motor_controller

# Try sending command directly
echo '$spd:0,0,0,0#' > /dev/motor_controller
```

## Troubleshooting Commands

```bash
# List all USB devices
lsusb

# Check serial devices
ls -la /dev/ttyUSB* /dev/ttyACM*

# Check device info
udevadm info -q property -n /dev/ttyUSB0

# Check ROS2 motor driver
ros2 node list | grep motor
ros2 param get /generic_motor_driver dev
```










