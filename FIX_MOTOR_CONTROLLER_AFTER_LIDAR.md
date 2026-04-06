# Fix Motor Controller After LiDAR Connection

## Current Status
- Motor controller: `/dev/ttyUSB0` (CH340, vendor 1a86)
- LiDAR: `/dev/ttyUSB1` (CP2102, vendor 10c4)
- Arduino: `/dev/ttyACM0`

## Problem
Ports can swap when devices are unplugged/replugged, causing the motor controller to move to a different port.

## Solution: Install udev Rules

Run these commands to create consistent port names:

```bash
# Create udev rules file
cat > /tmp/99-motor-controller.rules << 'EOF'
# udev rules for Motor Controller and LiDAR
# Motor Controller: CH340 (Vendor 1a86)
# LiDAR: CP2102 (Vendor 10c4)

# Motor Controller - CH340 chip
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7522", SYMLINK+="motor_controller", MODE="0666", GROUP="dialout"

# LiDAR - CP2102 chip (Silicon Labs)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666", GROUP="dialout"
EOF

# Install rules (requires sudo)
sudo cp /tmp/99-motor-controller.rules /etc/udev/rules.d/99-motor-controller.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

After unplugging and replugging devices, you'll have:
- `/dev/motor_controller` → Always points to motor controller
- `/dev/lidar` → Always points to LiDAR

## Update Launch File

Then update your launch file to use the consistent name:

```bash
ros2 launch generic_motor_driver complete_robot.launch.py dev:=/dev/motor_controller
```

Or update the default in the launch file to `/dev/motor_controller`.

## Quick Test

Test motor controller communication:
```bash
python3 /home/user/test_ws/src/create_robot/create_driver/scripts/comprehensive_motor_test.py
```

## Current Configuration

- Motor controller baud: **115200** (works!)
- Motor controller port: `/dev/ttyUSB0` (may change)
- LiDAR port: `/dev/ttyUSB1` (may change)








