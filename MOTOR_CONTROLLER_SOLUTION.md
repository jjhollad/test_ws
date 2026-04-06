# Motor Controller Issue - SOLVED! ✓

## Problem
Yahboom motor controller was not responding after connecting LiDAR.

## Root Cause
The Yahboom motor controller requires **1152000 baud rate**, not the default 115200!

## Solution

### Step 1: Set baud rate before launching ROS2

Run the setup script:
```bash
/home/user/test_ws/src/create_robot/create_driver/scripts/setup_motor_controller.sh /dev/ttyUSB0
```

Or manually:
```bash
stty -F /dev/ttyUSB0 1152000
```

### Step 2: Launch ROS2 with correct baud rate

```bash
cd /home/user/test_ws
source install/setup.bash
ros2 launch generic_motor_driver complete_robot.launch.py dev:=/dev/ttyUSB0 baud:=1152000
```

Or use the default (now set to 1152000):
```bash
ros2 launch generic_motor_driver complete_robot.launch.py
```

## Files Updated

1. **Launch files** - Updated default baud rate to 1152000:
   - `create_driver/launch/complete_robot.launch.py`
   - `create_driver/launch/rectangular_robot.launch.py`
   - `create_driver/launch/generic_motor_driver.launch.py`

2. **C++ driver** - Added warning for 1152000 baud (requires stty setup):
   - `create_driver/src/generic_motor_driver.cpp`

3. **Setup script** - Created helper script:
   - `create_driver/scripts/setup_motor_controller.sh`

## Testing Results

- Motor controller responds at **1152000 baud**
- Device: `/dev/ttyUSB0` (CH340 chip, vendor 1a86)
- Response format: `\x7f` bytes (acknowledgment pattern)

## Quick Start

```bash
# 1. Set baud rate
stty -F /dev/ttyUSB0 1152000

# 2. Launch ROS2
cd /home/user/test_ws
source install/setup.bash
ros2 launch generic_motor_driver complete_robot.launch.py
```

## Troubleshooting

If motor controller still doesn't work:

1. **Check device**: `ls -la /dev/ttyUSB0`
2. **Check baud rate**: `stty -F /dev/ttyUSB0`
3. **Test communication**: 
   ```bash
   python3 /home/user/test_ws/src/create_robot/create_driver/scripts/test_motor_controller.py
   ```
4. **Check permissions**: Ensure you're in `dialout` group: `groups`
5. **Check if device is in use**: `lsof /dev/ttyUSB0`

## Note on Response Format

The motor controller responds with `\x7f` (127 decimal) bytes. This appears to be an acknowledgment pattern. If encoder data doesn't come through, you may need to:
- Check motor controller documentation for exact protocol
- Update the driver to handle `\x7f` responses differently
- Try different command formats










