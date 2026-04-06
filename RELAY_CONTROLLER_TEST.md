# Arduino Relay Controller Test Guide

## Quick Test

The relay controller test script is ready. First, make sure nothing else is using the Arduino port:

### Step 1: Close Arduino IDE Serial Monitor
If Arduino IDE Serial Monitor is open, close it.

### Step 2: Run the test
```bash
python3 /home/user/test_ws/src/create_robot/create_driver/scripts/test_relay_controller.py
```

### Step 3: Interactive Mode (optional)
For manual testing:
```bash
python3 /home/user/test_ws/src/create_robot/create_driver/scripts/test_relay_controller.py --interactive
```

## Test Commands

The relay controller accepts these commands:
- `RELAY1_ON` - Turn relay 1 on
- `RELAY1_OFF` - Turn relay 1 off
- `RELAY2_ON`, `RELAY2_OFF` - Relay 2
- `RELAY3_ON`, `RELAY3_OFF` - Relay 3
- `RELAY4_ON`, `RELAY4_OFF` - Relay 4
- `ALL_ON` - Turn all relays on
- `ALL_OFF` - Turn all relays off
- `STATUS` - Get relay status (returns STATUS:0,0,0,0 format)

## Expected Behavior

1. **On startup**: Arduino sends "RELAY_CONTROLLER_READY"
2. **On command**: Arduino responds with confirmation like "RELAY1:ON"
3. **On STATUS**: Arduino responds with "STATUS:1,0,1,0" (1=ON, 0=OFF)

## Troubleshooting

### Port Busy
If you get "Device or resource busy":
```bash
# Check what's using it
lsof /dev/ttyACM0

# Kill Arduino Serial Monitor if needed
pkill -f serial-monitor
```

### Permission Denied
```bash
sudo chmod 666 /dev/ttyACM0
# Or add user to dialout group
sudo usermod -a -G dialout $USER
# Then log out and back in
```

### Wrong Port
If relay controller is on a different port:
```bash
python3 /home/user/test_ws/src/create_robot/create_driver/scripts/test_relay_controller.py --port /dev/ttyACM1
```

## Testing with ROS2

Once the relay controller works, test with ROS2:
```bash
cd /home/user/test_ws
source install/setup.bash
ros2 launch generic_motor_driver relay_controller.launch.py dev:=/dev/ttyACM0
```

Then in another terminal:
```bash
# Check relay status topic
ros2 topic echo /relay_status

# Control relays (if there's a service or topic)
# Check available services:
ros2 service list | grep relay
```








