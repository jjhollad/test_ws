# Arduino Relay Controller Setup

## Problem
Arduino at `/dev/ttyACM0` is not responding - the relay controller code needs to be uploaded.

## Solution: Upload relay_controller.ino

### Step 1: Open Arduino IDE
1. Open Arduino IDE
2. Make sure Arduino DUE is connected via USB

### Step 2: Open relay_controller.ino
Open the file:
```
/home/user/test_ws/src/create_robot/create_driver/arduino/relay_controller/relay_controller.ino
```

### Step 3: Select Board and Port
1. **Tools → Board → Arduino DUE (Programming Port)**
2. **Tools → Port → /dev/ttyACM0** (or whatever port your Arduino is on)

### Step 4: Upload
1. Click the **Upload** button (→ arrow icon)
2. Wait for "Done uploading" message

### Step 5: Test
After uploading, close Arduino IDE Serial Monitor (if open), then test:

```bash
python3 /home/user/test_ws/src/create_robot/create_driver/scripts/test_relay_controller.py
```

## Expected Behavior After Upload

When you open the serial port, you should see:
```
RELAY_CONTROLLER_READY
```

Then commands like `STATUS` should work.

## Troubleshooting

### Arduino not found
- Check USB connection
- Try different USB port
- Check if Arduino power LED is on

### Upload fails
- Make sure correct board is selected (Arduino DUE)
- Make sure correct port is selected
- Try pressing reset button on Arduino before uploading
- Close Serial Monitor before uploading

### Still no response after upload
- Close Arduino IDE Serial Monitor
- Unplug and replug USB
- Try the test script again








