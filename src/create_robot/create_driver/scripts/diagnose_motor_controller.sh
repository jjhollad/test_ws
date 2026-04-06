#!/bin/bash
# Diagnostic script for Yahboom motor controller after LiDAR connection

echo "=========================================="
echo "Yahboom Motor Controller Diagnostic"
echo "=========================================="
echo ""

# 1. List all serial devices
echo "1. Available Serial Devices:"
echo "------------------------------"
echo "USB Serial Devices (ttyUSB*):"
ls -la /dev/ttyUSB* 2>/dev/null | while read line; do
    dev=$(echo $line | awk '{print $NF}')
    echo "  $line"
    # Try to identify device
    if udevadm info -q name -n $dev >/dev/null 2>&1; then
        vendor=$(udevadm info -q property -n $dev 2>/dev/null | grep ID_VENDOR= | cut -d= -f2)
        model=$(udevadm info -q property -n $dev 2>/dev/null | grep ID_MODEL= | cut -d= -f2)
        serial=$(udevadm info -q property -n $dev 2>/dev/null | grep ID_SERIAL_SHORT= | cut -d= -f2)
        echo "    Vendor: $vendor"
        echo "    Model: $model"
        echo "    Serial: $serial"
    fi
done

echo ""
echo "ACM Serial Devices (ttyACM*):"
ls -la /dev/ttyACM* 2>/dev/null | while read line; do
    dev=$(echo $line | awk '{print $NF}')
    echo "  $line"
    if udevadm info -q name -n $dev >/dev/null 2>&1; then
        vendor=$(udevadm info -q property -n $dev 2>/dev/null | grep ID_VENDOR= | cut -d= -f2)
        model=$(udevadm info -q property -n $dev 2>/dev/null | grep ID_MODEL= | cut -d= -f2)
        echo "    Vendor: $vendor"
        echo "    Model: $model"
    fi
done
echo ""

# 2. Check permissions
echo "2. Device Permissions:"
echo "----------------------"
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$dev" ]; then
        perms=$(stat -c "%a %U:%G" "$dev" 2>/dev/null)
        echo "  $dev: $perms"
        if [[ ! "$perms" =~ "rw" ]]; then
            echo "    WARNING: Device may not be readable/writable by your user"
        fi
    fi
done
echo ""

# 3. Test each USB device for motor controller response
echo "3. Testing Devices for Motor Controller Response:"
echo "--------------------------------------------------"
echo "Sending test commands to each device..."
echo ""

for dev in /dev/ttyUSB*; do
    if [ -e "$dev" ]; then
        echo "Testing $dev..."
        
        # Check if device is accessible
        if [ ! -r "$dev" ] || [ ! -w "$dev" ]; then
            echo "  ERROR: Cannot read/write $dev (permission denied)"
            echo "  Try: sudo chmod 666 $dev"
            echo ""
            continue
        fi
        
        # Try to send a command and read response
        # Use stty to configure serial port
        stty -F "$dev" 115200 cs8 -cstopb -parenb raw -echo -echoe -echok 2>/dev/null
        
        if [ $? -eq 0 ]; then
            # Send stop command
            echo -n '$spd:0,0,0,0#' > "$dev" 2>/dev/null
            sleep 0.1
            
            # Try to read response (with timeout)
            timeout 0.5 cat < "$dev" > /tmp/motor_test_$$ 2>/dev/null &
            CAT_PID=$!
            sleep 0.3
            kill $CAT_PID 2>/dev/null
            wait $CAT_PID 2>/dev/null
            
            if [ -s /tmp/motor_test_$$ ]; then
                response=$(cat /tmp/motor_test_$$)
                echo "  ✓ RESPONSE RECEIVED: $response"
                echo "  → This is likely the motor controller!"
            else
                echo "  ✗ No response (might be LiDAR or other device)"
            fi
            rm -f /tmp/motor_test_$$ 2>/dev/null
        else
            echo "  ERROR: Could not configure $dev"
        fi
        echo ""
    fi
done

# 4. Check if ROS2 motor driver is running
echo "4. ROS2 Motor Driver Status:"
echo "-----------------------------"
if command -v ros2 >/dev/null 2>&1; then
    if ros2 node list 2>/dev/null | grep -q generic_motor_driver; then
        echo "  Motor driver node is running"
        echo "  Current device parameter:"
        ros2 param get /generic_motor_driver dev 2>/dev/null || echo "    (could not get parameter)"
    else
        echo "  Motor driver node is NOT running"
    fi
else
    echo "  ROS2 not found in PATH"
fi
echo ""

# 5. Check dmesg for recent USB device connections
echo "5. Recent USB Device Connections (last 20 lines):"
echo "-------------------------------------------------"
dmesg | tail -20 | grep -i "usb\|tty" | tail -10
echo ""

# 6. Recommendations
echo "6. Recommendations:"
echo "-------------------"
echo "If motor controller moved to a different port:"
echo "  1. Update launch file: dev:=/dev/ttyUSB1 (or correct port)"
echo "  2. Or create udev rule to assign fixed name"
echo ""
echo "If no response from any device:"
echo "  1. Check power supply - LiDAR may be drawing too much current"
echo "  2. Check USB hub power (if using hub)"
echo "  3. Try unplugging LiDAR to see if motor controller works"
echo "  4. Check wiring connections"
echo ""

echo "=========================================="
echo "Diagnostic Complete"
echo "=========================================="





