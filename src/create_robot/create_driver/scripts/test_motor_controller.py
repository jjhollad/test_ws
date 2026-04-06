#!/usr/bin/env python3
"""
Simple test script to communicate with Yahboom motor controller
Tests both /dev/ttyUSB0 and /dev/ttyUSB1 to find which is the motor controller
"""

import serial
import time
import sys

def test_device(port, baud=115200):
    """Test if a serial port responds to motor controller commands"""
    print(f"\nTesting {port}...")
    
    try:
        # Open serial port
        ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(0.1)  # Wait for port to initialize
        
        # Clear any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Send stop command
        command = b'$spd:0,0,0,0#'
        print(f"  Sending: {command.decode()}")
        ser.write(command)
        ser.flush()
        time.sleep(0.1)
        
        # Try to read response
        response = ser.read(100)
        
        if response:
            print(f"  ✓ RESPONSE RECEIVED: {response}")
            print(f"  → This appears to be the motor controller!")
            ser.close()
            return True
        else:
            print(f"  ✗ No response")
            ser.close()
            return False
            
    except serial.SerialException as e:
        print(f"  ERROR: {e}")
        if "Permission denied" in str(e):
            print(f"  → Fix: sudo chmod 666 {port}")
            print(f"  → Or add user to dialout: sudo usermod -a -G dialout $USER")
        return False
    except Exception as e:
        print(f"  ERROR: {e}")
        return False

def main():
    print("=" * 50)
    print("Yahboom Motor Controller Test")
    print("=" * 50)
    
    # Test both USB ports
    devices = ['/dev/ttyUSB0', '/dev/ttyUSB1']
    found = False
    
    for dev in devices:
        if test_device(dev):
            found = True
            print(f"\n✓ Motor controller found at: {dev}")
            print(f"\nTo use this device, update your launch file:")
            print(f"  ros2 launch generic_motor_driver complete_robot.launch.py dev:={dev}")
            break
    
    if not found:
        print("\n✗ Motor controller not responding on either port")
        print("\nTroubleshooting:")
        print("1. Check if user is in dialout group: groups")
        print("2. Try: sudo chmod 666 /dev/ttyUSB*")
        print("3. Unplug LiDAR and test again")
        print("4. Check power supply - LiDAR may be drawing too much current")
        print("5. Check USB hub power if using one")
        sys.exit(1)

if __name__ == '__main__':
    main()





