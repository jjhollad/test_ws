#!/usr/bin/env python3
"""
Comprehensive motor controller communication test
Tests everything systematically to find the issue
"""

import serial
import time
import sys
import os

def test_port_exists(port):
    """Check if port exists and is accessible"""
    print(f"\n{'='*60}")
    print(f"Testing: {port}")
    print(f"{'='*60}")
    
    if not os.path.exists(port):
        print(f"✗ Port {port} does not exist!")
        return False
    
    print(f"✓ Port exists: {port}")
    
    # Check permissions
    if not os.access(port, os.R_OK | os.W_OK):
        print(f"✗ Permission denied on {port}")
        print(f"  Fix: sudo chmod 666 {port}")
        print(f"  Or: sudo usermod -a -G dialout $USER")
        return False
    
    print(f"✓ Port is readable/writable")
    return True

def test_baud_rate(port, baud):
    """Test if we can open port at specific baud rate"""
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(0.1)
        ser.close()
        return True
    except Exception as e:
        print(f"  Cannot open at {baud}: {e}")
        return False

def test_communication(port, baud):
    """Test actual communication"""
    print(f"\nTesting communication at {baud} baud...")
    
    try:
        ser = serial.Serial(port, baud, timeout=1.0, write_timeout=1.0)
        time.sleep(0.2)
        
        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Test 1: Just listen (device might send data automatically)
        print("  Test 1: Listening for automatic data (2 seconds)...")
        start = time.time()
        data_received = False
        while time.time() - start < 2.0:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"    ✓ Received: {data} (hex: {data.hex()})")
                data_received = True
            time.sleep(0.1)
        
        if not data_received:
            print("    ✗ No automatic data")
        
        # Test 2: Send stop command
        print("  Test 2: Sending stop command...")
        cmd = b'$spd:0,0,0,0#'
        print(f"    Sending: {cmd}")
        ser.write(cmd)
        ser.flush()
        time.sleep(0.5)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"    ✓ Response: {response} (hex: {response.hex()})")
            ser.close()
            return True
        else:
            print("    ✗ No response")
        
        # Test 3: Try different command formats
        print("  Test 3: Trying different command formats...")
        commands = [
            (b'$upload:1,1,1#', 'Encoder request'),
            (b'$MAll#', 'MAll command'),
            (b'AT\r\n', 'AT command'),
            (b'?\r\n', 'Query command'),
        ]
        
        for cmd, desc in commands:
            ser.reset_input_buffer()
            ser.write(cmd)
            ser.flush()
            time.sleep(0.5)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"    ✓ {desc}: {response} (hex: {response.hex()})")
                ser.close()
                return True
        
        ser.close()
        return False
        
    except serial.SerialException as e:
        print(f"  ✗ Serial error: {e}")
        return False
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False

def check_system_baud(port):
    """Check what baud rate the system thinks the port is at"""
    import subprocess
    try:
        result = subprocess.run(['stty', '-F', port], 
                              capture_output=True, text=True, timeout=2)
        print(f"\nSystem stty settings for {port}:")
        for line in result.stdout.split('\n'):
            if 'speed' in line.lower() or 'baud' in line.lower():
                print(f"  {line}")
        return True
    except:
        return False

def main():
    print("="*60)
    print("Comprehensive Motor Controller Communication Test")
    print("="*60)
    
    # Find USB devices
    devices = []
    for dev in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0']:
        if os.path.exists(dev):
            devices.append(dev)
    
    if not devices:
        print("\n✗ No USB serial devices found!")
        print("  Make sure motor controller is connected")
        sys.exit(1)
    
    print(f"\nFound {len(devices)} device(s): {devices}")
    
    # Test each device
    for dev in devices:
        if not test_port_exists(dev):
            continue
        
        # Check system baud rate
        check_system_baud(dev)
        
        # Test multiple baud rates
        bauds_to_test = [1152000, 115200, 9600, 57600, 38400]
        
        print(f"\nTesting baud rates: {bauds_to_test}")
        
        for baud in bauds_to_test:
            if not test_baud_rate(dev, baud):
                continue
            
            if test_communication(dev, baud):
                print(f"\n{'='*60}")
                print(f"✓ SUCCESS! Motor controller found!")
                print(f"  Device: {dev}")
                print(f"  Baud rate: {baud}")
                print(f"{'='*60}")
                print(f"\nTo use this configuration:")
                print(f"  1. Set baud rate: stty -F {dev} {baud}")
                print(f"  2. Launch ROS2: ros2 launch generic_motor_driver complete_robot.launch.py dev:={dev} baud:={baud}")
                sys.exit(0)
        
        print(f"\n✗ No communication on {dev} at any tested baud rate")
    
    print(f"\n{'='*60}")
    print("✗ Motor controller not responding")
    print(f"{'='*60}")
    print("\nTroubleshooting checklist:")
    print("1. ✓ Motor controller powered? (check LED)")
    print("2. ✓ USB cable is data-capable? (not power-only)")
    print("3. ✓ Correct USB port on computer?")
    print("4. ✓ Try unplugging/replugging USB")
    print("5. ✓ Check motor controller manual for correct baud rate")
    print("6. ✓ Motor controller may need initialization/reset")
    print("\nTry:")
    print("  - Different USB cable")
    print("  - Different USB port")
    print("  - Check motor controller power supply")
    print("  - Look for reset button on motor controller")
    
    sys.exit(1)

if __name__ == '__main__':
    main()










