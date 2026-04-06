#!/usr/bin/env python3
"""
Quick motor controller test - fast diagnostic
"""

import serial
import time
import sys

def quick_test(port='/dev/ttyUSB0', baud=115200):
    """Quick test of motor controller"""
    print(f"Testing {port} at {baud} baud...")
    
    try:
        ser = serial.Serial(port, baud, timeout=0.3, write_timeout=0.3)
        time.sleep(0.1)
        
        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Send stop command
        cmd = b'$spd:0,0,0,0#'
        print(f"  Sending: {cmd.decode()}")
        ser.write(cmd)
        ser.flush()
        
        # Wait for response
        time.sleep(0.2)
        
        # Check for any data
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"  ✓ Response: {response}")
            ser.close()
            return True
        else:
            print(f"  ✗ No response")
            ser.close()
            return False
            
    except serial.SerialException as e:
        if "Permission denied" in str(e):
            print(f"  ✗ Permission denied - run: sudo chmod 666 {port}")
        elif "could not open" in str(e).lower() or "busy" in str(e).lower():
            print(f"  ✗ Port busy - close Arduino Serial Monitor or other programs")
        else:
            print(f"  ✗ Error: {e}")
        return False
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False

def check_device_info(port):
    """Check device information"""
    import subprocess
    try:
        result = subprocess.run(['udevadm', 'info', '-q', 'property', '-n', port], 
                              capture_output=True, text=True, timeout=2)
        props = {}
        for line in result.stdout.split('\n'):
            if '=' in line:
                key, value = line.split('=', 1)
                props[key] = value
        
        print(f"\nDevice Info for {port}:")
        print(f"  Vendor: {props.get('ID_VENDOR_FROM_DATABASE', props.get('ID_VENDOR', 'Unknown'))}")
        print(f"  Model: {props.get('ID_MODEL_FROM_DATABASE', props.get('ID_MODEL', 'Unknown'))}")
        print(f"  Serial: {props.get('ID_SERIAL_SHORT', 'N/A')}")
        return props
    except:
        print(f"  Could not get device info")
        return {}

if __name__ == '__main__':
    print("="*50)
    print("Quick Motor Controller Test")
    print("="*50)
    
    # Check what devices are available
    import os
    devices = []
    for dev in ['/dev/ttyUSB0', '/dev/ttyUSB1']:
        if os.path.exists(dev):
            devices.append(dev)
    
    if not devices:
        print("No USB serial devices found!")
        sys.exit(1)
    
    # Test each device
    for dev in devices:
        check_device_info(dev)
        if quick_test(dev):
            print(f"\n✓ Motor controller found at {dev}!")
            print(f"\nUse in launch file:")
            print(f"  dev:={dev}")
            sys.exit(0)
        print()
    
    print("="*50)
    print("Motor controller not responding")
    print("="*50)
    print("\nTroubleshooting:")
    print("1. Check motor controller power LED")
    print("2. Try unplugging/replugging USB cable")
    print("3. Try different USB port on computer")
    print("4. Check USB cable (must be data-capable, not power-only)")
    print("5. Check motor controller manual for correct baud rate")
    print("\nTry manual test:")
    print(f"  sudo chmod 666 {devices[0]}")
    print(f"  echo '$spd:0,0,0,0#' > {devices[0]}")
    print(f"  cat < {devices[0]}")
    sys.exit(1)










