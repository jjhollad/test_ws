#!/usr/bin/env python3
"""
Advanced motor controller test - tries different baud rates and commands
"""

import serial
import time
import sys

def test_device_advanced(port):
    """Test device with multiple baud rates and commands"""
    print(f"\n{'='*60}")
    print(f"Advanced Testing: {port}")
    print(f"{'='*60}")
    
    baud_rates = [115200, 9600, 57600, 38400, 19200]
    commands = [
        b'$spd:0,0,0,0#',      # Stop command
        b'$upload:1,1,1#',      # Request encoder data
        b'$MAll#',              # Alternative encoder request
        b'$MTEP#',              # Real-time encoder
        b'\r\n',                # Just newline
        b'AT\r\n',              # Some controllers respond to AT
    ]
    
    for baud in baud_rates:
        print(f"\n  Testing baud rate: {baud}")
        try:
            ser = serial.Serial(port, baud, timeout=0.5, write_timeout=0.5)
            time.sleep(0.2)
            
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            for cmd in commands:
                try:
                    print(f"    Sending: {cmd}")
                    ser.write(cmd)
                    ser.flush()
                    time.sleep(0.2)
                    
                    # Try to read response
                    response = ser.read(100)
                    if response:
                        print(f"    ✓ RESPONSE: {response}")
                        print(f"    ✓ Found working baud rate: {baud}")
                        ser.close()
                        return True, baud
                    else:
                        # Check if any data is available (even if not complete)
                        if ser.in_waiting > 0:
                            partial = ser.read(ser.in_waiting)
                            print(f"    → Partial data: {partial}")
                except Exception as e:
                    print(f"    Error sending command: {e}")
            
            ser.close()
        except serial.SerialException as e:
            print(f"    Cannot open at {baud}: {e}")
        except Exception as e:
            print(f"    Error: {e}")
    
    return False, None

def check_port_activity(port):
    """Check if port is receiving any data at all"""
    print(f"\n  Checking for any incoming data on {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(0.1)
        ser.reset_input_buffer()
        
        print("    Listening for 2 seconds...")
        start = time.time()
        data_received = False
        
        while time.time() - start < 2:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"    ✓ Received data: {data}")
                data_received = True
            time.sleep(0.1)
        
        ser.close()
        return data_received
    except Exception as e:
        print(f"    Error: {e}")
        return False

def main():
    print("="*60)
    print("Advanced Motor Controller Diagnostic")
    print("="*60)
    
    devices = ['/dev/ttyUSB0', '/dev/ttyUSB1']
    
    for dev in devices:
        # First check if device exists and is accessible
        try:
            ser = serial.Serial(dev, 115200, timeout=0.1)
            ser.close()
        except serial.SerialException as e:
            if "Permission denied" in str(e):
                print(f"\n{dev}: Permission denied")
                print(f"  Fix: sudo chmod 666 {dev}")
                continue
            elif "could not open" in str(e).lower():
                print(f"\n{dev}: Cannot open - may be in use")
                print(f"  Check: lsof {dev}")
                continue
            else:
                print(f"\n{dev}: {e}")
                continue
        
        # Check for spontaneous data
        if check_port_activity(dev):
            print(f"  → Device is sending data (might be LiDAR)")
        
        # Test with various commands
        found, baud = test_device_advanced(dev)
        if found:
            print(f"\n✓ Motor controller found at {dev} with baud rate {baud}")
            print(f"\nUpdate your launch file:")
            print(f"  ros2 launch generic_motor_driver complete_robot.launch.py dev:={dev} baud:={baud}")
            sys.exit(0)
    
    print("\n" + "="*60)
    print("No motor controller response found")
    print("="*60)
    print("\nPossible issues:")
    print("1. Motor controller not powered")
    print("2. Wrong USB cable (data vs power-only)")
    print("3. Motor controller needs initialization/reset")
    print("4. Motor controller uses different protocol")
    print("5. USB hub power issue (try direct connection)")
    print("\nTry:")
    print("- Unplug LiDAR and test again")
    print("- Unplug/replug motor controller")
    print("- Check motor controller power LED")
    print("- Try different USB port on computer")
    sys.exit(1)

if __name__ == '__main__':
    main()










