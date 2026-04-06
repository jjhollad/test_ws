#!/usr/bin/env python3
"""
Yahboom motor controller specific test
Tests various baud rates and checks if device sends data automatically
"""

import serial
import time
import sys

def test_baud_rates(port):
    """Test all common baud rates"""
    bauds = [115200, 9600, 57600, 38400, 19200, 1152000, 230400]
    
    for baud in bauds:
        print(f"\nTesting {baud} baud...")
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
            time.sleep(0.2)
            
            # First, just listen for any spontaneous data
            print("  Listening for 0.5 seconds...")
            ser.reset_input_buffer()
            time.sleep(0.5)
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"  ✓ Received spontaneous data: {data}")
                ser.close()
                return True, baud
            
            # Try sending commands
            commands = [
                b'$spd:0,0,0,0#',      # Stop
                b'$upload:1,1,1#',      # Request encoder
                b'$MAll#',              # Alternative
                b'AT\r\n',             # AT command
                b'\r\n',               # Just newline
            ]
            
            for cmd in commands:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                ser.write(cmd)
                ser.flush()
                time.sleep(0.2)
                
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    print(f"  ✓ Response to {cmd}: {data}")
                    ser.close()
                    return True, baud
            
            ser.close()
        except Exception as e:
            print(f"  Error: {e}")
    
    return False, None

def listen_only(port, baud=115200, duration=3):
    """Just listen without sending anything"""
    print(f"\nListening on {port} at {baud} baud for {duration} seconds...")
    print("(Don't send any commands, just see if device talks)")
    
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.1)
        ser.reset_input_buffer()
        
        start = time.time()
        data_received = False
        
        while time.time() - start < duration:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"  Received: {data}")
                data_received = True
            time.sleep(0.1)
        
        ser.close()
        return data_received
    except Exception as e:
        print(f"  Error: {e}")
        return False

def test_raw_commands(port, baud=115200):
    """Test raw serial communication"""
    print(f"\nTesting raw communication on {port}...")
    
    try:
        ser = serial.Serial(port, baud, timeout=0.3)
        time.sleep(0.1)
        
        # Configure for raw mode
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Try different command formats
        test_commands = [
            (b'$spd:0,0,0,0#', "Stop command"),
            (b'$spd:0,0,0,0\r\n', "Stop with CRLF"),
            (b'spd:0,0,0,0\n', "No $ or #"),
            (b'STOP\n', "Text command"),
            (b'\x00\x01\x02', "Binary test"),
        ]
        
        for cmd, desc in test_commands:
            print(f"  Trying: {desc} ({cmd})")
            ser.write(cmd)
            ser.flush()
            time.sleep(0.3)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"    ✓ Response: {response}")
                ser.close()
                return True
        
        ser.close()
        return False
    except Exception as e:
        print(f"  Error: {e}")
        return False

if __name__ == '__main__':
    port = '/dev/ttyUSB0'
    
    print("="*60)
    print("Yahboom Motor Controller Comprehensive Test")
    print("="*60)
    
    # Check if port exists
    import os
    if not os.path.exists(port):
        print(f"Error: {port} not found!")
        sys.exit(1)
    
    # Test 1: Just listen (device might send data automatically)
    print("\n[TEST 1] Listening for automatic data...")
    if listen_only(port):
        print("✓ Device is sending data automatically!")
    else:
        print("✗ No automatic data")
    
    # Test 2: Try different baud rates
    print("\n[TEST 2] Testing different baud rates...")
    found, baud = test_baud_rates(port)
    if found:
        print(f"\n✓ Found working baud rate: {baud}")
        sys.exit(0)
    
    # Test 3: Raw command testing
    print("\n[TEST 3] Testing raw commands...")
    if test_raw_commands(port):
        print("✓ Got response with raw commands!")
        sys.exit(0)
    
    print("\n" + "="*60)
    print("No response from motor controller")
    print("="*60)
    print("\nPossible issues:")
    print("1. Motor controller not powered (check LED)")
    print("2. Wrong USB cable (needs data lines, not just power)")
    print("3. Motor controller needs hardware reset")
    print("4. Different protocol than expected")
    print("5. Motor controller may need initialization sequence")
    print("\nNext steps:")
    print("- Check motor controller power LED")
    print("- Try unplugging/replugging USB")
    print("- Check motor controller manual/documentation")
    print("- Try different USB cable")
    print("- Check if motor controller has a reset button")
    
    sys.exit(1)










