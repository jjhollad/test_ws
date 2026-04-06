#!/usr/bin/env python3
"""
Comprehensive Arduino relay controller diagnostic
Tests serial communication systematically
"""

import serial
import time
import sys
import os

def test_port_basic(port, baud=115200):
    """Basic port test - just try to open and read"""
    print(f"\n{'='*60}")
    print(f"Testing: {port} at {baud} baud")
    print(f"{'='*60}")
    
    if not os.path.exists(port):
        print(f"✗ Port {port} does not exist!")
        return False
    
    if not os.access(port, os.R_OK | os.W_OK):
        print(f"✗ Permission denied")
        print(f"  Fix: sudo chmod 666 {port}")
        return False
    
    try:
        print(f"Opening port...")
        ser = serial.Serial(port, baud, timeout=2.0)
        print(f"✓ Port opened successfully")
        
        # Wait for Arduino to reset (Arduino resets when serial opens)
        print(f"Waiting for Arduino reset (2 seconds)...")
        time.sleep(2.0)
        
        # Read any data that came in
        print(f"Checking for data...")
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"✓ Received {len(data)} bytes:")
            try:
                text = data.decode('utf-8', errors='ignore')
                print(f"  Text: {text}")
            except:
                pass
            print(f"  Hex: {data.hex()}")
            print(f"  Raw: {data}")
        else:
            print(f"✗ No data received")
        
        # Try sending a command
        print(f"\nSending STATUS command...")
        ser.write(b'STATUS\n')
        ser.flush()
        time.sleep(0.5)
        
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"✓ Response received:")
            try:
                text = data.decode('utf-8', errors='ignore')
                print(f"  Text: {text}")
            except:
                pass
            print(f"  Hex: {data.hex()}")
        else:
            print(f"✗ No response to STATUS command")
        
        ser.close()
        return True
        
    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        if "busy" in str(e).lower():
            print(f"  Port is busy - close Arduino IDE Serial Monitor")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_multiple_bauds(port):
    """Test multiple baud rates"""
    bauds = [115200, 9600, 57600, 38400, 19200, 230400]
    
    print(f"\n{'='*60}")
    print(f"Testing multiple baud rates on {port}")
    print(f"{'='*60}")
    
    for baud in bauds:
        print(f"\nTesting {baud} baud...")
        try:
            ser = serial.Serial(port, baud, timeout=2.0)
            time.sleep(2.0)  # Wait for Arduino reset
            
            # Check for initial data
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                text = data.decode('utf-8', errors='ignore')
                print(f"  ✓ Received: {text.strip()}")
                if "READY" in text.upper():
                    print(f"  ✓ Found READY message at {baud} baud!")
                    ser.close()
                    return baud
            
            # Try sending command
            ser.write(b'STATUS\n')
            ser.flush()
            time.sleep(0.5)
            
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                text = data.decode('utf-8', errors='ignore')
                print(f"  ✓ Response: {text.strip()}")
                ser.close()
                return baud
            
            ser.close()
            print(f"  ✗ No response")
            
        except Exception as e:
            print(f"  ✗ Error: {e}")
    
    return None

def listen_for_data(port, baud=115200, duration=5):
    """Just listen for any data"""
    print(f"\n{'='*60}")
    print(f"Listening on {port} at {baud} baud for {duration} seconds")
    print(f"{'='*60}")
    print("(Don't send commands, just see if Arduino sends data)")
    
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)  # Wait for reset
        
        start = time.time()
        data_received = []
        
        while time.time() - start < duration:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                timestamp = time.time() - start
                data_received.append((timestamp, data))
                try:
                    text = data.decode('utf-8', errors='ignore')
                    print(f"[{timestamp:.2f}s] {text.strip()}")
                except:
                    print(f"[{timestamp:.2f}s] {data.hex()}")
            time.sleep(0.1)
        
        ser.close()
        
        if data_received:
            print(f"\n✓ Received {len(data_received)} data packets")
            return True
        else:
            print(f"\n✗ No data received")
            return False
            
    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    print("="*60)
    print("Arduino Relay Controller Diagnostic")
    print("="*60)
    
    # Find Arduino devices
    devices = []
    for dev in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']:
        if os.path.exists(dev):
            devices.append(dev)
    
    if not devices:
        print("\n✗ No serial devices found!")
        print("Make sure Arduino is connected via USB")
        sys.exit(1)
    
    print(f"\nFound devices: {devices}")
    
    # Test each device
    for dev in devices:
        # Basic test
        if test_port_basic(dev):
            # Try multiple baud rates
            working_baud = test_multiple_bauds(dev)
            if working_baud:
                print(f"\n{'='*60}")
                print(f"✓ SUCCESS! Arduino found at {dev} with baud {working_baud}")
                print(f"{'='*60}")
                sys.exit(0)
            
            # Listen for data
            print(f"\nTrying listen mode...")
            listen_for_data(dev)
    
    print(f"\n{'='*60}")
    print("✗ Arduino not responding")
    print(f"{'='*60}")
    print("\nTroubleshooting:")
    print("1. ✓ Is Arduino connected via USB?")
    print("2. ✓ Is relay_controller.ino uploaded to Arduino?")
    print("3. ✓ Close Arduino IDE Serial Monitor")
    print("4. ✓ Check Arduino power LED")
    print("5. ✓ Try pressing Arduino reset button")
    print("6. ✓ Try different USB cable/port")
    print("\nTo upload relay_controller.ino:")
    print("  1. Open Arduino IDE")
    print("  2. Open relay_controller.ino")
    print("  3. Select correct board (Arduino DUE)")
    print("  4. Select correct port")
    print("  5. Click Upload")
    
    sys.exit(1)

if __name__ == '__main__':
    main()








