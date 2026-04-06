#!/usr/bin/env python3
"""
Test script for Arduino Relay Controller
Tests all relay commands and status reporting
"""

import serial
import time
import sys
import os

def test_relay_controller(port='/dev/ttyACM0', baud=115200):
    """Test Arduino relay controller"""
    print("="*60)
    print("Arduino Relay Controller Test")
    print("="*60)
    print(f"Port: {port}")
    print(f"Baud: {baud}")
    print()
    
    # Check if port exists
    if not os.path.exists(port):
        print(f"✗ Port {port} not found!")
        print(f"Available ports:")
        for p in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']:
            if os.path.exists(p):
                print(f"  - {p}")
        return False
    
    # Check permissions
    if not os.access(port, os.R_OK | os.W_OK):
        print(f"✗ Permission denied on {port}")
        print(f"Fix: sudo chmod 666 {port}")
        return False
    
    try:
        # Open serial port
        print(f"Opening {port} at {baud} baud...")
        ser = serial.Serial(port, baud, timeout=2.0)
        time.sleep(2.0)  # Wait for Arduino to reset and send READY message
        
        # Read any initial data (should be "RELAY_CONTROLLER_READY")
        print("\n1. Checking for initial READY message...")
        if ser.in_waiting > 0:
            initial_data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"   Received: {initial_data.strip()}")
            if "READY" in initial_data.upper():
                print("   ✓ Relay controller is ready!")
            else:
                print("   ⚠ Unexpected initial message")
        else:
            print("   ⚠ No initial message (Arduino may not be running relay_controller.ino)")
        
        # Test 2: Get status
        print("\n2. Requesting status...")
        ser.write(b'STATUS\n')
        ser.flush()
        time.sleep(0.5)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()
            print(f"   Response: {response}")
            if response.startswith("STATUS:"):
                print("   ✓ Status command works!")
            else:
                print("   ⚠ Unexpected response format")
        
        # Test 3: Test each relay individually
        print("\n3. Testing individual relays...")
        relays = [1, 2, 3, 4]
        for relay_num in relays:
            # Turn ON
            print(f"\n   Testing Relay {relay_num}:")
            cmd_on = f'RELAY{relay_num}_ON\n'.encode()
            print(f"     Sending: RELAY{relay_num}_ON")
            ser.write(cmd_on)
            ser.flush()
            time.sleep(0.5)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()
                print(f"     Response: {response}")
                if f"RELAY{relay_num}:ON" in response:
                    print(f"     ✓ Relay {relay_num} turned ON")
                else:
                    print(f"     ⚠ Unexpected response")
            
            # Wait a bit
            time.sleep(0.5)
            
            # Turn OFF
            cmd_off = f'RELAY{relay_num}_OFF\n'.encode()
            print(f"     Sending: RELAY{relay_num}_OFF")
            ser.write(cmd_off)
            ser.flush()
            time.sleep(0.5)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()
                print(f"     Response: {response}")
                if f"RELAY{relay_num}:OFF" in response:
                    print(f"     ✓ Relay {relay_num} turned OFF")
                else:
                    print(f"     ⚠ Unexpected response")
            
            time.sleep(0.3)
        
        # Test 4: Test ALL_ON and ALL_OFF
        print("\n4. Testing ALL_ON command...")
        ser.write(b'ALL_ON\n')
        ser.flush()
        time.sleep(0.5)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()
            print(f"   Response: {response}")
        
        time.sleep(1.0)
        
        print("\n5. Testing ALL_OFF command...")
        ser.write(b'ALL_OFF\n')
        ser.flush()
        time.sleep(0.5)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()
            print(f"   Response: {response}")
        
        # Test 6: Final status check
        print("\n6. Final status check...")
        ser.write(b'STATUS\n')
        ser.flush()
        time.sleep(0.5)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()
            print(f"   Response: {response}")
            if response.startswith("STATUS:0,0,0,0"):
                print("   ✓ All relays are OFF")
            else:
                print(f"   ⚠ Status: {response}")
        
        # Test 7: Invalid command
        print("\n7. Testing invalid command...")
        ser.write(b'INVALID_COMMAND\n')
        ser.flush()
        time.sleep(0.5)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()
            print(f"   Response: {response}")
            if "ERROR" in response:
                print("   ✓ Error handling works!")
        
        ser.close()
        
        print("\n" + "="*60)
        print("✓ Relay controller test complete!")
        print("="*60)
        return True
        
    except serial.SerialException as e:
        print(f"\n✗ Serial error: {e}")
        if "Permission denied" in str(e):
            print(f"Fix: sudo chmod 666 {port}")
        elif "could not open" in str(e).lower() or "busy" in str(e).lower():
            print(f"Port is busy. Close Arduino IDE Serial Monitor or other programs using {port}")
        return False
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

def interactive_mode(port='/dev/ttyACM0', baud=115200):
    """Interactive mode for manual testing"""
    print("\n" + "="*60)
    print("Interactive Relay Controller Mode")
    print("="*60)
    print("Commands: RELAY1_ON, RELAY1_OFF, RELAY2_ON, etc.")
    print("          ALL_ON, ALL_OFF, STATUS")
    print("          Type 'quit' to exit")
    print()
    
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2.0)  # Wait for Arduino reset
        
        # Clear initial data
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        
        while True:
            cmd = input("Relay> ").strip()
            if cmd.lower() == 'quit':
                break
            
            if cmd:
                ser.write((cmd + '\n').encode())
                ser.flush()
                time.sleep(0.3)
                
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()
                    print(f"Response: {response}")
        
        ser.close()
        print("Exiting...")
        
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Test Arduino Relay Controller')
    parser.add_argument('--port', '-p', default='/dev/ttyACM0', 
                       help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baud', '-b', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Run in interactive mode')
    
    args = parser.parse_args()
    
    if args.interactive:
        interactive_mode(args.port, args.baud)
    else:
        success = test_relay_controller(args.port, args.baud)
        sys.exit(0 if success else 1)








