#!/usr/bin/env python3
"""
Test motors on ESP32 devices directly
This script is used to test the motors without starting the full robot system
"""

import sys
import os
import time
import argparse
import serial
import logging

def setup_logging(debug=False):
    """Set up logging configuration"""
    level = logging.DEBUG if debug else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler()
        ]
    )
    return logging.getLogger("MotorTest")

def find_serial_ports():
    """Find available serial ports"""
    import glob
    
    # Check common Linux/Raspberry Pi serial ports
    available_ports = []
    
    # Find all tty devices on Linux/Mac
    if os.name == 'posix':
        available_ports.extend(glob.glob('/dev/tty*'))
    
    # Find COM ports on Windows
    elif os.name == 'nt':
        for i in range(1, 20):
            port = f"COM{i}"
            try:
                s = serial.Serial(port)
                s.close()
                available_ports.append(port)
            except (OSError, serial.SerialException):
                pass
    
    return available_ports

def connect_to_port(port, baud=9600, timeout=1):
    """Connect to a serial port"""
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f"Connected to {port}")
        return ser
    except Exception as e:
        print(f"Failed to connect to {port}: {e}")
        return None

def send_command(ser, command, logger=None, wait_time=0.5):
    """Send a command to the serial port and return the response"""
    if not ser:
        print("Serial port not connected")
        return None
        
    try:
        # Clear input buffer
        ser.reset_input_buffer()
        
        # Send command
        if logger:
            logger.debug(f"Sending: {command}")
        ser.write(f"{command}\n".encode())
        
        # Wait for response
        time.sleep(wait_time)
        
        # Read response
        response = ""
        while ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                if logger:
                    logger.debug(f"Received: {line}")
                response += line + "\n"
        
        return response
    except Exception as e:
        if logger:
            logger.error(f"Error sending command: {e}")
        print(f"Error sending command: {e}")
        return None

def test_nav_motors(ser, logger=None):
    """Test navigation motors"""
    print("\n=== Testing Navigation Motors ===")
    
    if not ser:
        print("Navigation controller not connected")
        return False
    
    try:
        # First clear any emergency state
        print("Clearing emergency status...")
        send_command(ser, "No_Emergency", logger)
        time.sleep(1)
        
        # Test forward movement
        print("Moving forward for 2 seconds...")
        send_command(ser, "IR:ON", logger)
        time.sleep(2)
        
        # Stop
        print("Stopping...")
        send_command(ser, "IR:OFF", logger)
        time.sleep(1)
        
        # Test left turn
        print("Turning left for 2 seconds...")
        send_command(ser, "TURN_LEFT", logger)
        time.sleep(2)
        
        # Stop
        print("Stopping...")
        send_command(ser, "IR:OFF", logger)
        time.sleep(1)
        
        # Test right turn
        print("Turning right for 2 seconds...")
        send_command(ser, "TURN_RIGHT", logger)
        time.sleep(2)
        
        # Stop
        print("Stopping...")
        send_command(ser, "IR:OFF", logger)
        
        print("Navigation motor test complete")
        return True
        
    except Exception as e:
        print(f"Error testing navigation motors: {e}")
        return False
    finally:
        # Make sure motors are stopped
        send_command(ser, "IR:OFF", logger)

def test_arm_motors(ser, logger=None):
    """Test arm motors"""
    print("\n=== Testing Arm Motors ===")
    
    if not ser:
        print("Arm controller not connected")
        return False
    
    try:
        # First clear any emergency state
        print("Clearing emergency status...")
        send_command(ser, "No_Emergency", logger)
        time.sleep(1)
        
        # Test each motor
        for motor_num in [5, 6, 7]:
            # Forward
            print(f"Moving motor {motor_num} forward...")
            send_command(ser, f"FWD{motor_num}", logger)
            time.sleep(2)
            
            # Stop by sending the same command (toggle behavior)
            print(f"Stopping motor {motor_num}...")
            send_command(ser, f"FWD{motor_num}", logger)
            time.sleep(1)
            
            # Backward
            print(f"Moving motor {motor_num} backward...")
            send_command(ser, f"BACK{motor_num}", logger)
            time.sleep(2)
            
            # Stop by sending the same command (toggle behavior)
            print(f"Stopping motor {motor_num}...")
            send_command(ser, f"BACK{motor_num}", logger)
            time.sleep(1)
        
        # Test gripper
        print("Opening gripper...")
        send_command(ser, "RELEASE", logger)
        time.sleep(2)
        
        print("Closing gripper...")
        send_command(ser, "GRAB", logger)
        time.sleep(2)
        
        print("Arm motor test complete")
        return True
        
    except Exception as e:
        print(f"Error testing arm motors: {e}")
        return False

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Test ESP32 motors directly')
    parser.add_argument('--nav', action='store_true', help='Test navigation motors')
    parser.add_argument('--arm', action='store_true', help='Test arm motors')
    parser.add_argument('--nav-port', help='Serial port for navigation controller')
    parser.add_argument('--arm-port', help='Serial port for arm controller')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    args = parser.parse_args()
    
    # If no specific test is selected, test both
    if not args.nav and not args.arm:
        args.nav = True
        args.arm = True
    
    # Set up logging
    logger = setup_logging(args.debug)
    
    # Find available ports
    available_ports = find_serial_ports()
    print(f"Available serial ports: {available_ports}")
    
    # Connect to navigation controller
    nav_ser = None
    if args.nav:
        if args.nav_port:
            nav_ser = connect_to_port(args.nav_port)
        else:
            print("No navigation port specified. Please use --nav-port")
    
    # Connect to arm controller
    arm_ser = None
    if args.arm:
        if args.arm_port:
            arm_ser = connect_to_port(args.arm_port)
        else:
            print("No arm port specified. Please use --arm-port")
    
    # Test navigation motors
    if args.nav and nav_ser:
        test_nav_motors(nav_ser, logger)
    
    # Test arm motors
    if args.arm and arm_ser:
        test_arm_motors(arm_ser, logger)
    
    # Close serial ports
    if nav_ser:
        nav_ser.close()
    if arm_ser:
        arm_ser.close()

if __name__ == "__main__":
    main() 