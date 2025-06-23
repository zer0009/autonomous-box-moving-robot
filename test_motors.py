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
            try:
                line = ser.readline().decode(errors="replace").strip()
                if line:
                    if logger:
                        logger.debug(f"Received: {line}")
                    response += line + "\n"
            except Exception as e:
                if logger:
                    logger.error(f"Error decoding response: {e}")
                continue
        
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
        send_command(ser, "No_Emergency", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test direct motor control commands
        print("Testing left motors forward (3 seconds)...")
        send_command(ser, "MOTOR:LEFT:FORWARD", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop left motors
        print("Stopping left motors...")
        send_command(ser, "MOTOR:LEFT:STOP", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test right motors forward
        print("Testing right motors forward (3 seconds)...")
        send_command(ser, "MOTOR:RIGHT:FORWARD", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop right motors
        print("Stopping right motors...")
        send_command(ser, "MOTOR:RIGHT:STOP", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test all motors forward (standard IR:ON command)
        print("Testing all motors forward with IR:ON (3 seconds)...")
        send_command(ser, "IR:ON", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop all motors
        print("Stopping all motors with IR:OFF...")
        send_command(ser, "IR:OFF", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test left turn using direct motor control
        print("Testing left turn (right motors forward, left motors backward) for 3 seconds...")
        send_command(ser, "MOTOR:LEFT:BACKWARD", logger, wait_time=0.5)
        send_command(ser, "MOTOR:RIGHT:FORWARD", logger, wait_time=0.5)
        time.sleep(3)
        
        # Stop all motors
        print("Stopping all motors...")
        send_command(ser, "MOTOR:LEFT:STOP", logger, wait_time=0.5)
        send_command(ser, "MOTOR:RIGHT:STOP", logger, wait_time=0.5)
        time.sleep(1)
        
        # Test right turn using direct motor control
        print("Testing right turn (left motors forward, right motors backward) for 3 seconds...")
        send_command(ser, "MOTOR:LEFT:FORWARD", logger, wait_time=0.5)
        send_command(ser, "MOTOR:RIGHT:BACKWARD", logger, wait_time=0.5)
        time.sleep(3)
        
        # Stop all motors
        print("Stopping all motors...")
        send_command(ser, "MOTOR:LEFT:STOP", logger, wait_time=0.5)
        send_command(ser, "MOTOR:RIGHT:STOP", logger, wait_time=0.5)
        
        # Try using the standard turn commands as well
        print("Testing standard TURN_LEFT command (3 seconds)...")
        send_command(ser, "TURN_LEFT", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop
        print("Stopping with IR:OFF...")
        send_command(ser, "IR:OFF", logger, wait_time=1.0)
        time.sleep(1)
        
        # Try right turn command
        print("Testing standard TURN_RIGHT command (3 seconds)...")
        send_command(ser, "TURN_RIGHT", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop
        print("Stopping with IR:OFF...")
        send_command(ser, "IR:OFF", logger, wait_time=1.0)
        
        print("Navigation motor test complete")
        return True
        
    except Exception as e:
        print(f"Error testing navigation motors: {e}")
        return False
    finally:
        # Make sure motors are stopped
        try:
            send_command(ser, "IR:OFF", logger, wait_time=1.0)
            print("Motors stopped")
        except:
            print("Warning: Could not send final stop command")

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

def test_esp_now_arm(ser, logger=None):
    """Test ARM motors via ESP-NOW communication"""
    print("\n=== Testing ARM Motors via ESP-NOW ===")
    print("Sending commands through NAV controller to ARM via ESP-NOW")
    
    if not ser:
        print("NAV controller not connected")
        return False
    
    try:
        # First clear any emergency state
        print("Clearing emergency status...")
        send_command(ser, "ESP_NOW:ARM:No_Emergency", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test motor 5 (forward)
        print("Testing ARM motor 5 forward via ESP-NOW...")
        send_command(ser, "ESP_NOW:ARM:FWD5", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop motor 5
        print("Stopping ARM motor 5...")
        send_command(ser, "ESP_NOW:ARM:FWD5", logger, wait_time=1.0)  # Toggle to stop
        time.sleep(1)
        
        # Test motor 5 (backward)
        print("Testing ARM motor 5 backward via ESP-NOW...")
        send_command(ser, "ESP_NOW:ARM:BACK5", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop motor 5
        print("Stopping ARM motor 5...")
        send_command(ser, "ESP_NOW:ARM:BACK5", logger, wait_time=1.0)  # Toggle to stop
        time.sleep(1)
        
        # Test motor 6 (forward)
        print("Testing ARM motor 6 forward via ESP-NOW...")
        send_command(ser, "ESP_NOW:ARM:FWD6", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop motor 6
        print("Stopping ARM motor 6...")
        send_command(ser, "ESP_NOW:ARM:FWD6", logger, wait_time=1.0)  # Toggle to stop
        time.sleep(1)
        
        # Test motor 6 (backward)
        print("Testing ARM motor 6 backward via ESP-NOW...")
        send_command(ser, "ESP_NOW:ARM:BACK6", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop motor 6
        print("Stopping ARM motor 6...")
        send_command(ser, "ESP_NOW:ARM:BACK6", logger, wait_time=1.0)  # Toggle to stop
        time.sleep(1)
        
        # Test motor 7 (forward)
        print("Testing ARM motor 7 forward via ESP-NOW...")
        send_command(ser, "ESP_NOW:ARM:FWD7", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop motor 7
        print("Stopping ARM motor 7...")
        send_command(ser, "ESP_NOW:ARM:FWD7", logger, wait_time=1.0)  # Toggle to stop
        time.sleep(1)
        
        # Test motor 7 (backward)
        print("Testing ARM motor 7 backward via ESP-NOW...")
        send_command(ser, "ESP_NOW:ARM:BACK7", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop motor 7
        print("Stopping ARM motor 7...")
        send_command(ser, "ESP_NOW:ARM:BACK7", logger, wait_time=1.0)  # Toggle to stop
        time.sleep(1)
        
        # Test gripper (open)
        print("Testing ARM gripper open via ESP-NOW...")
        send_command(ser, "ESP_NOW:ARM:RELEASE", logger, wait_time=1.0)
        time.sleep(3)
        
        # Test gripper (close)
        print("Testing ARM gripper close via ESP-NOW...")
        send_command(ser, "ESP_NOW:ARM:GRAB", logger, wait_time=1.0)
        time.sleep(3)
        
        print("ARM ESP-NOW test complete")
        return True
        
    except Exception as e:
        print(f"Error testing ARM via ESP-NOW: {e}")
        return False

def test_alternative_nav_commands(ser, logger=None):
    """Test alternative navigation motor commands that might work with specific ESP32 firmware"""
    print("\n=== Testing Alternative Navigation Commands ===")
    
    if not ser:
        print("Navigation controller not connected")
        return False
    
    try:
        # First clear any emergency state
        print("Clearing emergency status...")
        send_command(ser, "No_Emergency", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test alternative forward command
        print("Testing forward movement with M1_ON command (3 seconds)...")
        send_command(ser, "M1_ON", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop
        print("Stopping with M1_OFF...")
        send_command(ser, "M1_OFF", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test direct motor pin control
        print("Testing direct pin control for motors...")
        send_command(ser, "PIN:12:HIGH", logger, wait_time=0.5)  # Example pin number
        send_command(ser, "PIN:13:HIGH", logger, wait_time=0.5)  # Example pin number
        time.sleep(3)
        
        # Stop motors
        print("Stopping motors with direct pin control...")
        send_command(ser, "PIN:12:LOW", logger, wait_time=0.5)
        send_command(ser, "PIN:13:LOW", logger, wait_time=0.5)
        time.sleep(1)
        
        # Test ESP32 specific commands
        print("Testing ESP32-specific motor commands...")
        
        # Try different formats to see what works
        commands_to_try = [
            "MOTORS:FORWARD",
            "FORWARD",
            "MOVE_FORWARD",
            "GO_FORWARD",
            "MOTORS:ALL:ON",
            "MOTORS:ON",
            "DRIVE:FORWARD",
            "FWD",
            "MOTORS:FWD",
            "MOTORS:1:FWD,2:FWD,3:FWD,4:FWD"  # Try addressing individual motors
        ]
        
        for cmd in commands_to_try:
            print(f"Trying command: {cmd}")
            send_command(ser, cmd, logger, wait_time=1.0)
            time.sleep(2)
            
            # Stop after each command
            print("Stopping motors...")
            send_command(ser, "IR:OFF", logger, wait_time=1.0)
            time.sleep(1)
        
        print("Alternative command test complete")
        return True
        
    except Exception as e:
        print(f"Error testing alternative commands: {e}")
        return False
    finally:
        # Make sure motors are stopped
        try:
            send_command(ser, "IR:OFF", logger, wait_time=1.0)
            print("Motors stopped")
        except:
            print("Warning: Could not send final stop command")

def test_wasd_nav_commands(ser, logger=None):
    """Test WASD+X navigation commands"""
    print("\n=== Testing WASD+X Navigation Commands ===")
    
    if not ser:
        print("Navigation controller not connected")
        return False
    
    try:
        # First clear any emergency state
        print("Clearing emergency status...")
        send_command(ser, "No_Emergency", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test W - Forward
        print("Testing 'w' command (forward) for 3 seconds...")
        send_command(ser, "w", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop with X
        print("Stopping with 'x' command...")
        send_command(ser, "x", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test S - Backward
        print("Testing 's' command (backward) for 3 seconds...")
        send_command(ser, "s", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop with X
        print("Stopping with 'x' command...")
        send_command(ser, "x", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test A - Turn Left
        print("Testing 'a' command (turn left) for 3 seconds...")
        send_command(ser, "a", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop with X
        print("Stopping with 'x' command...")
        send_command(ser, "x", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test D - Turn Right
        print("Testing 'd' command (turn right) for 3 seconds...")
        send_command(ser, "d", logger, wait_time=1.0)
        time.sleep(3)
        
        # Stop with X
        print("Stopping with 'x' command...")
        send_command(ser, "x", logger, wait_time=1.0)
        time.sleep(1)
        
        # Test sequence of commands
        print("Testing sequence: forward, left, backward, right, stop")
        send_command(ser, "w", logger, wait_time=0.5)  # Forward
        time.sleep(2)
        send_command(ser, "a", logger, wait_time=0.5)  # Left
        time.sleep(2)
        send_command(ser, "s", logger, wait_time=0.5)  # Backward
        time.sleep(2)
        send_command(ser, "d", logger, wait_time=0.5)  # Right
        time.sleep(2)
        send_command(ser, "x", logger, wait_time=0.5)  # Stop
        
        print("WASD+X command test complete")
        return True
        
    except Exception as e:
        print(f"Error testing WASD+X commands: {e}")
        return False
    finally:
        # Make sure motors are stopped
        try:
            send_command(ser, "x", logger, wait_time=1.0)
            print("Motors stopped")
        except:
            print("Warning: Could not send final stop command")

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Test ESP32 motors directly')
    parser.add_argument('--nav', action='store_true', help='Test navigation motors')
    parser.add_argument('--arm', action='store_true', help='Test arm motors')
    parser.add_argument('--nav-port', help='Serial port for navigation controller')
    parser.add_argument('--arm-port', help='Serial port for arm controller')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    parser.add_argument('--esp-now', action='store_true', help='ARM controller communicates via ESP-NOW through NAV')
    parser.add_argument('--alt-commands', action='store_true', help='Try alternative motor control commands')
    parser.add_argument('--wasd', action='store_true', help='Test WASD+X navigation commands')
    args = parser.parse_args()
    
    # If no specific test is selected, test both
    if not args.nav and not args.arm and not args.wasd:
        args.nav = True
        args.arm = True
    
    # Set up logging
    logger = setup_logging(args.debug)
    
    # Find available ports
    available_ports = find_serial_ports()
    print(f"Available serial ports: {available_ports}")
    
    # Connect to navigation controller
    nav_ser = None
    if args.nav or args.wasd:
        if args.nav_port:
            nav_ser = connect_to_port(args.nav_port)
        else:
            # Try to auto-detect NAV controller
            for port in available_ports:
                if 'USB' in port or 'ACM' in port:
                    print(f"Trying {port} for navigation controller...")
                    nav_ser = connect_to_port(port)
                    if nav_ser:
                        # Test if this is the NAV controller
                        response = send_command(nav_ser, "IDENTIFY", logger)
                        if response and ("NAV" in response or "NAVIGATION" in response):
                            print(f"Found navigation controller on {port}")
                            break
                        else:
                            nav_ser.close()
                            nav_ser = None
            
            if not nav_ser:
                print("Could not auto-detect navigation controller. Please specify with --nav-port")
    
    # Connect to arm controller
    arm_ser = None
    if args.arm:
        if args.esp_now:
            print("ARM controller communicates via ESP-NOW through the NAV controller")
            arm_ser = nav_ser  # Use the same serial connection as NAV
        elif args.arm_port:
            arm_ser = connect_to_port(args.arm_port)
        else:
            # Try to auto-detect ARM controller (only if not using ESP-NOW)
            for port in available_ports:
                if port != (nav_ser.port if nav_ser else None) and ('USB' in port or 'ACM' in port):
                    print(f"Trying {port} for arm controller...")
                    arm_ser = connect_to_port(port)
                    if arm_ser:
                        # Test if this is the ARM controller
                        response = send_command(arm_ser, "IDENTIFY", logger)
                        if response and ("ARM" in response or "GRIPPER" in response):
                            print(f"Found arm controller on {port}")
                            break
                        else:
                            arm_ser.close()
                            arm_ser = None
            
            if not arm_ser and args.arm:
                print("Could not auto-detect arm controller. Please specify with --arm-port or use --esp-now if it communicates via ESP-NOW")
    
    # Test navigation motors
    if args.nav and nav_ser:
        if args.alt_commands:
            test_alternative_nav_commands(nav_ser, logger)
        else:
            test_nav_motors(nav_ser, logger)
    elif args.nav:
        print("Navigation controller not available for testing")
    
    # Test arm motors
    if args.arm and args.esp_now and nav_ser:
        test_esp_now_arm(nav_ser, logger)
    elif args.arm and arm_ser:
        test_arm_motors(arm_ser, logger)
    elif args.arm:
        print("Arm controller not available for testing")
    
    # Test WASD+X navigation commands
    if args.wasd and nav_ser:
        test_wasd_nav_commands(nav_ser, logger)
    elif args.wasd:
        print("Navigation controller not available for testing WASD+X commands")
    
    # Close serial ports
    if nav_ser:
        nav_ser.close()
    if arm_ser and arm_ser != nav_ser:  # Don't close twice if they're the same
        arm_ser.close()

if __name__ == "__main__":
    main() 