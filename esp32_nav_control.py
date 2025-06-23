#!/usr/bin/env python3
"""
ESP32 Navigation Controller - Direct Control Script
Simple script to control ESP32 navigation controller via serial port
Designed for Raspberry Pi with /dev/ttyUSB0
"""

import serial
import time
import sys
import os
import threading
import signal

class ESP32NavControl:
    def __init__(self, port="/dev/ttyUSB0", baud=9600):
        """Initialize ESP32 navigation controller connection"""
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.response_thread = None
        
        print(f"ESP32 Navigation Controller Control")
        print(f"Port: {port}, Baud: {baud}")
        print("Commands: w(forward), s(backward), a(left), d(right), x(stop), q(quit)")
        print("=" * 50)
    
    def connect(self):
        """Connect to ESP32 navigation controller"""
        try:
            print(f"Connecting to {self.port}...")
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            
            # Clear any startup data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(2)  # Give ESP32 time to stabilize
            
            print(f"✓ Connected to {self.port}")
            
            # Start response monitoring thread
            self.running = True
            self.response_thread = threading.Thread(target=self.monitor_responses, daemon=True)
            self.response_thread.start()
            
            return True
            
        except Exception as e:
            print(f"✗ Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from ESP32"""
        self.running = False
        
        if self.ser and self.ser.is_open:
            # Send stop command before disconnecting
            self.send_command("x")
            time.sleep(0.5)
            
            self.ser.close()
            print("✓ Disconnected from ESP32")
    
    def send_command(self, command):
        """Send a command to the ESP32"""
        if not self.ser or not self.ser.is_open:
            print("✗ Not connected to ESP32")
            return False
        
        try:
            # Format command
            if command in ['w', 'a', 's', 'd', 'x']:
                cmd = f"{command}\n"
            else:
                cmd = f"{command}\n"
            
            # Send command
            self.ser.write(cmd.encode())
            print(f"→ Sent: {command}")
            return True
            
        except Exception as e:
            print(f"✗ Error sending command '{command}': {e}")
            return False
    
    def monitor_responses(self):
        """Monitor responses from ESP32 in background thread"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    response = self.ser.readline().decode().strip()
                    if response:
                        print(f"← ESP32: {response}")
                time.sleep(0.1)
            except Exception as e:
                if self.running:
                    print(f"✗ Error reading from ESP32: {e}")
                break
    
    def test_connection(self):
        """Test the connection with a simple command"""
        print("Testing connection...")
        
        # Send identify command
        self.send_command("IDENTIFY")
        time.sleep(1)
        
        # Send stop command to ensure motors are off
        self.send_command("x")
        time.sleep(0.5)
        
        print("Connection test completed")
    
    def run_interactive(self):
        """Run interactive command loop"""
        print("Enter commands (w/a/s/d/x/q):")
        
        while True:
            try:
                # Get user input
                command = input("> ").strip().lower()
                
                if command == 'q':
                    print("Quitting...")
                    break
                elif command in ['w', 'a', 's', 'd', 'x']:
                    self.send_command(command)
                elif command == 'test':
                    self.test_connection()
                elif command == 'help':
                    print("Commands:")
                    print("  w - Move forward")
                    print("  s - Move backward") 
                    print("  a - Turn left")
                    print("  d - Turn right")
                    print("  x - Stop")
                    print("  test - Test connection")
                    print("  help - Show this help")
                    print("  q - Quit")
                elif command:
                    print(f"Unknown command: {command}")
                    print("Use w/a/s/d/x for movement, 'help' for commands, 'q' to quit")
                    
            except KeyboardInterrupt:
                print("\nInterrupted by user")
                break
            except EOFError:
                print("\nEnd of input")
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def run_sequence(self, sequence, delay=1.0):
        """Run a sequence of commands with delays"""
        print(f"Running sequence: {sequence}")
        
        for cmd in sequence:
            if cmd in ['w', 'a', 's', 'd', 'x']:
                self.send_command(cmd)
                time.sleep(delay)
            else:
                print(f"Skipping invalid command: {cmd}")
        
        print("Sequence completed")

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='ESP32 Navigation Controller Control')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=9600, help='Baud rate (default: 9600)')
    parser.add_argument('--test', action='store_true', help='Run connection test and exit')
    parser.add_argument('--sequence', help='Run command sequence (e.g., "w,a,s,d,x")')
    parser.add_argument('--delay', type=float, default=1.0, help='Delay between commands in sequence (default: 1.0s)')
    parser.add_argument('--interactive', action='store_true', help='Run in interactive mode (default)')
    
    args = parser.parse_args()
    
    # Create controller
    controller = ESP32NavControl(port=args.port, baud=args.baud)
    
    # Set up signal handler for graceful shutdown
    def signal_handler(sig, frame):
        print("\nShutting down...")
        controller.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Connect to ESP32
        if not controller.connect():
            print("Failed to connect. Exiting.")
            sys.exit(1)
        
        # Run based on arguments
        if args.test:
            controller.test_connection()
        elif args.sequence:
            # Parse sequence
            sequence = [cmd.strip() for cmd in args.sequence.split(',')]
            controller.run_sequence(sequence, args.delay)
        else:
            # Default to interactive mode
            controller.run_interactive()
    
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.disconnect()

if __name__ == "__main__":
    main() 