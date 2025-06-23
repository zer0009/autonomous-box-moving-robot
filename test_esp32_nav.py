#!/usr/bin/env python3
"""
Test script for ESP32 Navigation Controller
Tests basic WASD commands and connection
"""

import time
import sys
from esp32_nav_control import ESP32NavControl

def test_esp32_nav():
    """Test ESP32 navigation controller"""
    print("ESP32 Navigation Controller Test")
    print("=" * 40)
    
    # Create controller
    controller = ESP32NavControl(port="/dev/ttyUSB0")
    
    try:
        # Connect to ESP32
        if not controller.connect():
            print("Failed to connect to ESP32!")
            return False
        
        print("✓ Connected to ESP32")
        
        # Test connection
        print("\n1. Testing connection...")
        controller.test_connection()
        time.sleep(1)
        
        # Test individual commands
        print("\n2. Testing individual commands...")
        
        commands = [
            ("w", "Forward"),
            ("s", "Backward"), 
            ("a", "Left"),
            ("d", "Right"),
            ("x", "Stop")
        ]
        
        for cmd, description in commands:
            print(f"\nTesting {description} ({cmd})...")
            controller.send_command(cmd)
            time.sleep(2)  # Let motor run for 2 seconds
            controller.send_command("x")  # Stop
            time.sleep(1)  # Wait before next command
        
        # Test command sequence
        print("\n3. Testing command sequence...")
        sequence = ["w", "a", "s", "d", "x"]
        controller.run_sequence(sequence, delay=1.5)
        
        print("\n✓ All tests completed successfully!")
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        return False
    finally:
        controller.disconnect()

if __name__ == "__main__":
    success = test_esp32_nav()
    sys.exit(0 if success else 1) 