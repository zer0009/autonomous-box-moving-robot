#!/usr/bin/env python3
"""
Test script to verify the full system integration
"""

import subprocess
import time
import requests
import sys
import os

def test_full_system():
    """Test the full system with web server and robot controller"""
    print("Testing full system integration...")
    
    # Start web server
    print("Starting web server...")
    web_process = subprocess.Popen([sys.executable, "robot_web_server.py"])
    
    # Wait for web server to start
    time.sleep(3)
    
    # Test if web server is running
    try:
        response = requests.get("http://localhost:5000/api/status", timeout=5)
        if response.status_code == 200:
            print("✓ Web server is running")
        else:
            print("✗ Web server not responding")
            return False
    except Exception as e:
        print(f"✗ Cannot connect to web server: {e}")
        return False
    
    # Start robot controller
    print("Starting robot controller...")
    robot_process = subprocess.Popen([sys.executable, "robot_master_controller.py", "--debug"])
    
    # Wait for robot controller to start
    time.sleep(5)
    
    # Test sending commands
    test_commands = [
        {"command": "move", "params": {"direction": "forward"}},
        {"command": "turn", "params": {"direction": "left"}},
        {"command": "stop"}
    ]
    
    for i, command in enumerate(test_commands, 1):
        print(f"\nTest {i}: Sending command: {command}")
        
        try:
            # Send command to web server
            response = requests.post(
                "http://localhost:5000/api/send_command",
                json=command,
                timeout=5
            )
            
            if response.status_code == 200:
                result = response.json()
                print(f"✓ Command sent: {result}")
                
                # Wait a bit for robot to process
                time.sleep(2)
                
                # Check if command was processed
                response = requests.get("http://localhost:5000/api/check_commands", timeout=5)
                if response.status_code == 200:
                    data = response.json()
                    if not data.get('has_commands', False):
                        print("✓ Command was processed by robot")
                    else:
                        print("✗ Command still in queue")
                else:
                    print("✗ Cannot check command status")
                    
            else:
                print(f"✗ Failed to send command: {response.status_code}")
                
        except Exception as e:
            print(f"✗ Error sending command: {e}")
    
    print("\nFull system test completed!")
    
    # Cleanup
    print("Cleaning up...")
    try:
        robot_process.terminate()
        robot_process.wait(timeout=5)
    except:
        robot_process.kill()
    
    try:
        web_process.terminate()
        web_process.wait(timeout=5)
    except:
        web_process.kill()
    
    return True

if __name__ == "__main__":
    test_full_system() 