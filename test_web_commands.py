#!/usr/bin/env python3
"""
Test script to verify web command functionality
"""

import requests
import time
import json

def test_web_commands():
    """Test sending commands to the web server"""
    base_url = "http://localhost:5000"
    
    print("Testing web command functionality...")
    
    # Test 1: Check if web server is running
    try:
        response = requests.get(f"{base_url}/api/status", timeout=5)
        if response.status_code == 200:
            print("✓ Web server is running")
            status = response.json()
            print(f"  Robot status: {status}")
        else:
            print(f"✗ Web server returned status code: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Cannot connect to web server: {e}")
        return False
    
    # Test 2: Send a move command
    try:
        command_data = {
            "command": "move",
            "params": {"direction": "forward"}
        }
        
        print(f"\nSending command: {command_data}")
        response = requests.post(
            f"{base_url}/api/send_command",
            json=command_data,
            timeout=5
        )
        
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Command sent successfully: {result}")
            command_id = result.get('command_id')
        else:
            print(f"✗ Failed to send command: {response.status_code}")
            return False
            
    except Exception as e:
        print(f"✗ Error sending command: {e}")
        return False
    
    # Test 3: Check if command is in queue
    try:
        response = requests.get(f"{base_url}/api/check_commands", timeout=5)
        if response.status_code == 200:
            data = response.json()
            if data.get('has_commands', False):
                print(f"✓ Command found in queue: {data.get('commands')}")
            else:
                print("✗ No commands found in queue")
        else:
            print(f"✗ Failed to check commands: {response.status_code}")
            
    except Exception as e:
        print(f"✗ Error checking commands: {e}")
    
    # Test 4: Send a turn command
    try:
        command_data = {
            "command": "turn",
            "params": {"direction": "left"}
        }
        
        print(f"\nSending command: {command_data}")
        response = requests.post(
            f"{base_url}/api/send_command",
            json=command_data,
            timeout=5
        )
        
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Turn command sent successfully: {result}")
        else:
            print(f"✗ Failed to send turn command: {response.status_code}")
            
    except Exception as e:
        print(f"✗ Error sending turn command: {e}")
    
    # Test 5: Send a stop command
    try:
        command_data = {
            "command": "stop"
        }
        
        print(f"\nSending command: {command_data}")
        response = requests.post(
            f"{base_url}/api/send_command",
            json=command_data,
            timeout=5
        )
        
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Stop command sent successfully: {result}")
        else:
            print(f"✗ Failed to send stop command: {response.status_code}")
            
    except Exception as e:
        print(f"✗ Error sending stop command: {e}")
    
    print("\nWeb command test completed!")
    return True

if __name__ == "__main__":
    test_web_commands() 