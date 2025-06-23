#!/usr/bin/env python3
"""
Test script to verify robot controller web command processing
"""

import sys
import os

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_master_controller import RobotMasterController

def test_robot_controller():
    """Test the robot controller's web command processing"""
    print("Testing robot controller web command processing...")
    
    try:
        # Create robot controller instance
        robot = RobotMasterController()
        print("✓ Robot controller created successfully")
        
        # Test web command processing
        test_commands = [
            {
                "command": "move",
                "params": {"direction": "forward"}
            },
            {
                "command": "turn", 
                "params": {"direction": "left"}
            },
            {
                "command": "stop"
            }
        ]
        
        for i, command in enumerate(test_commands, 1):
            print(f"\nTest {i}: Processing command: {command}")
            try:
                robot.execute_web_command(command)
                print(f"✓ Command processed successfully")
            except Exception as e:
                print(f"✗ Error processing command: {e}")
        
        print("\nRobot controller test completed!")
        
    except Exception as e:
        print(f"✗ Error creating robot controller: {e}")
        return False
    
    return True

if __name__ == "__main__":
    test_robot_controller() 