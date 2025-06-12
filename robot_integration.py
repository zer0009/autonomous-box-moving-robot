#!/usr/bin/env python3
"""
Box-Moving Robot - Integration Module
Connects the robot master controller with the web server
Enables remote control and status reporting
"""

import threading
import time
import json
import requests
import os
import sys
import argparse
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("robot_integration.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

class RobotIntegration:
    def __init__(self, server_url="http://localhost:5000", robot_controller=None):
        """Initialize integration between robot controller and web server"""
        self.server_url = server_url
        self.robot_controller = robot_controller
        self.running = False
        self.status_thread = None
        self.command_thread = None
        self.update_interval = 3  # seconds
        self.last_status_update = None
        
        logger.info(f"Robot integration initializing, connecting to server at {server_url}")
    
    def start(self):
        """Start integration threads"""
        if self.running:
            return
            
        self.running = True
        
        # Start status reporting thread
        self.status_thread = threading.Thread(target=self.status_reporting_loop)
        self.status_thread.daemon = True
        self.status_thread.start()
        
        # Start command processing thread
        self.command_thread = threading.Thread(target=self.command_processing_loop)
        self.command_thread.daemon = True
        self.command_thread.start()
        
        logger.info("Robot integration started")
    
    def stop(self):
        """Stop integration threads"""
        self.running = False
        
        if self.status_thread:
            self.status_thread.join(timeout=2)
        
        if self.command_thread:
            self.command_thread.join(timeout=2)
            
        logger.info("Robot integration stopped")
    
    def status_reporting_loop(self):
        """Thread to periodically report robot status to web server"""
        while self.running:
            try:
                self.report_status()
                time.sleep(self.update_interval)
            except Exception as e:
                logger.error(f"Error in status reporting: {e}")
                time.sleep(5)  # Retry after delay
    
    def command_processing_loop(self):
        """Thread to periodically check for new commands from web server"""
        while self.running:
            try:
                commands = self.check_for_commands()
                if commands:
                    self.process_command(commands)
                time.sleep(1)
            except Exception as e:
                logger.error(f"Error in command processing: {e}")
                time.sleep(5)  # Retry after delay
    
    def report_status(self):
        """Report current robot status to web server"""
        if self.robot_controller:
            # Get actual status from robot controller
            status = self.robot_controller.get_status_report()
        else:
            # Mock status for testing
            status = {
                'current_position': (5, 5),
                'current_orientation': 90,
                'robot_busy': False,
                'pending_tasks': 2,
                'completed_tasks': 5,
                'obstacle_count': 0,
                'recent_logs': []
            }
        
        try:
            response = requests.post(
                f"{self.server_url}/api/update_robot_status", 
                json=status,
                timeout=5
            )
            
            if response.status_code == 200:
                self.last_status_update = datetime.now()
                logger.debug("Status reported successfully")
                
                # Check if there are commands to process
                data = response.json()
                if data.get('has_commands', False):
                    return data.get('commands')
                    
            else:
                logger.warning(f"Failed to report status: {response.status_code}")
                
        except requests.exceptions.RequestException as e:
            logger.error(f"Connection error reporting status: {e}")
        
        return None
    
    def check_for_commands(self):
        """Check for new commands from the web server"""
        # The status reporting already checks for commands
        # This is a backup method that can be called separately
        try:
            response = requests.get(
                f"{self.server_url}/api/check_commands", 
                timeout=5
            )
            
            if response.status_code == 200:
                data = response.json()
                if data.get('has_commands', False):
                    return data.get('commands')
                    
        except requests.exceptions.RequestException as e:
            logger.debug(f"Connection error checking commands: {e}")
        
        return None
    
    def process_command(self, command):
        """Process a command received from the web server"""
        if not command:
            return
            
        command_type = command.get('command')
        params = command.get('params', {})
        
        logger.info(f"Processing command: {command_type} with params: {params}")
        
        try:
            if self.robot_controller:
                # Execute command on real robot controller
                if command_type == 'move':
                    direction = params.get('direction', 'forward')
                    distance = params.get('distance', 100)
                    
                    if direction == 'forward':
                        result = self.robot_controller.send_nav_command("MOVE", "FORWARD", str(distance))
                    elif direction == 'backward':
                        result = self.robot_controller.send_nav_command("MOVE", "BACKWARD", str(distance))
                    else:
                        result = "ERROR:INVALID_DIRECTION"
                        
                elif command_type == 'turn':
                    direction = params.get('direction', 'left')
                    angle = params.get('angle', 90)
                    
                    result = self.robot_controller.send_nav_command("TURN", direction.upper(), str(angle))
                
                elif command_type == 'stop':
                    result = self.robot_controller.send_nav_command("STOP")
                
                elif command_type == 'home':
                    # Navigate to home position
                    home_pos = self.robot_controller.shelf_positions.get('HOME', (1, 1))
                    self.robot_controller.navigate_to_position(home_pos)
                    result = "SUCCESS:NAVIGATING_TO_HOME"
                
                elif command_type == 'emergency_stop':
                    result = self.robot_controller.emergency_stop()
                    
                else:
                    result = f"ERROR:UNKNOWN_COMMAND:{command_type}"
                
            else:
                # Mock execution for testing
                logger.info(f"Mock execution of {command_type}")
                result = "SUCCESS:MOCK_EXECUTION"
            
            # Report command result
            self.report_command_result(command, result)
            
        except Exception as e:
            logger.error(f"Error executing command {command_type}: {e}")
            self.report_command_result(command, f"ERROR:{str(e)}")
    
    def report_command_result(self, command, result):
        """Report the result of a command execution back to the web server"""
        try:
            response = requests.post(
                f"{self.server_url}/api/command_result", 
                json={
                    'command_id': command.get('timestamp', 0),
                    'command': command.get('command'),
                    'result': result,
                    'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                },
                timeout=5
            )
            
            if response.status_code == 200:
                logger.debug(f"Command result reported: {result}")
            else:
                logger.warning(f"Failed to report command result: {response.status_code}")
                
        except requests.exceptions.RequestException as e:
            logger.error(f"Connection error reporting command result: {e}")

def connect_to_robot_controller():
    """Try to import and connect to the real robot controller"""
    try:
        # Add parent directory to path if needed
        parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        if parent_dir not in sys.path:
            sys.path.append(parent_dir)
            
        # Try to import the robot controller
        from robot_master_controller import RobotMasterController
        
        # Create controller instance
        controller = RobotMasterController()
        logger.info("Successfully connected to robot master controller")
        return controller
        
    except ImportError:
        logger.warning("Could not import robot_master_controller, running in simulation mode")
        return None
    except Exception as e:
        logger.error(f"Error connecting to robot controller: {e}")
        return None

def main():
    parser = argparse.ArgumentParser(description='Robot Web Integration')
    parser.add_argument('--server', default='http://localhost:5000', help='Web server URL')
    parser.add_argument('--interval', type=int, default=3, help='Status update interval in seconds')
    parser.add_argument('--simulate', action='store_true', help='Run in simulation mode without robot controller')
    
    args = parser.parse_args()
    
    # Connect to robot controller if not in simulation mode
    controller = None if args.simulate else connect_to_robot_controller()
    
    # Create and start integration
    integration = RobotIntegration(server_url=args.server, robot_controller=controller)
    integration.update_interval = args.interval
    
    try:
        integration.start()
        
        # Keep the main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, stopping...")
    finally:
        integration.stop()
        
        # Shutdown robot controller if connected
        if controller:
            controller.shutdown()

if __name__ == "__main__":
    main() 