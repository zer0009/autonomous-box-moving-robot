#!/usr/bin/env python3
"""
Box-Moving Robot - Raspberry Pi Master Controller
Coordinates navigation and manipulation via UART to two ESP32 controllers
"""

import cv2
import sqlite3
import serial
import time
import json
import threading
import queue
from pyzbar import pyzbar
import numpy as np
from datetime import datetime
import heapq
import math
import random
import os
import logging
import glob
import signal
import sys
import requests
import functools

class RobotMasterController:
    def __init__(self):
        # Debug mode flag
        self.debug_mode = False
        
        # Set up logger
        self.logger = logging.getLogger("RobotController")
        
        # Check for controller mode flags
        self.nav_only_mode = os.environ.get('NAV_ONLY', '0') == '1'
        self.arm_only_mode = os.environ.get('ARM_ONLY', '0') == '1'
        
        # UART connections to ESP32 controllers with retry mechanism
        self.nav_uart = None
        self.arm_uart = None
        
        # Response queues for async communication
        self.nav_response_queue = queue.Queue()
        self.arm_response_queue = queue.Queue()
        
        self.connect_uart_devices()
        if self.nav_uart or self.arm_uart:
            self.test_esp32_communication()
        if self.nav_only_mode:
            print("Running in NAV ONLY MODE - Arm controller not required")
        if self.arm_only_mode:
            print("Running in ARM ONLY MODE - Navigation controller not required")
        self.camera = None
        self.setup_camera()
        self.current_position = (0, 0)  # Grid coordinates
        self.current_orientation = 0    # Degrees
        self.robot_busy = False
        self.obstacle_map = {}
        self.grid_size = 20  # 20x20 grid
        self.cell_size = 50  # 50mm per cell
        self.shelf_positions = {
            'SHELF_A': {
                'base': (18, 5),
                'sections': {
                    'A1': {'offset': (0, 0), 'capacity': 3, 'occupied': 0},
                    'A2': {'offset': (0, 1), 'capacity': 3, 'occupied': 0},
                    'A3': {'offset': (0, 2), 'capacity': 3, 'occupied': 0}
                }
            },
            'SHELF_B': {
                'base': (18, 10),
                'sections': {
                    'B1': {'offset': (0, 0), 'capacity': 3, 'occupied': 0},
                    'B2': {'offset': (0, 1), 'capacity': 3, 'occupied': 0},
                    'B3': {'offset': (0, 2), 'capacity': 3, 'occupied': 0}
                }
            },
            'SHELF_C': {
                'base': (18, 15),
                'sections': {
                    'C1': {'offset': (0, 0), 'capacity': 3, 'occupied': 0},
                    'C2': {'offset': (0, 1), 'capacity': 3, 'occupied': 0},
                    'C3': {'offset': (0, 2), 'capacity': 3, 'occupied': 0}
                }
            },
            'HOME': (1, 1)
        }
        self.box_queue = []
        self.max_box_capacity = 2  # Maximum boxes the robot can carry at once
        self.carrying_boxes = []  # List of boxes currently being carried
        self.init_database()
        if (self.nav_uart and self.arm_uart) or self.nav_only_mode or self.arm_only_mode:
            self.start_communication_threads()
            print("Robot Master Controller Initialized")
        else:
            print("Robot partially initialized - some hardware not connected")
    
    def connect_uart_devices(self):
        """Attempt to connect to ESP32 controllers via UART with retry"""
        # Check for environment variables specifying ports
        nav_port_env = os.environ.get('NAV_PORT')
        arm_port_env = os.environ.get('ARM_PORT')
        
        if nav_port_env or arm_port_env:
            print(f"Using ports specified in environment variables:")
            if nav_port_env:
                print(f"  Navigation controller: {nav_port_env}")
            if arm_port_env:
                print(f"  Arm controller: {arm_port_env}")
        
        # Get list of available serial ports
        available_ports = []
        
        # Check common Linux/Raspberry Pi serial ports
        linux_ports = glob.glob('/dev/tty*')
        available_ports.extend(linux_ports)
        
        # Add common port patterns
        uart_ports = [
            # Raspberry Pi ports
            '/dev/ttyAMA0', '/dev/ttyS0', 
            # USB-to-serial adapters on Raspberry Pi
            '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2',
            # CH340 and CP210x adapters common with ESP32
            '/dev/ttyACM0', '/dev/ttyACM1',
            # Windows ports (for development)
            'COM3', 'COM4', 'COM5', 'COM6', 'COM7'
        ]
        
        # Filter available ports to only include TTY devices
        available_tty_ports = [p for p in available_ports if 'tty' in p or 'COM' in p]
        
        # Log available ports
        if self.debug_mode:
            self.logger.debug(f"Available serial ports: {available_tty_ports}")
            
        print(f"Available serial ports: {available_tty_ports}")
        
        # Try to connect using environment variables first
        if nav_port_env and not self.arm_only_mode:
            try:
                print(f"Connecting to navigation controller on {nav_port_env} (from environment)...")
                self.nav_uart = serial.Serial(nav_port_env, 9600, timeout=1)
                # Test if port is actually usable
                if self.test_port_connection(self.nav_uart, "NAV"):
                    print(f"Connected to navigation controller on {nav_port_env}")
                else:
                    print(f"Port {nav_port_env} is not responding as navigation controller")
                    self.nav_uart.close()
                    self.nav_uart = None
            except Exception as e:
                print(f"Failed to connect to {nav_port_env}: {e}")
                self.nav_uart = None
        
        if arm_port_env and not self.nav_only_mode:
            try:
                print(f"Connecting to arm controller on {arm_port_env} (from environment)...")
                self.arm_uart = serial.Serial(arm_port_env, 9600, timeout=1)
                # Test if port is actually usable
                if self.test_port_connection(self.arm_uart, "ARM"):
                    print(f"Connected to arm controller on {arm_port_env}")
                else:
                    print(f"Port {arm_port_env} is not responding as arm controller")
                    self.arm_uart.close()
                    self.arm_uart = None
            except Exception as e:
                print(f"Failed to connect to {arm_port_env}: {e}")
                self.arm_uart = None
        
        # If we still need to detect devices, proceed with auto-detection
        if (not self.nav_uart and not self.arm_only_mode) or (not self.arm_uart and not self.nav_only_mode):
            # First, try to auto-detect ESP32 devices
            print("Attempting to auto-detect ESP32 devices...")
            
            # Try to identify devices on available ports
            for port in available_tty_ports:
                if (self.nav_uart and self.arm_uart) or \
                   (self.nav_uart and self.nav_only_mode) or \
                   (self.arm_uart and self.arm_only_mode):
                    break  # Both required controllers found
                    
                try:
                    device_type, ser = self.identify_esp32_device(port)
                    
                    if device_type == "NAV" and not self.nav_uart and not self.arm_only_mode:
                        self.nav_uart = ser
                        print(f"Auto-detected navigation controller on {port}")
                    elif device_type == "ARM" and not self.arm_uart and not self.nav_only_mode:
                        self.arm_uart = ser
                        print(f"Auto-detected arm controller on {port}")
                    elif device_type == "UNKNOWN" and ser:
                        ser.close()  # Close the port if we couldn't identify the device
                except Exception as e:
                    print(f"Error checking port {port}: {e}")
            
            # If auto-detection failed, try manual connection
            if (not self.nav_uart and not self.arm_only_mode) or (not self.arm_uart and not self.nav_only_mode):
                print("Auto-detection incomplete, trying manual connection...")
                
                # Try to connect to navigation controller if not already connected and needed
                if not self.nav_uart and not self.arm_only_mode:
                    for port in uart_ports:
                        if port in available_tty_ports:  # Only try ports that actually exist
                            try:
                                print(f"Trying navigation controller on {port}...")
                                test_uart = serial.Serial(port, 9600, timeout=1)
                                if self.test_port_connection(test_uart, "NAV"):
                                    self.nav_uart = test_uart
                                    print(f"Connected to navigation controller on {port}")
                                    break
                                else:
                                    test_uart.close()
                            except Exception as e:
                                print(f"Failed to connect to {port}: {e}")
                
                # Try to connect to arm controller if not already connected and needed
                if not self.arm_uart and not self.nav_only_mode:
                    arm_ports = [p for p in uart_ports if p != (self.nav_uart.port if self.nav_uart else None)]
                    for port in arm_ports:
                        if port in available_tty_ports:  # Only try ports that actually exist
                            try:
                                print(f"Trying arm controller on {port}...")
                                test_uart = serial.Serial(port, 9600, timeout=1)
                                if self.test_port_connection(test_uart, "ARM"):
                                    self.arm_uart = test_uart
                                    print(f"Connected to arm controller on {port}")
                                    break
                                else:
                                    test_uart.close()
                            except Exception as e:
                                print(f"Failed to connect to {port}: {e}")
        
        # Check if required connections established
        if not self.nav_uart and not self.arm_only_mode:
            print("WARNING: Navigation controller not connected!")
        if not self.arm_uart and not self.nav_only_mode:
            print("WARNING: Arm controller not connected!")
    
    def test_port_connection(self, uart, controller_type):
        """Test if a port is actually connected to the expected controller"""
        if not uart:
            return False
            
        try:
            # Clear any pending data
            uart.reset_input_buffer()
            uart.reset_output_buffer()
            time.sleep(0.5)  # Give device time to process
            
            # Send test command based on controller type
            if controller_type == "NAV":
                # First try IDENTIFY command
                uart.write("IDENTIFY\n".encode())
                time.sleep(0.5)
                
                # Check for response
                if uart.in_waiting > 0:
                    response = uart.readline().decode(errors='replace').strip()
                    if "NAV" in response or "NAVIGATION" in response:
                        return True
                    
                # Clear buffer and try simple movement command (stop)
                uart.reset_input_buffer()
                uart.write("x\n".encode())
                time.sleep(0.5)
                
                # Even if no response, consider it a success if no errors
                # Most ESP32 NAV controllers don't respond to basic movement commands
                return True
                    
            elif controller_type == "ARM":
                # First try IDENTIFY command
                uart.write("IDENTIFY\n".encode())
                time.sleep(0.5)
                
                # Check for response
                if uart.in_waiting > 0:
                    response = uart.readline().decode(errors='replace').strip()
                    if "ARM" in response or "GRIPPER" in response:
                        return True
                
                # Try Enable Arm command (z)
                uart.reset_input_buffer()
                uart.write("z\n".encode())  # Enable arm command
                time.sleep(0.5)
                
                # Check for any response
                if uart.in_waiting > 0:
                    return True
                    
                # Try Open Gripper command (i)
                uart.reset_input_buffer()
                uart.write("i\n".encode())  # Open gripper command
                time.sleep(0.5)
                
                # Check for any response
                if uart.in_waiting > 0:
                    return True
            
            # If we got here, no valid response was received
            return False
            
        except Exception as e:
            print(f"Error testing port connection: {e}")
            return False
    
    def setup_camera(self):
        """Setup camera with fallback options for USB cameras"""
        # Try different camera indices - expanded range to include higher indices
        # First check standard indices 0-2
        for camera_index in [0, 1, 2]:
            try:
                print(f"Trying camera at index {camera_index}...")
                camera = cv2.VideoCapture(camera_index)
                if camera.isOpened():
                    ret, test_frame = camera.read()
                    if ret and test_frame is not None:
                        self.camera = camera
                        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        print(f"Connected to camera at index {camera_index}")
                        print(f"Camera resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
                        return
                    else:
                        print(f"Camera at index {camera_index} opened but couldn't read frame")
                        camera.release()
                else:
                    print(f"Failed to open camera at index {camera_index}")
            except Exception as e:
                print(f"Error with camera at index {camera_index}: {e}")
        
        # If standard indices failed, try higher indices (19-35) that appear on some systems
        for camera_index in range(19, 36):
            try:
                print(f"Trying camera at higher index {camera_index}...")
                camera = cv2.VideoCapture(camera_index)
                if camera.isOpened():
                    ret, test_frame = camera.read()
                    if ret and test_frame is not None:
                        self.camera = camera
                        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        print(f"Connected to camera at index {camera_index}")
                        print(f"Camera resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
                        return
                    else:
                        print(f"Camera at index {camera_index} opened but couldn't read frame")
                        camera.release()
                else:
                    print(f"Failed to open camera at index {camera_index}")
            except Exception as e:
                print(f"Error with camera at index {camera_index}: {e}")
        
        # Try direct device path for video devices
        for device_id in range(19, 36):
            device_path = f"/dev/video{device_id}"
            try:
                if os.path.exists(device_path):
                    print(f"Trying camera at device path {device_path}...")
                    camera = cv2.VideoCapture(device_path)
                    if camera.isOpened():
                        ret, test_frame = camera.read()
                        if ret and test_frame is not None:
                            self.camera = camera
                            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                            print(f"Connected to camera at device path {device_path}")
                            print(f"Camera resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
                            return
                        else:
                            print(f"Camera at device path {device_path} opened but couldn't read frame")
                            camera.release()
                    else:
                        print(f"Failed to open camera at device path {device_path}")
            except Exception as e:
                print(f"Error with camera at device path {device_path}: {e}")
        
        # Try Raspberry Pi camera module if available
        if not self.camera:
            try:
                print("Trying Raspberry Pi camera module...")
                # Special case for Raspberry Pi camera
                gst_str = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"
                camera = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
                
                if not camera.isOpened():
                    # Try another common Raspberry Pi camera pipeline
                    print("Trying alternative Raspberry Pi camera pipeline...")
                    camera.release()
                    camera = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
                
                if camera.isOpened():
                    ret, test_frame = camera.read()
                    if ret and test_frame is not None:
                        self.camera = camera
                        print("Connected to Raspberry Pi camera module")
                        print(f"Camera resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
                    else:
                        print("Raspberry Pi camera opened but couldn't read frame")
                        camera.release()
                else:
                    print("Failed to open Raspberry Pi camera module")
            except Exception as e:
                print(f"Error with Raspberry Pi camera: {e}")
        
        if not self.camera:
            print("WARNING: No camera connected! QR code functionality disabled.")
            print("Check camera connections and permissions.")
            print("On Raspberry Pi, ensure the camera is enabled with 'sudo raspi-config'")
            print("Check permissions with 'ls -l /dev/video*'")
            print("Available video devices: " + ", ".join([f"/dev/video{i}" for i in range(19, 36) if os.path.exists(f"/dev/video{i}")]))
    
    def init_database(self):
        """Initialize SQLite database for tracking boxes and tasks"""
        self.db = sqlite3.connect('robot_tasks.db', check_same_thread=False)
        cursor = self.db.cursor()
        
        # Create tables
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS boxes (
                id TEXT PRIMARY KEY,
                status TEXT DEFAULT 'pending',
                source_position TEXT,
                destination_shelf TEXT,
                destination_section TEXT,
                stack_position INTEGER DEFAULT 0,
                pickup_time TIMESTAMP,
                delivery_time TIMESTAMP,
                weight REAL,
                attempts INTEGER DEFAULT 0
            )
        ''')
        
        # Check if stack_position column exists, add it if not
        cursor.execute("PRAGMA table_info(boxes)")
        columns = [column[1] for column in cursor.fetchall()]
        if 'stack_position' not in columns:
            cursor.execute('ALTER TABLE boxes ADD COLUMN stack_position INTEGER DEFAULT 0')
            print("Added missing stack_position column to boxes table")
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS robot_log (
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                event_type TEXT,
                description TEXT,
                position TEXT,
                success BOOLEAN
            )
        ''')
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS shelf_sections (
                shelf_id TEXT,
                section_id TEXT,
                capacity INTEGER,
                occupied INTEGER DEFAULT 0,
                last_updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                PRIMARY KEY (shelf_id, section_id)
            )
        ''')
        
        # Initialize shelf sections in database if not already populated
        for shelf_id, shelf_data in self.shelf_positions.items():
            if isinstance(shelf_data, tuple):  # Skip HOME which is just coordinates
                continue
            
            for section_id, section_data in shelf_data['sections'].items():
                cursor.execute('''
                    INSERT OR IGNORE INTO shelf_sections (shelf_id, section_id, capacity, occupied)
                    VALUES (?, ?, ?, ?)
                ''', (shelf_id, section_id, section_data['capacity'], section_data['occupied']))
        
        self.db.commit()
    
    def log_event(self, event_type, description, success=True):
        """Log robot events to database"""
        cursor = self.db.cursor()
        cursor.execute('''
            INSERT INTO robot_log (event_type, description, position, success)
            VALUES (?, ?, ?, ?)
        ''', (event_type, description, str(self.current_position), success))
        self.db.commit()
    
    def start_communication_threads(self):
        """Start threads for UART communication"""
        if self.nav_uart:
            nav_thread = threading.Thread(target=self.nav_communication_handler, daemon=True)
            nav_thread.start()
            
        if self.arm_uart:
            arm_thread = threading.Thread(target=self.arm_communication_handler, daemon=True)
            arm_thread.start()
    
    def nav_communication_handler(self):
        """Handle navigation ESP32 communication"""
        reconnect_attempts = 0
        max_reconnect_attempts = 5
        last_reconnect_time = 0
        reconnect_delay = 5  # seconds between reconnection attempts
        
        # List of commands that should be blocked for NAV controller
        blocked_commands = ['g', 'h']  # These are gripper commands
        
        while True:
            try:
                if self.nav_uart and self.nav_uart.in_waiting > 0:
                    try:
                        # Read a line from the NAV controller
                        line = self.nav_uart.readline()
                        # Try to decode as text
                        response = line.decode(errors='replace').strip()
                        
                        # Process only if there's actual content
                        if response:
                            # Check if response contains any blocked commands
                            if any(cmd in response for cmd in blocked_commands):
                                if self.debug_mode:
                                    self.logger.warning(f"Blocked command in NAV response: {response}")
                                # Clear input buffer to prevent command execution
                                self.nav_uart.reset_input_buffer()
                            else:
                                if self.debug_mode:
                                    self.logger.debug(f"NAV RECV: {response}")
                                self.nav_response_queue.put(response)
                                # Reset reconnection counter on successful read
                                reconnect_attempts = 0
                    except UnicodeDecodeError:
                        # Sometimes we get binary garbage from the serial port
                        if self.debug_mode:
                            self.logger.warning("Received non-text data from NAV controller")
                        # Clear the buffer
                        self.nav_uart.reset_input_buffer()
            except (serial.SerialException, OSError) as e:
                # Handle serial port errors
                current_time = time.time()
                if current_time - last_reconnect_time > reconnect_delay:
                    reconnect_attempts += 1
                    last_reconnect_time = current_time
                    
                    if reconnect_attempts <= max_reconnect_attempts:
                        print(f"Navigation controller connection lost: {e}")
                        print(f"Attempting to reconnect (attempt {reconnect_attempts}/{max_reconnect_attempts})...")
                        
                        # Close the port if it's still open
                        try:
                            if self.nav_uart:
                                self.nav_uart.close()
                        except:
                            pass
                            
                        # Try to reconnect
                        self.nav_uart = None
                        if not self.arm_only_mode:
                            self.connect_uart_devices()
                            
                        # If reconnected, reset counter
                        if self.nav_uart:
                            print("Successfully reconnected to navigation controller")
                            reconnect_attempts = 0
                    else:
                        if self.nav_uart:
                            try:
                                self.nav_uart.close()
                            except:
                                pass
                            self.nav_uart = None
                        print("Failed to reconnect to navigation controller after multiple attempts")
                        # Only log once per minute
                        if current_time % 60 < 1:
                            self.log_event("ERROR", "Navigation controller disconnected", False)
            except Exception as e:
                if self.debug_mode:
                    self.logger.error(f"Nav UART error: {e}")
                else:
                    # Only print every 30 seconds to avoid flooding the console
                    if time.time() % 30 < 0.1:
                        print(f"Nav UART error: {e}")
            
            time.sleep(0.01)
    
    def arm_communication_handler(self):
        """Handle arm ESP32 communication"""
        reconnect_attempts = 0
        max_reconnect_attempts = 5
        last_reconnect_time = 0
        reconnect_delay = 5  # seconds between reconnection attempts
        
        # Track time of last gripper command for special handling
        last_gripper_command_time = 0
        gripper_commands = ["RELEASE", "GRAB"]
        
        while True:
            try:
                if self.arm_uart and self.arm_uart.in_waiting:
                    try:
                        response = self.arm_uart.readline().decode(errors='replace').strip()
                        if response:
                            if self.debug_mode:
                                self.logger.debug(f"ARM RECV: {response}")
                            self.arm_response_queue.put(response)
                            # Reset reconnection counter on successful read
                            reconnect_attempts = 0
                    except UnicodeDecodeError:
                        # Sometimes we get binary garbage from the serial port
                        if self.debug_mode:
                            self.logger.warning("Received non-text data from ARM controller")
                        # Clear the buffer
                        self.arm_uart.reset_input_buffer()
            except (serial.SerialException, OSError) as e:
                # Handle serial port errors
                current_time = time.time()
                if current_time - last_reconnect_time > reconnect_delay:
                    reconnect_attempts += 1
                    last_reconnect_time = current_time
                    
                    if reconnect_attempts <= max_reconnect_attempts:
                        print(f"Arm controller connection lost: {e}")
                        print(f"Attempting to reconnect (attempt {reconnect_attempts}/{max_reconnect_attempts})...")
                        
                        # Close the port if it's still open
                        try:
                            if self.arm_uart:
                                self.arm_uart.close()
                        except:
                            pass
                            
                        # Try to reconnect
                        self.arm_uart = None
                        if not self.nav_only_mode:
                            self.connect_uart_devices()
                    else:
                        if self.arm_uart:
                            try:
                                self.arm_uart.close()
                            except:
                                pass
                            self.arm_uart = None
                        print("Failed to reconnect to arm controller after multiple attempts")
                        # Only log once per minute
                        if current_time % 60 < 1:
                            self.log_event("ERROR", "Arm controller disconnected", False)
            except Exception as e:
                if self.debug_mode:
                    self.logger.error(f"Arm UART error: {e}")
                else:
                    # Only print every 30 seconds to avoid flooding the console
                    if time.time() % 30 < 0.1:
                        print(f"Arm UART error: {e}")
            
            time.sleep(0.01)
    
    def send_nav_command(self, action, param1="", param2="", timeout=10):
        """Send command to navigation ESP32 or simulate response"""
        # Format command according to the ESP32 NAV code
        # Updated to use w,a,s,d,x commands
        if action == "MOVE":
            if param1 == "FORWARD":
                # Forward movement - w
                command = "w\n"
            elif param1 == "BACKWARD":
                # Backward movement - s
                command = "s\n"
            else:
                # Default to forward
                command = "w\n"
        elif action == "STOP":
            # Stop - x
            command = "x\n"
        elif action == "TURN_LEFT" or (action == "TURN" and param1 == "LEFT"):
            # Turn left - a
            command = "a\n"
        elif action == "TURN_RIGHT" or (action == "TURN" and param1 == "RIGHT"):
            # Turn right - d
            command = "d\n"
        else:
            # Default format for other commands
            command = f"{action}\n"
            
        # Safety check - block 'g' and 'h' commands which should only go to the arm controller
        if command.strip() in ['g', 'h']:
            print(f"WARNING: Attempt to send gripper command '{command.strip()}' to navigation controller blocked")
            return "ERROR:WRONG_CONTROLLER"
            
        # Log command if in debug mode
        if self.debug_mode:
            self.logger.debug(f"NAV SEND: {command.strip()}")
            
        if not self.nav_uart:
            print("Navigation controller not connected!")
            return "ERROR:NOT_CONNECTED"
        
        try:
            # Clear input buffer before sending command
            self.nav_uart.reset_input_buffer()
            
            # Send the command
            self.nav_uart.write(command.encode())
            print(f"Sent command to NAV: {command.strip()}")
            
            # Wait for response
            start_time = time.time()
            response_received = False
            response = ""
            
            while time.time() - start_time < timeout:
                try:
                    if self.nav_uart.in_waiting > 0:
                        line = self.nav_uart.readline().decode(errors='replace').strip()
                        if line:
                            if self.debug_mode:
                                self.logger.debug(f"NAV RESPONSE: {line}")
                            print(f"NAV response: {line}")
                            response = line
                            response_received = True
                            break
                    else:
                        # For simple movement commands, we might not get a response
                        if command.strip() in ['w', 'a', 's', 'd', 'x']:
                            if time.time() - start_time > 0.5:  # Wait at least 0.5 sec for response
                                return "OK"  # Assume success for movement commands
                        # Wait a bit before checking again
                        time.sleep(0.1)
                except (serial.SerialException, OSError) as e:
                    # Handle serial port errors
                    print(f"Navigation controller error during command: {e}")
                    # Try to reconnect
                    try:
                        if self.nav_uart:
                            self.nav_uart.close()
                    except:
                        pass
                    self.nav_uart = None
                    if not self.arm_only_mode:
                        self.connect_uart_devices()
                    return "ERROR:CONNECTION_LOST"
                except Exception as e:
                    if self.debug_mode:
                        self.logger.error(f"Error reading from NAV: {e}")
                    break
            
            if response_received:
                return response
            
            # For basic movement commands, assume success even without response
            if command.strip() in ['w', 'a', 's', 'd', 'x']:
                return "OK"
                
            if self.debug_mode:
                self.logger.warning(f"NAV TIMEOUT: No response received for {command.strip()}")
            return "TIMEOUT"
        except (serial.SerialException, OSError) as e:
            # Handle serial port errors
            print(f"Navigation controller error sending command: {e}")
            # Try to reconnect
            try:
                if self.nav_uart:
                    self.nav_uart.close()
            except:
                pass
            self.nav_uart = None
            if not self.arm_only_mode:
                self.connect_uart_devices()
            return "ERROR:CONNECTION_LOST"
        except Exception as e:
            if self.debug_mode:
                self.logger.error(f"Error sending command to NAV: {e}")
            return f"ERROR:{str(e)}"
    
    def send_arm_command(self, action, param1="", param2="", timeout=15):
        """Send command to arm ESP32 or simulate response"""
        # New command mappings based on the Flask app
        # Enable Arm: 'z', Disable Arm: 'x', Base +: 'w', Base -: 's'
        # Shoulder +: 'a', Shoulder -: 'd', Elbow +: 'q', Elbow -: 'e'
        # Gripper Open: 'i', Gripper Close: 'o'
        
        # Format command according to the new ESP32 ARM code
        if action == "GRIP":
            if param1 == "OPEN":
                command = "i\n"  # Open gripper
            else:
                command = "o\n"  # Close gripper
        elif action == "ENABLE":
            command = "z\n"  # Enable arm
        elif action == "DISABLE":
            command = "x\n"  # Disable arm
        elif action == "MOVE_MOTOR":
            # Format for moving specific motors
            motor_num = param1  # Motor number (base, shoulder, elbow)
            direction = param2  # + or -
            
            if motor_num == "5" or motor_num.lower() == "base":
                if direction == "FWD" or direction == "+":
                    command = "w\n"  # Base +
                else:
                    command = "s\n"  # Base -
            elif motor_num == "6" or motor_num.lower() == "shoulder":
                if direction == "FWD" or direction == "+":
                    command = "a\n"  # Shoulder +
                else:
                    command = "d\n"  # Shoulder -
            elif motor_num == "7" or motor_num.lower() == "elbow":
                if direction == "FWD" or direction == "+":
                    command = "q\n"  # Elbow +
                else:
                    command = "e\n"  # Elbow -
            else:
                # Default command if motor not recognized
                command = "x\n"  # Disable arm as a safe default
        else:
            # Legacy command format - convert to new format
            if action == "RELEASE":
                command = "i\n"  # Open gripper
            elif action == "GRAB":
                command = "o\n"  # Close gripper
            elif action == "HOME":
                # Home position - first enable arm then return to default position
                command = "z\n"  # Enable arm first
            elif action == "STOP" or action == "EMERGENCY":
                command = "x\n"  # Disable arm
            elif action == "CLEAR_EMERGENCY" or action == "No_Emergency":
                command = "z\n"  # Enable arm
            else:
                # Default format for other commands
                command = f"{action}\n"
        
        # Log command if in debug mode
        if self.debug_mode:
            self.logger.debug(f"ARM SEND: {command.strip()}")
        
        # Check if arm controller is connected
        if not self.arm_uart:
            print("Arm controller not connected!")
            return "ERROR:NOT_CONNECTED"
            
        try:
            # Clear input buffer before sending command
            self.arm_uart.reset_input_buffer()
                
            # Send the command
            self.arm_uart.write(command.encode())
            print(f"Sent command to ARM: {command.strip()}")
            
            # Gripper commands often don't get a response, so handle them specially
            if command.strip() in ["i", "o"]:
                # Wait a short time for any immediate response
                time.sleep(0.5)
                
                # Check for any response but don't require one
                if self.arm_uart.in_waiting > 0:
                    response = self.arm_uart.readline().decode(errors='replace').strip()
                    if self.debug_mode:
                        self.logger.debug(f"ARM RESPONSE: {response}")
                    return response if response else "OK"
                else:
                    # For gripper commands, no response is common and acceptable
                    return "OK"
            
            # For other commands, wait for response
            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    if self.arm_uart and self.arm_uart.in_waiting > 0:
                        response = self.arm_uart.readline().decode(errors='replace').strip()
                        if self.debug_mode:
                            self.logger.debug(f"ARM RESPONSE: {response}")
                        
                        # If we got a response, return it
                        if response:
                            return response
                    else:
                        # Wait a bit before checking again
                        time.sleep(0.1)
                except (serial.SerialException, OSError) as e:
                    # Handle serial port errors
                    print(f"Arm controller error during command: {e}")
                    # Try to reconnect
                    try:
                        if self.arm_uart:
                            self.arm_uart.close()
                    except:
                        pass
                    self.arm_uart = None
                    if not self.nav_only_mode:
                        self.connect_uart_devices()
                    return "ERROR:CONNECTION_LOST"
                except Exception as e:
                    if self.debug_mode:
                        self.logger.error(f"Error reading from ARM: {e}")
                    break
            
            if self.debug_mode:
                self.logger.warning(f"ARM TIMEOUT: No response received for {command.strip()}")
            return "TIMEOUT"
        except (serial.SerialException, OSError) as e:
            # Handle serial port errors
            print(f"Arm controller error sending command: {e}")
            # Try to reconnect
            try:
                if self.arm_uart:
                    self.arm_uart.close()
            except:
                pass
            self.arm_uart = None
            if not self.nav_only_mode:
                self.connect_uart_devices()
            return "ERROR:CONNECTION_LOST"
        except Exception as e:
            if self.debug_mode:
                self.logger.error(f"Error sending command to ARM: {e}")
            return f"ERROR:{str(e)}"
    
    def check_controller_status(self):
        """Check if ESP32 controllers are still responsive"""
        nav_status = False
        arm_status = False
        
        # Check navigation controller
        if self.nav_uart:
            try:
                # Send a simple stop command to check if controller is responsive
                test_cmd = "x\n"
                self.nav_uart.reset_input_buffer()
                self.nav_uart.write(test_cmd.encode())
                time.sleep(0.2)
                
                # For navigation controller, we don't always get a response for basic commands
                # So consider it responsive if we can write to it without errors
                nav_status = True
                
                # If there is a response, read it to clear the buffer
                if self.nav_uart.in_waiting > 0:
                    response = self.nav_uart.readline().decode(errors='replace').strip()
                    if self.debug_mode:
                        self.logger.debug(f"NAV status response: {response}")
            except Exception as e:
                nav_status = False
                if self.debug_mode:
                    self.logger.error(f"Error checking NAV status: {e}")
        
        # Check arm controller
        if self.arm_uart:
            try:
                # Send a simple command to check if controller is responsive
                test_cmd = "z\n"  # Enable arm command
                self.arm_uart.reset_input_buffer()
                self.arm_uart.write(test_cmd.encode())
                time.sleep(0.2)
                
                # Check for any response
                if self.arm_uart.in_waiting > 0:
                    arm_status = True
                    # Clear any response
                    response = self.arm_uart.readline().decode(errors='replace').strip()
                    if self.debug_mode:
                        self.logger.debug(f"ARM status response: {response}")
                else:
                    # Try a second command
                    self.arm_uart.write("i\n".encode())  # Open gripper command
                    time.sleep(0.2)
                    if self.arm_uart.in_waiting > 0:
                        arm_status = True
                        self.arm_uart.reset_input_buffer()  # Clear any response
            except Exception as e:
                arm_status = False
                if self.debug_mode:
                    self.logger.error(f"Error checking ARM status: {e}")
        
        return {
            "nav_controller_responsive": nav_status,
            "arm_controller_responsive": arm_status
        }

    def status_update_thread(self):
        """Thread to periodically update web server status"""
        # Use a session for connection pooling and better performance
        session = requests.Session()
        
        # Set default timeout for all requests in this session
        session.request = functools.partial(session.request, timeout=0.5)
        
        # Track consecutive failures to avoid flooding logs
        consecutive_failures = 0
        max_consecutive_failures = 10
        
        while True:
            try:
                # Get current status
                status = self.get_status_report()
                
                # Use a separate thread for the actual request to avoid blocking
                def send_status_update():
                    nonlocal consecutive_failures
                    try:
                        response = session.post(
                            'http://localhost:5000/api/update_robot_status',
                            json=status,
                            timeout=0.5
                        )
                        
                        if response.status_code == 200:
                            consecutive_failures = 0  # Reset failure counter on success
                    except requests.exceptions.Timeout:
                        # Timeout is expected sometimes, just continue
                        pass
                    except requests.exceptions.ConnectionError:
                        # Web server might not be running yet
                        consecutive_failures += 1
                        if consecutive_failures <= max_consecutive_failures:
                            if self.debug_mode or consecutive_failures == 1:
                                print(f"Web server connection error (attempt {consecutive_failures})")
                    except Exception as e:
                        consecutive_failures += 1
                        if consecutive_failures <= max_consecutive_failures:
                            if self.debug_mode or consecutive_failures == 1:
                                print(f"Web server communication error: {e}")
                
                # Run the request in a separate thread with a timeout
                update_thread = threading.Thread(target=send_status_update, daemon=True)
                update_thread.start()
                update_thread.join(timeout=1.0)  # Wait max 1 second for thread to complete
                
            except Exception as e:
                if self.debug_mode:
                    print(f"Status update error: {e}")
                
            # Update every 2 seconds
            time.sleep(2)

    def clear_command_buffers(self):
        """Clear command buffers for both controllers to reset state"""
        try:
            if self.nav_uart:
                self.nav_uart.reset_input_buffer()
                self.nav_uart.reset_output_buffer()
                print("Navigation controller buffers cleared")
                
            if self.arm_uart:
                self.arm_uart.reset_input_buffer()
                self.arm_uart.reset_output_buffer()
                print("Arm controller buffers cleared")
                
            # Also clear response queues
            while not self.nav_response_queue.empty():
                try:
                    self.nav_response_queue.get_nowait()
                except:
                    pass
                    
            while not self.arm_response_queue.empty():
                try:
                    self.arm_response_queue.get_nowait()
                except:
                    pass
                    
            return True
        except Exception as e:
            print(f"Error clearing command buffers: {e}")
            return False

    def check_web_commands(self):
        """Check for commands from the web server"""
        try:
            import requests
            response = requests.get('http://localhost:5000/api/check_commands', timeout=1)
            if response.status_code == 200:
                data = response.json()
                if data.get('has_commands', False):
                    # Clear buffers before processing new commands to prevent backlog
                    self.clear_command_buffers()
                    
                    command = data.get('commands')
                    if command:
                        print(f"Executing web command: {command}")
                        self.execute_web_command(command)
                        return True
        except Exception as e:
            # Only log if debug mode is enabled to avoid flooding
            if self.debug_mode:
                print(f"Error checking web commands: {e}")
        return False

    def identify_esp32_device(self, port):
        """Try to identify if a port is connected to a NAV or ARM ESP32 controller"""
        try:
            # Try to open the port
            ser = serial.Serial(port, 9600, timeout=1)
            time.sleep(0.5)  # Give device time to initialize
            
            # Clear any pending data
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Send identify command
            ser.write("IDENTIFY\n".encode())
            time.sleep(0.5)
            
            # Check for response
            if ser.in_waiting > 0:
                response = ser.readline().decode(errors='replace').strip().upper()
                if "NAV" in response or "NAVIGATION" in response:
                    print(f"Identified navigation controller on {port}")
                    return "NAV", ser
                elif "ARM" in response or "GRIPPER" in response:
                    print(f"Identified arm controller on {port}")
                    return "ARM", ser
            
            # If no response to IDENTIFY, try specific commands
            # Try NAV-specific command
            ser.reset_input_buffer()
            ser.write("x\n".encode())  # Stop command for navigation
            time.sleep(0.5)
            
            # Try ARM-specific command
            ser.reset_input_buffer()
            ser.write("z\n".encode())  # Enable arm command
            time.sleep(0.5)
            if ser.in_waiting > 0:
                response = ser.readline().decode(errors='replace').strip()
                if response:
                    print(f"Got response from {port} after arm command: {response}")
                    return "ARM", ser
            
            # Try another ARM-specific command
            ser.reset_input_buffer()
            ser.write("i\n".encode())  # Open gripper command
            time.sleep(0.5)
            if ser.in_waiting > 0:
                response = ser.readline().decode(errors='replace').strip()
                if response:
                    print(f"Got response from {port} after gripper command: {response}")
                    return "ARM", ser
            
            # If we got here, couldn't identify the device
            print(f"Could not identify device on {port}")
            return "UNKNOWN", ser
            
        except Exception as e:
            print(f"Error checking port {port}: {e}")
            return "ERROR", None
    
    def test_esp32_communication(self):
        """Test communication with ESP32 controllers"""
        print("Testing communication with ESP32 controllers...")
        
        # Test navigation controller if connected
        if self.nav_uart and not self.arm_only_mode:
            print("Testing navigation controller...")
            try:
                # Send stop command to test communication
                response = self.send_nav_command("STOP")
                if "ERROR" in response:
                    print(f"Navigation controller test failed: {response}")
                else:
                    print("Navigation controller test successful")
            except Exception as e:
                print(f"Navigation controller test error: {e}")
        
        # Test arm controller if connected
        if self.arm_uart and not self.nav_only_mode:
            print("Testing arm controller...")
            try:
                # First enable the arm
                response = self.send_arm_command("ENABLE")
                if "ERROR" in response:
                    print(f"Arm controller enable test failed: {response}")
                else:
                    print("Arm controller enable test successful")
                    
                # Then test gripper
                time.sleep(0.5)
                response = self.send_arm_command("GRIP", "OPEN")
                if "ERROR" in response:
                    print(f"Arm controller gripper test failed: {response}")
                else:
                    print("Arm controller gripper test successful")
            except Exception as e:
                print(f"Arm controller test error: {e}")
        
        print("ESP32 communication test completed")

    def main_loop(self):
        """Main operation loop for the robot"""
        print("Starting robot main loop...")
        
        # Start status update thread
        status_thread = threading.Thread(target=self.status_update_thread, daemon=True)
        status_thread.start()
        
        try:
            while True:
                # Check for web commands
                self.check_web_commands()
                
                # Check controller status periodically
                if time.time() % 30 < 0.1:  # Every 30 seconds
                    status = self.check_controller_status()
                    if self.debug_mode:
                        print(f"Controller status: {status}")
                
                # Sleep to avoid high CPU usage
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received, shutting down...")
        except Exception as e:
            print(f"Error in main loop: {e}")
            self.log_event("ERROR", f"Main loop error: {e}", False)
    
    def shutdown(self):
        """Clean shutdown of the robot controller"""
        print("Shutting down robot controller...")
        
        # Stop any ongoing movement
        if self.nav_uart:
            try:
                self.send_nav_command("STOP")
                print("Navigation stopped")
            except:
                pass
        
        # Disable the arm
        if self.arm_uart:
            try:
                self.send_arm_command("DISABLE")
                print("Arm disabled")
            except:
                pass
        
        # Close serial connections
        if self.nav_uart:
            try:
                self.nav_uart.close()
                print("Navigation controller connection closed")
            except:
                pass
            
        if self.arm_uart:
            try:
                self.arm_uart.close()
                print("Arm controller connection closed")
            except:
                pass
        
        # Close camera if open
        if self.camera:
            try:
                self.camera.release()
                print("Camera released")
            except:
                pass
        
        # Close database connection
        try:
            self.db.close()
            print("Database connection closed")
        except:
            pass
            
        print("Shutdown complete")
    
    def execute_web_command(self, command):
        """Execute commands received from the web interface"""
        if not command:
            return False
            
        cmd = command.get('command', '').lower()
        params = command.get('params', {})
        
        print(f"Executing web command: {cmd} with params: {params}")
        
        try:
            if cmd == 'move':
                direction = params.get('direction', 'forward')
                if direction == 'forward':
                    response = self.send_nav_command("MOVE", "FORWARD")
                elif direction == 'backward':
                    response = self.send_nav_command("MOVE", "BACKWARD")
                else:
                    response = "ERROR:INVALID_DIRECTION"
                    
            elif cmd == 'turn':
                direction = params.get('direction', 'left')
                if direction == 'left':
                    response = self.send_nav_command("TURN", "LEFT")
                elif direction == 'right':
                    response = self.send_nav_command("TURN", "RIGHT")
                else:
                    response = "ERROR:INVALID_DIRECTION"
                    
            elif cmd == 'stop':
                response = self.send_nav_command("STOP")
                
            elif cmd == 'emergency_stop':
                # Stop navigation
                nav_response = self.send_nav_command("STOP")
                # Disable arm
                arm_response = self.send_arm_command("DISABLE")
                response = f"NAV:{nav_response}, ARM:{arm_response}"
                
            elif cmd == 'clear_emergency':
                # Enable arm
                response = self.send_arm_command("ENABLE")
                
            elif cmd == 'home':
                # This would involve path planning and navigation
                response = "HOME command not fully implemented"
                
            elif cmd == 'arm_move':
                # Direct arm command
                arm_cmd = params.get('arm_cmd', '')
                if arm_cmd:
                    response = self.send_arm_command(arm_cmd)
                else:
                    response = "ERROR:MISSING_ARM_COMMAND"
                    
            else:
                response = f"ERROR:UNKNOWN_COMMAND:{cmd}"
                
            print(f"Command response: {response}")
            return True
            
        except Exception as e:
            print(f"Error executing web command: {e}")
            self.log_event("ERROR", f"Web command error: {cmd} - {e}", False)
            return False
    
    def get_status_report(self):
        """Generate a status report for the web interface"""
        # Check controller responsiveness
        controller_status = self.check_controller_status()
        
        # Get task counts from database
        try:
            conn = self.db
            cursor = conn.cursor()
            pending = cursor.execute('SELECT COUNT(*) FROM boxes WHERE status != "delivered"').fetchone()[0]
            completed = cursor.execute('SELECT COUNT(*) FROM boxes WHERE status = "delivered"').fetchone()[0]
        except Exception as e:
            if self.debug_mode:
                print(f"Error getting task counts: {e}")
            pending = 0
            completed = 0
        
        # Compile status report
        status = {
            "current_position": self.current_position,
            "current_orientation": self.current_orientation,
            "robot_busy": self.robot_busy,
            "pending_tasks": pending,
            "completed_tasks": completed,
            "nav_controller_connected": self.nav_uart is not None,
            "arm_controller_connected": self.arm_uart is not None,
            "nav_controller_responsive": controller_status.get("nav_controller_responsive", False),
            "arm_controller_responsive": controller_status.get("arm_controller_responsive", False),
            "nav_only_mode": self.nav_only_mode,
            "arm_only_mode": self.arm_only_mode,
            "carrying_boxes": len(self.carrying_boxes),
            "max_box_capacity": self.max_box_capacity
        }
        
        return status

    def test_arm_motors(self):
        """Test each arm motor with a short movement sequence"""
        if not self.arm_uart:
            print("Arm controller not connected")
            return False
            
        print("Testing arm motors...")
        
        try:
            # First enable the arm
            print("Enabling arm...")
            response = self.send_arm_command("ENABLE")
            if "ERROR" in response:
                print(f"Failed to enable arm: {response}")
                return False
                
            time.sleep(1)
            
            # Test base motor
            print("Testing base motor...")
            self.send_arm_command("MOVE_MOTOR", "base", "+")
            time.sleep(0.5)
            self.send_arm_command("MOVE_MOTOR", "base", "-")
            time.sleep(0.5)
            
            # Test shoulder motor
            print("Testing shoulder motor...")
            self.send_arm_command("MOVE_MOTOR", "shoulder", "+")
            time.sleep(0.5)
            self.send_arm_command("MOVE_MOTOR", "shoulder", "-")
            time.sleep(0.5)
            
            # Test elbow motor
            print("Testing elbow motor...")
            self.send_arm_command("MOVE_MOTOR", "elbow", "+")
            time.sleep(0.5)
            self.send_arm_command("MOVE_MOTOR", "elbow", "-")
            time.sleep(0.5)
            
            # Test gripper
            print("Testing gripper...")
            self.send_arm_command("GRIP", "OPEN")
            time.sleep(1)
            self.send_arm_command("GRIP", "CLOSE")
            time.sleep(1)
            self.send_arm_command("GRIP", "OPEN")
            time.sleep(1)
            
            # Disable arm when done
            print("Disabling arm...")
            self.send_arm_command("DISABLE")
            
            print("Arm motor test completed successfully")
            return True
            
        except Exception as e:
            print(f"Error testing arm motors: {e}")
            # Try to disable arm in case of error
            try:
                self.send_arm_command("DISABLE")
            except:
                pass
            return False

if __name__ == "__main__":
    robot = None
    
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Robot Master Controller')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging for ESP32 communication')
    args = parser.parse_args()
    
    # Configure debug logging if enabled
    if args.debug:
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler("robot_controller_debug.log"),
                logging.StreamHandler()
            ]
        )
        logger = logging.getLogger("RobotController")
        logger.info("Debug logging enabled")
    
    try:
        robot = RobotMasterController()
        
        # Set debug mode if enabled
        if args.debug:
            robot.debug_mode = True
            print("Debug mode enabled - verbose ESP32 communication logging")
        
        # Check if essential components are connected
        if not robot.camera:
            print("WARNING: Running without camera - QR code detection disabled")
            
        if (not robot.nav_uart or not robot.arm_uart):
            print("WARNING: Running with limited functionality")
            print("Continuing with available hardware...")
        
        # Start main operation loop
        robot.main_loop()
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Failed to start robot: {e}")
    finally:
        # Cleanup
        if robot:
            robot.shutdown()