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
                
                # Try No_Emergency command
                uart.reset_input_buffer()
                uart.write("No_Emergency\n".encode())
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
        
        while True:
            try:
                if self.nav_uart and self.nav_uart.in_waiting > 0:
                    try:
                        response = self.nav_uart.readline().decode(errors='replace').strip()
                        if response:
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
        
        while True:
            try:
                if self.arm_uart and self.arm_uart.in_waiting:
                    response = self.arm_uart.readline().decode().strip()
                    if self.debug_mode:
                        self.logger.debug(f"ARM RECV: {response}")
                    self.arm_response_queue.put(response)
                    # Reset reconnection counter on successful read
                    reconnect_attempts = 0
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
        # Format command according to the ESP32 ARM code
        # Based on the logs, we need to send direct commands that match the ESP32 code
        if action == "GRIP":
            if param1 == "OPEN":
                command = "RELEASE\n"  # Release command
            else:
                command = "GRAB\n"  # Grab command
        elif action == "CHECK_IR":
            command = "CHECK_IR\n"  # Check IR sensors
        elif action == "EMERGENCY":
            command = "Emergency\n"  # Emergency stop
        elif action == "CLEAR_EMERGENCY":
            command = "No_Emergency\n"  # Clear emergency
        elif action == "MOVE_MOTOR":
            # Format for moving specific motors
            motor_num = param1  # Motor number (5, 6, or 7)
            direction = param2  # FWD or BACK
            command = f"{direction}{motor_num}\n"
        else:
            # Default format for other commands
            command = f"{action}\n"
        
        # Log command if in debug mode
        if self.debug_mode:
            self.logger.debug(f"ARM SEND: {command.strip()}")
        
        try:
            # Clear input buffer before sending command
            if self.arm_uart:
                self.arm_uart.reset_input_buffer()
                
            # Send the command
            self.arm_uart.write(command.encode())
            
            # Wait for response
            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    if self.arm_uart and self.arm_uart.in_waiting:
                        response = self.arm_uart.readline().decode().strip()
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
    
    def scan_qr_codes(self):
        """Scan for QR codes in camera view"""
        if not self.camera:
            return []
            
        try:
            ret, frame = self.camera.read()
            if not ret or frame is None:
                print("Failed to read frame from camera")
                return []
            
            # Decode QR codes
            qr_codes = pyzbar.decode(frame)
            detected_codes = []
            
            for qr in qr_codes:
                data = qr.data.decode('utf-8')
                # Get QR code center position in image
                x = qr.rect.left + qr.rect.width // 2
                y = qr.rect.top + qr.rect.height // 2
                
                detected_codes.append({
                    'data': data,
                    'position': (x, y),
                    'type': self.classify_qr_code(data)
                })
            
            return detected_codes
        except Exception as e:
            print(f"Error scanning QR codes: {e}")
            return []
    
    def classify_qr_code(self, data):
        """Classify QR code type based on data"""
        if data.startswith('BOX_'):
            return 'box'
        elif data.startswith('SHELF_'):
            return 'shelf'
        elif data.startswith('FLOOR_'):
            return 'floor_marker'
        else:
            return 'unknown'
    
    def parse_box_qr(self, qr_data):
        """Parse box QR code data"""
        # Updated format: BOX_ID123_SHELF_A_SECTION_A2_WEIGHT_2.5
        parts = qr_data.split('_')
        if len(parts) >= 6:
            # Extract the full box ID (which might contain underscores)
            if parts[0] == "BOX" and "SHELF" in qr_data:
                # Find the index where SHELF starts
                shelf_index = parts.index("SHELF")
                # Combine all parts between BOX and SHELF for the ID
                box_id = "_".join(parts[1:shelf_index])
                
                # Get shelf letter
                shelf_letter = parts[shelf_index + 1]
                destination = f"SHELF_{shelf_letter}"
                
                # Get section
                section_index = parts.index("SECTION") if "SECTION" in parts else -1
                if section_index != -1 and section_index + 1 < len(parts):
                    section = parts[section_index + 1]
                else:
                    section = f"{shelf_letter}1"  # Default to section 1
                
                # Get weight
                weight = 1.0  # Default weight
                weight_index = parts.index("WEIGHT") if "WEIGHT" in parts else -1
                if weight_index != -1 and weight_index + 1 < len(parts):
                    try:
                        weight = float(parts[weight_index + 1])
                    except ValueError:
                        print(f"Warning: Could not convert weight '{parts[weight_index + 1]}' to float, using default 1.0")
                
                return {'id': box_id, 'destination': destination, 'section': section, 'weight': weight}
        
        # Handle legacy format
        elif len(parts) >= 4:
            box_id = parts[1]
            destination = f"SHELF_{parts[3]}"
            section = f"{parts[3]}1"  # Default to section 1
            
            # Make sure weight is parsed correctly
            weight = 1.0  # Default weight
            if len(parts) > 5 and parts[4] == "WEIGHT":
                try:
                    weight = float(parts[5])
                except ValueError:
                    print(f"Warning: Could not convert weight '{parts[5]}' to float, using default 1.0")
                
            return {'id': box_id, 'destination': destination, 'section': section, 'weight': weight}
        
        print(f"Invalid QR code format: {qr_data}")
        return None
    
    def find_available_section(self, shelf_id):
        """Find an available section in the specified shelf"""
        cursor = self.db.cursor()
        cursor.execute('''
            SELECT section_id, capacity, occupied 
            FROM shelf_sections 
            WHERE shelf_id = ? 
            ORDER BY occupied
        ''', (shelf_id,))
        
        sections = cursor.fetchall()
        
        for section_id, capacity, occupied in sections:
            if occupied < capacity:
                return section_id
        
        return None  # No available sections
    
    def process_box_task(self, box_qr_data):
        """Process complete box moving task"""
        box_info = self.parse_box_qr(box_qr_data)
        if not box_info:
            print("Invalid box QR code")
            return False
        
        # Check if we can pick up another box
        if len(self.carrying_boxes) >= self.max_box_capacity:
            print(f"Already carrying maximum capacity of {self.max_box_capacity} boxes")
            return False
        
        # If no specific section provided, find an available one
        if 'section' not in box_info or not box_info['section']:
            shelf_id = box_info['destination']
            box_info['section'] = self.find_available_section(shelf_id) or f"{shelf_id[-1]}1"  # Default to section 1
        
        # Add to database
        cursor = self.db.cursor()
        cursor.execute('''
            INSERT OR REPLACE INTO boxes 
            (id, destination_shelf, destination_section, source_position, weight)
            VALUES (?, ?, ?, ?, ?)
        ''', (box_info['id'], box_info['destination'], box_info['section'], 
              str(self.current_position), box_info['weight']))
        self.db.commit()
        
        # Execute pickup
        if self.execute_pickup_sequence(box_info):
            # Add to carrying boxes list
            self.carrying_boxes.append(box_info)
            
            # If we've reached capacity or it's a priority delivery, deliver immediately
            if len(self.carrying_boxes) >= self.max_box_capacity:
                return self.deliver_all_boxes()
            else:
                print(f"Box {box_info['id']} picked up, capacity: {len(self.carrying_boxes)}/{self.max_box_capacity}")
                return True
        else:
            self.log_event("PICKUP_FAILED", f"Failed to pick up box {box_info['id']}", False)
        
        return False
    
    def execute_pickup_sequence(self, box_info):
        """Execute complete box pickup sequence"""
        print(f"Executing pickup for box {box_info['id']}")
        
        # Position robot for optimal pickup angle
        response = self.send_nav_command("POSITION", "PICKUP")
        if "SUCCESS" not in response:
            return False
        
        # Initialize arm to home position if not already there
        response = self.send_arm_command("HOME")
        if "SUCCESS" not in response:
            print("Arm home position failed")
            return False
        
        # Calculate pickup coordinates (adjust based on QR position)
        pickup_x = 150  # mm from robot center
        pickup_y = 0    # mm from robot center
        
        # Adjust height based on already carrying boxes
        height_offset = len(self.carrying_boxes) * 50  # 50mm per box height
        
        # Execute pickup sequence
        response = self.send_arm_command("PICK", str(pickup_x), str(pickup_y), str(height_offset))
        if "SUCCESS" in response:
            # Confirm grip
            grip_status = self.send_arm_command("GRIP", "STATUS")
            if "GRIPPED" in grip_status:
                print("Box successfully picked up")
                
                # Move to transport position
                self.send_arm_command("TRANSPORT")
                
                # Update database
                cursor = self.db.cursor()
                cursor.execute('''
                    UPDATE boxes SET status = 'picked', pickup_time = ?, stack_position = ?
                    WHERE id = ?
                ''', (datetime.now(), len(self.carrying_boxes), box_info['id']))
                self.db.commit()
                
                return True
        
        print("Pickup failed")
        return False
    
    def execute_delivery_sequence(self, box_info):
        """Execute complete box delivery sequence"""
        print(f"Executing delivery for box {box_info['id']}")
        
        # Navigate to destination shelf
        shelf_pos = self.shelf_positions.get(box_info['destination'])
        if not shelf_pos:
            print(f"Unknown shelf: {box_info['destination']}")
            return False
        
        if not self.navigate_to_position(shelf_pos):
            print("Navigation to shelf failed")
            return False
        
        # Position for placement
        response = self.send_nav_command("POSITION", "PLACE")
        if "SUCCESS" not in response:
            return False
        
        # Calculate placement coordinates
        place_x = 200  # mm from robot center
        place_y = 0    # mm from robot center
        
        # Execute placement
        response = self.send_arm_command("PLACE", str(place_x), str(place_y))
        if "SUCCESS" in response:
            # Open gripper
            self.send_arm_command("GRIP", "OPEN")
            
            # Return to home position
            self.send_arm_command("HOME")
            
            # Update database
            cursor = self.db.cursor()
            cursor.execute('''
                UPDATE boxes SET status = 'delivered', delivery_time = ?
                WHERE id = ?
            ''', (datetime.now(), box_info['id']))
            self.db.commit()
            
            print(f"Box {box_info['id']} delivered successfully")
            return True
        
        print("Delivery failed")
        return False
    
    def deliver_all_boxes(self):
        """Deliver all boxes currently being carried"""
        if not self.carrying_boxes:
            print("No boxes to deliver")
            return True
        
        # Group boxes by destination shelf to optimize delivery
        boxes_by_shelf = {}
        for box in self.carrying_boxes:
            shelf = box['destination']
            if shelf not in boxes_by_shelf:
                boxes_by_shelf[shelf] = []
            boxes_by_shelf[shelf].append(box)
        
        # Deliver boxes shelf by shelf
        success = True
        for shelf, boxes in boxes_by_shelf.items():
            # Navigate to shelf position
            shelf_data = self.shelf_positions.get(shelf)
            if not shelf_data:
                print(f"Unknown shelf: {shelf}")
                success = False
                continue
            
            # Get base position for shelf
            if isinstance(shelf_data, tuple):
                # Legacy format
                shelf_pos = shelf_data
            else:
                # New format with sections
                shelf_pos = shelf_data['base']
            
            if not self.navigate_to_position(shelf_pos):
                print(f"Navigation to shelf {shelf} failed")
                success = False
                continue
            
            # Position for placement
            response = self.send_nav_command("POSITION", "PLACE")
            
            # Deliver each box to its section
            for box in boxes:
                section = box['section']
                section_data = None
                
                # Get section data if available
                if not isinstance(shelf_data, tuple) and section in shelf_data['sections']:
                    section_data = shelf_data['sections'][section]
                
                # Calculate placement coordinates based on section
                if section_data and 'offset' in section_data:
                    place_x = 200  # Base distance from robot
                    place_y = section_data['offset'][1] * 50  # Offset based on section
                else:
                    # Default placement if section not found
                    place_x = 200
                    place_y = 0
                
                # Execute placement
                print(f"Placing box {box['id']} at shelf {shelf} section {section}")
                response = self.send_arm_command("PLACE", str(place_x), str(place_y))
                
                if "SUCCESS" in response:
                    # Open gripper
                    self.send_arm_command("GRIP", "OPEN")
                    
                    # Update database
                    cursor = self.db.cursor()
                    cursor.execute('''
                        UPDATE boxes SET status = 'delivered', delivery_time = ?
                        WHERE id = ?
                    ''', (datetime.now(), box['id']))
                    
                    # Update shelf section occupancy
                    cursor.execute('''
                        UPDATE shelf_sections 
                        SET occupied = occupied + 1, last_updated = ?
                        WHERE shelf_id = ? AND section_id = ?
                    ''', (datetime.now(), shelf, section))
                    
                    self.db.commit()
                    
                    print(f"Box {box['id']} delivered successfully to {shelf} section {section}")
                else:
                    print(f"Delivery failed for box {box['id']}")
                    success = False
        
        # Return arm to home position
        self.send_arm_command("HOME")
        
        # Clear the carrying boxes list regardless of success
        # In real implementation, you might want to only remove successfully delivered boxes
        self.carrying_boxes = []
        
        return success
    
    def navigate_to_position(self, target_pos):
        """Navigate to target position using A* pathfinding"""
        print(f"Navigating from {self.current_position} to {target_pos}")
        
        # Handle tuple or dict target position
        if isinstance(target_pos, dict) and 'base' in target_pos:
            target_pos = target_pos['base']
        
        # Plan path
        path = self.a_star_pathfinding(self.current_position, target_pos)
        if not path:
            print("No path found!")
            return False
        
        # Execute path
        for i in range(1, len(path)):
            current = path[i-1]
            next_pos = path[i]
            
            # Calculate movement direction
            dx = next_pos[0] - current[0]
            dy = next_pos[1] - current[1]
            
            # Convert to robot commands
            if dx == 1 and dy == 0:
                direction = "FORWARD"
                target_angle = 0
            elif dx == -1 and dy == 0:
                direction = "FORWARD"
                target_angle = 180
            elif dx == 0 and dy == 1:
                direction = "FORWARD"
                target_angle = 90
            elif dx == 0 and dy == -1:
                direction = "FORWARD"
                target_angle = 270
            else:
                # Diagonal movement
                direction = "FORWARD"
                target_angle = math.degrees(math.atan2(dy, dx))
            
            # Turn to correct orientation
            angle_diff = target_angle - self.current_orientation
            if abs(angle_diff) > 5:  # 5 degree tolerance
                if angle_diff > 180:
                    angle_diff -= 360
                elif angle_diff < -180:
                    angle_diff += 360
                
                # Determine turn direction and execute turn
                if angle_diff > 0:
                    # Turn left (a)
                    print("Turning left...")
                    response = self.send_nav_command("TURN_LEFT")
                    time.sleep(abs(angle_diff) / 90)  # Approximate time to turn
                    self.send_nav_command("STOP")
                else:
                    # Turn right (d)
                    print("Turning right...")
                    response = self.send_nav_command("TURN_RIGHT")
                    time.sleep(abs(angle_diff) / 90)  # Approximate time to turn
                    self.send_nav_command("STOP")
                
                self.current_orientation = target_angle
            
            # Move forward
            print("Moving forward...")
            response = self.send_nav_command("MOVE", "FORWARD")
            
            # Wait approximate time to travel one cell
            cell_travel_time = self.cell_size / 100  # Assuming 100mm/sec speed
            time.sleep(cell_travel_time)
            
            # Stop movement
            self.send_nav_command("STOP")
            
            # Update position
            self.current_position = next_pos
            print(f"Moved to {self.current_position}")
            
            # Check for obstacles during movement
            obstacle_data = self.send_nav_command("SCAN")
            if "OBSTACLE" in obstacle_data:
                print("Obstacle detected, replanning...")
                self.update_obstacle_map(obstacle_data)
                return self.navigate_to_position(target_pos)  # Replan
        
        return True
    
    def update_obstacle_map(self, sensor_data):
        """Update obstacle map based on ultrasonic sensor data"""
        # Parse sensor data format: "NAV:OBSTACLE:FRONT:15" or "NAV:CLEAR:FRONT:50,LEFT:45,RIGHT:48"
        if ":" not in sensor_data:
            return
            
        parts = sensor_data.split(":")
        if len(parts) < 3:
            return
            
        # Skip the NAV: prefix
        status = parts[1]  # OBSTACLE or CLEAR
        
        # Process sensor readings if they exist
        if len(parts) >= 3:
            sensors_data = parts[2:]
            
            # Join remaining parts in case there are colons in the data
            sensor_readings = ":".join(sensors_data)
            
            # Split by comma for multiple sensor readings
            sensors = sensor_readings.split(',')
            obstacle_threshold = 20  # 20cm
            
            for sensor in sensors:
                if ":" in sensor:
                    try:
                        direction, distance = sensor.split(':')
                        distance = float(distance)
                        if distance < obstacle_threshold:
                            # Calculate obstacle position relative to robot
                            obstacle_pos = self.calculate_obstacle_position(direction, distance)
                            if obstacle_pos:
                                self.obstacle_map[obstacle_pos] = time.time()
                    except ValueError:
                        # Skip malformed sensor data
                        print(f"Malformed sensor data: {sensor}")
                        continue
    
    def calculate_obstacle_position(self, direction, distance):
        """Calculate obstacle grid position based on sensor data"""
        x, y = self.current_position
        
        if direction == "FRONT":
            if self.current_orientation == 0:
                return (x + 1, y)
            elif self.current_orientation == 90:
                return (x, y + 1)
            elif self.current_orientation == 180:
                return (x - 1, y)
            elif self.current_orientation == 270:
                return (x, y - 1)
        elif direction == "LEFT":
            if self.current_orientation == 0:
                return (x, y + 1)
            elif self.current_orientation == 90:
                return (x - 1, y)
            elif self.current_orientation == 180:
                return (x, y - 1)
            elif self.current_orientation == 270:
                return (x + 1, y)
        elif direction == "RIGHT":
            if self.current_orientation == 0:
                return (x, y - 1)
            elif self.current_orientation == 90:
                return (x + 1, y)
            elif self.current_orientation == 180:
                return (x, y + 1)
            elif self.current_orientation == 270:
                return (x - 1, y)
        
        return None
    
    def update_position_from_qr(self, floor_qr):
        """Update robot position based on floor QR marker"""
        # Format: FLOOR_X10_Y15
        if floor_qr.startswith('FLOOR_'):
            parts = floor_qr.split('_')
            if len(parts) >= 3:
                x = int(parts[1][1:])  # Remove 'X'
                y = int(parts[2][1:])  # Remove 'Y'
                self.current_position = (x, y)
                print(f"Position updated to {self.current_position}")
    
    def emergency_stop(self):
        """Emergency stop all robot operations"""
        self.send_nav_command("STOP")
        self.send_arm_command("STOP")
        self.robot_busy = False
        print("EMERGENCY STOP ACTIVATED")
    
    def get_status_report(self):
        """Get comprehensive robot status report"""
        cursor = self.db.cursor()
        
        # Get pending tasks
        cursor.execute("SELECT COUNT(*) FROM boxes WHERE status = 'pending'")
        pending_tasks = cursor.fetchone()[0]
        
        # Get completed tasks
        cursor.execute("SELECT COUNT(*) FROM boxes WHERE status = 'delivered'")
        completed_tasks = cursor.fetchone()[0]
        
        # Get recent logs
        cursor.execute('''
            SELECT * FROM robot_log 
            ORDER BY timestamp DESC LIMIT 5
        ''')
        recent_logs = cursor.fetchall()
        
        status = {
            'current_position': self.current_position,
            'current_orientation': self.current_orientation,
            'robot_busy': self.robot_busy,
            'pending_tasks': pending_tasks,
            'completed_tasks': completed_tasks,
            'obstacle_count': len(self.obstacle_map),
            'recent_logs': recent_logs,
            'carrying_boxes': len(self.carrying_boxes),
            'max_box_capacity': self.max_box_capacity,
            'carried_box_details': [box['id'] for box in self.carrying_boxes]
        }
        
        return status
    
    def shutdown(self):
        """Properly shutdown all robot systems"""
        print("Shutting down robot systems...")
        
        # Stop all movement
        if self.nav_uart:
            try:
                self.send_nav_command("STOP")
                self.nav_uart.close()
                print("Navigation controller disconnected")
            except Exception as e:
                print(f"Error closing navigation connection: {e}")
        
        if self.arm_uart:
            try:
                self.send_arm_command("STOP")
                self.arm_uart.close()
                print("Arm controller disconnected")
            except Exception as e:
                print(f"Error closing arm connection: {e}")
        
        # Release camera
        if self.camera:
            try:
                self.camera.release()
                print("Camera released")
            except Exception as e:
                print(f"Error closing camera: {e}")
        
        # Close database
        try:
            self.db.close()
            print("Database closed")
        except Exception as e:
            print(f"Error closing database: {e}")

    def main_loop(self):
        """Main operation loop for the robot"""
        print("Starting main operation loop...")
        
        # Initialize web server status update thread
        status_thread = threading.Thread(target=self.status_update_thread, daemon=True)
        status_thread.start()
        
        # Set up signal handler for graceful shutdown
        def signal_handler(sig, frame):
            print("\nReceived interrupt signal, shutting down gracefully...")
            self.shutdown()
            sys.exit(0)
            
        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, signal_handler)
        
        # Main loop
        try:
            while True:
                # Check for web commands
                self.check_web_commands()
                
                # Check for QR codes if camera available
                if self.camera:
                    qr_codes = self.scan_qr_codes()
                    for qr in qr_codes:
                        if qr['type'] == 'box':
                            # Process box task
                            self.process_box_task(qr['data'])
                        elif qr['type'] == 'floor_marker':
                            # Update position from floor marker
                            self.update_position_from_qr(qr['data'])
                
                # Sleep to prevent CPU overuse
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received in main loop, shutting down...")
            self.shutdown()
        except Exception as e:
            print(f"Error in main loop: {e}")
            self.shutdown()
    
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
                test_cmd = "No_Emergency\n"
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
                    self.arm_uart.write("CHECK_IR\n".encode())
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
    
    def execute_web_command(self, command_data):
        """Execute command received from web interface"""
        if not command_data or 'command' not in command_data:
            return
            
        cmd = command_data['command']
        params = command_data.get('params', {})
        command_id = command_data.get('id', 0)
        
        self.log_event("WEB_COMMAND", f"Received web command: {cmd}")
        
        # Track if command was executed
        command_executed = False
        command_result = "failed"
        
        # Navigation commands
        if cmd == "move":
            direction = params.get('direction', 'forward')
            if direction == 'forward':
                # Forward movement - w
                response = self.send_nav_command("MOVE", "FORWARD")
                command_executed = True
                command_result = response if response else "executed"
            elif direction == 'backward':
                # Backward movement - s
                response = self.send_nav_command("MOVE", "BACKWARD")
                command_executed = True
                command_result = response if response else "executed"
            elif direction == 'stop':
                response = self.send_nav_command("STOP")
                command_executed = True
                command_result = response if response else "executed"
        
        elif cmd == "turn":
            direction = params.get('direction', 'left')
            if direction == 'left':
                # Turn left - a
                response = self.send_nav_command("TURN_LEFT")
                command_executed = True
                command_result = response if response else "executed"
            elif direction == 'right':
                # Turn right - d
                response = self.send_nav_command("TURN_RIGHT")
                command_executed = True
                command_result = response if response else "executed"
        
        elif cmd == "stop":
            # Try to stop both controllers, but don't require both to be connected
            if self.nav_uart:
                self.send_nav_command("STOP")
                command_executed = True
                
            if self.arm_uart:
                self.send_arm_command("STOP")
                command_executed = True
                
            command_result = "executed"
        
        # Basic arm commands
        elif cmd == "grab":
            # Use the grab command for the arm
            response = self.send_arm_command("GRIP")
            command_executed = True
            command_result = response if response else "executed"
            
        elif cmd == "release":
            # Use the release command for the arm
            response = self.send_arm_command("GRIP", "OPEN")
            command_executed = True
            command_result = response if response else "executed"
            
        # New detailed arm commands
        elif cmd == "arm_move":
            arm_cmd = params.get('arm_cmd', '')
            
            # Arm movement commands
            if arm_cmd == 'f':  # Move Forward
                response = self.send_arm_command("MOVE_FORWARD")
                command_executed = True
                command_result = response if response else "executed"
            elif arm_cmd == 'b':  # Move Backward
                response = self.send_arm_command("MOVE_BACKWARD")
                command_executed = True
                command_result = response if response else "executed"
            elif arm_cmd == 'l':  # Strafe Left
                response = self.send_arm_command("STRAFE_LEFT")
                command_executed = True
                command_result = response if response else "executed"
            elif arm_cmd == 'r':  # Strafe Right
                response = self.send_arm_command("STRAFE_RIGHT")
                command_executed = True
                command_result = response if response else "executed"
            
            # Base rotation
            elif arm_cmd == 'q':  # Base Rotate Left
                response = self.send_arm_command("MOVE_MOTOR", "5", "BACK")
                command_executed = True
                command_result = response if response else "executed"
            elif arm_cmd == 'e':  # Base Rotate Right
                response = self.send_arm_command("MOVE_MOTOR", "5", "FWD")
                command_executed = True
                command_result = response if response else "executed"
            
            # Shoulder movement
            elif arm_cmd == 'z':  # Shoulder Down
                response = self.send_arm_command("MOVE_MOTOR", "6", "BACK")
                command_executed = True
                command_result = response if response else "executed"
            elif arm_cmd == 'x':  # Shoulder Up
                response = self.send_arm_command("MOVE_MOTOR", "6", "FWD")
                command_executed = True
                command_result = response if response else "executed"
            
            # Elbow movement
            elif arm_cmd == 'c':  # Elbow Down
                response = self.send_arm_command("MOVE_MOTOR", "7", "BACK")
                command_executed = True
                command_result = response if response else "executed"
            elif arm_cmd == 'v':  # Elbow Up
                response = self.send_arm_command("MOVE_MOTOR", "7", "FWD")
                command_executed = True
                command_result = response if response else "executed"
            
            # Gripper control
            elif arm_cmd == 'g':  # Gripper Open
                response = self.send_arm_command("GRIP", "OPEN")
                command_executed = True
                command_result = response if response else "executed"
            elif arm_cmd == 'h':  # Gripper Close
                response = self.send_arm_command("GRIP")
                command_executed = True
        
        elif cmd == "check_sensors":
            # Get ultrasonic data if nav controller is available
            ultrasonic = {"front": -1, "left": -1, "right": -1}
            if self.nav_uart:
                ultrasonic = self.get_ultrasonic_data()
                command_executed = True
                
            # Get line tracker data if arm controller is available
            line_trackers = {"raw": [-1, -1, -1, -1, -1]}
            if self.arm_uart:
                line_trackers = self.check_line_trackers()
                command_executed = True
            
            # Log the sensor data
            self.log_event("SENSOR_DATA", 
                          f"Ultrasonic: F={ultrasonic['front']}cm, L={ultrasonic['left']}cm, R={ultrasonic['right']}cm | " +
                          f"Line trackers: {line_trackers['raw']}")
            command_result = "executed"
        
        elif cmd == "emergency_stop":
            if self.nav_uart:
                self.send_nav_command("STOP")
                command_executed = True
                
            if self.arm_uart:
                self.send_arm_command("EMERGENCY")
                command_executed = True
            
            command_result = "executed"
            
        elif cmd == "clear_emergency":
            if self.arm_uart:
                self.send_arm_command("CLEAR_EMERGENCY")
                command_executed = True
                
            command_result = "executed"
        
        # Send result back to web server
        self.report_command_result(command_id, cmd, command_executed, command_result)
    
    def report_command_result(self, command_id, command, executed, result):
        """Report command execution result back to web server"""
        try:
            # Use a separate thread to avoid blocking
            def send_result():
                try:
                    import requests
                    requests.post(
                        'http://localhost:5000/api/command_result',
                        json={
                            'command_id': command_id,
                            'command': command,
                            'result': result if executed else 'failed'
                        },
                        timeout=1
                    )
                except Exception as e:
                    if self.debug_mode:
                        print(f"Error reporting command result: {e}")
            
            # Run in a separate thread to avoid blocking
            result_thread = threading.Thread(target=send_result, daemon=True)
            result_thread.start()
        except Exception as e:
            if self.debug_mode:
                print(f"Error creating result reporting thread: {e}")

    def a_star_pathfinding(self, start, goal):
        """A* pathfinding algorithm to navigate the grid"""
        # Initialize open and closed sets
        open_set = []
        closed_set = set()
        
        # Create start node
        start_node = (0, start, None)  # (f_score, position, parent)
        heapq.heappush(open_set, start_node)
        
        # Track g_scores (cost from start to node)
        g_scores = {start: 0}
        
        while open_set:
            # Get node with lowest f_score
            current_f, current_pos, parent = heapq.heappop(open_set)
            
            # Check if we've reached the goal
            if current_pos == goal:
                # Reconstruct path
                path = []
                while current_pos:
                    path.append(current_pos)
                    current_pos = parent
                    if current_pos in g_scores:
                        # Find parent of current position
                        for _, pos, p in open_set + list(closed_set):
                            if pos == current_pos:
                                parent = p
                                break
                    else:
                        break
                
                return path[::-1]  # Return reversed path
            
            # Add current to closed set
            closed_set.add((current_f, current_pos, parent))
            
            # Generate neighbors
            neighbors = []
            x, y = current_pos
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nx, ny = x + dx, y + dy
                
                # Check if valid position
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    # Check if not an obstacle
                    if (nx, ny) not in self.obstacle_map:
                        neighbors.append((nx, ny))
            
            # Process neighbors
            for neighbor in neighbors:
                # Skip if in closed set
                if any(pos == neighbor for _, pos, _ in closed_set):
                    continue
                
                # Calculate g_score for this neighbor
                tentative_g = g_scores[current_pos] + 1
                
                # Check if this path is better than any previous one
                if neighbor in g_scores and tentative_g >= g_scores[neighbor]:
                    continue
                
                # This is the best path so far
                g_scores[neighbor] = tentative_g
                
                # Calculate f_score (g_score + heuristic)
                h_score = abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])
                f_score = tentative_g + h_score
                
                # Add to open set
                heapq.heappush(open_set, (f_score, neighbor, current_pos))
        
        # No path found
        return []

    def get_ultrasonic_data(self):
        """Get ultrasonic sensor data from the navigation ESP32"""
        if not self.nav_uart:
            print("Navigation controller not connected!")
            return {
                "front": -1,
                "left": -1,
                "right": -1
            }
            
        try:
            # Clear input buffer before sending command
            self.nav_uart.reset_input_buffer()
            
            # Send command to get ultrasonic data
            # The ESP32 NAV automatically reads ultrasonic sensors in its main loop
            # We just need to request the data
            self.nav_uart.write("GET_ULTRASONIC\n".encode())
            
            # Wait for response
            start_time = time.time()
            timeout = 2  # Shorter timeout for sensor data
            
            while time.time() - start_time < timeout:
                try:
                    # Check if there's data in the queue from the communication handler
                    try:
                        response = self.nav_response_queue.get(timeout=0.1)
                        if response and "ULTRASONIC" in response:
                            # Parse response format: ULTRASONIC:FRONT:XX,LEFT:XX,RIGHT:XX
                            parts = response.replace("ULTRASONIC:", "").split(",")
                            data = {}
                            for part in parts:
                                if ":" in part:
                                    try:
                                        sensor, value = part.split(":")
                                        data[sensor.lower()] = int(value)
                                    except (ValueError, IndexError):
                                        # Handle malformed data
                                        continue
                            
                            # Fill in any missing values
                            for sensor in ["front", "left", "right"]:
                                if sensor not in data:
                                    data[sensor] = -1
                                    
                            return data
                    except queue.Empty:
                        # No data in queue, check if there's direct serial data
                        if self.nav_uart.in_waiting > 0:
                            response = self.nav_uart.readline().decode(errors='replace').strip()
                            if response and "ULTRASONIC" in response:
                                # Put in queue for processing
                                self.nav_response_queue.put(response)
                            else:
                                # Discard unrelated responses
                                if self.debug_mode and response:
                                    self.logger.debug(f"Discarded unrelated response: {response}")
                        else:
                            # Wait a bit before checking again
                            time.sleep(0.1)
                except Exception as e:
                    print(f"Error reading ultrasonic data: {e}")
                    break
                
            # If we get here, we timed out waiting for a response
            print("Timeout waiting for ultrasonic data")
            
            # Return default values if no response
            return {
                "front": -1,
                "left": -1,
                "right": -1
            }
        except Exception as e:
            print(f"Error getting ultrasonic data: {e}")
            return {
                "front": -1,
                "left": -1,
                "right": -1
            }

    def check_line_trackers(self):
        """Check line tracker sensors on the ARM ESP32"""
        if not self.arm_uart:
            print("Arm controller not connected!")
            return {
                "left_outer": -1,
                "left_inner": -1,
                "center": -1,
                "right_inner": -1,
                "right_outer": -1,
                "raw": [-1, -1, -1, -1, -1]
            }
            
        # Send command to check line tracker sensors
        self.arm_uart.write("CHECK_IR\n".encode())
        
        # Wait for response
        start_time = time.time()
        timeout = 5
        while time.time() - start_time < timeout:
            try:
                response = self.arm_response_queue.get(timeout=0.1)
                if response and response.startswith("IR_STATUS:"):
                    # Parse response format: IR_STATUS:1,0,1,0,1
                    values_str = response.replace("IR_STATUS:", "")
                    values = [int(v) for v in values_str.split(",") if v.strip()]
                    
                    # Ensure we have 5 values, pad with -1 if missing
                    while len(values) < 5:
                        values.append(-1)
                    
                    return {
                        "left_outer": values[0],
                        "left_inner": values[1],
                        "center": values[2],
                        "right_inner": values[3],
                        "right_outer": values[4],
                        "raw": values
                    }
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error parsing line tracker data: {e}")
                break
                
        # Return default values if no response
        return {
            "left_outer": -1,
            "left_inner": -1,
            "center": -1,
            "right_inner": -1,
            "right_outer": -1,
            "raw": [-1, -1, -1, -1, -1]
        }

    def identify_esp32_device(self, port):
        """Try to identify which ESP32 device is connected to a port (NAV or ARM)"""
        try:
            # Open the port
            ser = serial.Serial(port, 9600, timeout=2)
            
            # Clear any pending data
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(2)  # Give ESP32 time to stabilize
            
            # Send identification request
            if self.debug_mode:
                self.logger.debug(f"Sending identification request to {port}")
            
            # Try to identify by sending a command and checking response
            ser.write("IDENTIFY\n".encode())
            time.sleep(1)
            
            # Read response
            response = ""
            while ser.in_waiting > 0:
                line = ser.readline().decode(errors='replace').strip()
                if line:
                    response += line + "\n"
                    
            if self.debug_mode:
                self.logger.debug(f"Response from {port}: {response}")
            
            # Check response to identify device type
            if "NAV" in response or "NAVIGATION" in response:
                print(f"Identified navigation controller on {port}")
                return "NAV", ser
            elif "ARM" in response or "GRIPPER" in response:
                print(f"Identified arm controller on {port}")
                return "ARM", ser
            
            # If no specific identifier, try simple movement commands for NAV
            ser.reset_input_buffer()
            print(f"Testing if {port} is NAV controller using movement commands...")
            
            # Try a simple movement command (stop)
            ser.write("x\n".encode())
            time.sleep(0.5)
            
            # Check if there's any response
            if ser.in_waiting > 0:
                print(f"Identified navigation controller on {port} by command response")
                # Clear any response
                ser.reset_input_buffer()
                return "NAV", ser
                
            # Try ARM-specific command
            ser.reset_input_buffer()
            print(f"Testing if {port} is ARM controller...")
            ser.write("No_Emergency\n".encode())
            time.sleep(1)
            
            if ser.in_waiting > 0:
                response = ser.readline().decode(errors='replace').strip()
                if "IR_STATUS" in response or response:
                    print(f"Identified arm controller on {port} by command response")
                    return "ARM", ser
            
            # If we couldn't identify, close the port
            ser.close()
            return "UNKNOWN", None
            
        except Exception as e:
            if self.debug_mode:
                self.logger.error(f"Error identifying device on {port}: {e}")
            print(f"Error identifying device on {port}: {e}")
            return "ERROR", None

    def test_esp32_communication(self):
        """Test communication with connected ESP32 devices"""
        print("Testing communication with ESP32 devices...")
        
        # Test navigation controller
        if self.nav_uart:
            try:
                print("Testing navigation controller...")
                # Clear any pending data
                self.nav_uart.reset_input_buffer()
                
                # Send test command
                test_cmd = "IR:OFF\n"
                if self.debug_mode:
                    self.logger.debug(f"NAV TEST SEND: {test_cmd.strip()}")
                self.nav_uart.write(test_cmd.encode())
                
                # Wait for response
                time.sleep(1)
                if self.nav_uart.in_waiting:
                    response = self.nav_uart.readline().decode().strip()
                    if self.debug_mode:
                        self.logger.debug(f"NAV TEST RECV: {response}")
                    print(f"Navigation controller responded: {response}")
                else:
                    print("Navigation controller did not respond to test command")
            except Exception as e:
                print(f"Error testing navigation controller: {e}")
        else:
            print("Navigation controller not connected, skipping test")
        
        # Test arm controller
        if self.arm_uart:
            try:
                print("Testing arm controller...")
                # Clear any pending data
                self.arm_uart.reset_input_buffer()
                
                # Send test command
                test_cmd = "No_Emergency\n"
                if self.debug_mode:
                    self.logger.debug(f"ARM TEST SEND: {test_cmd.strip()}")
                self.arm_uart.write(test_cmd.encode())
                
                # Wait for response
                time.sleep(1)
                if self.arm_uart.in_waiting:
                    response = self.arm_uart.readline().decode().strip()
                    if self.debug_mode:
                        self.logger.debug(f"ARM TEST RECV: {response}")
                    print(f"Arm controller responded: {response}")
                else:
                    print("Arm controller did not respond to test command")
            except Exception as e:
                print(f"Error testing arm controller: {e}")
        else:
            print("Arm controller not connected, skipping test")

    def test_nav_motors(self):
        """Test navigation motors directly"""
        print("Testing navigation motors...")
        
        if not self.nav_uart:
            print("Navigation controller not connected!")
            return False
            
        try:
            # Test sequence: forward, stop, turn left, stop, turn right, stop
            print("Moving forward for 2 seconds...")
            self.nav_uart.write("w\n".encode())  # Send directly for testing
            time.sleep(2)
            
            print("Stopping...")
            self.nav_uart.write("x\n".encode())
            time.sleep(1)
            
            print("Turning left for 2 seconds...")
            self.nav_uart.write("a\n".encode())
            time.sleep(2)
            
            print("Stopping...")
            self.nav_uart.write("x\n".encode())
            time.sleep(1)
            
            print("Turning right for 2 seconds...")
            self.nav_uart.write("d\n".encode())
            time.sleep(2)
            
            print("Stopping...")
            self.nav_uart.write("x\n".encode())
            time.sleep(1)
            
            print("Moving backward for 2 seconds...")
            self.nav_uart.write("s\n".encode())
            time.sleep(2)
            
            print("Stopping...")
            self.nav_uart.write("x\n".encode())
            
            print("Navigation motor test complete")
            return True
            
        except Exception as e:
            print(f"Error testing navigation motors: {e}")
            return False
            
    def test_arm_motors(self):
        """Test arm motors directly"""
        print("Testing arm motors...")
        
        if not self.arm_uart:
            print("Arm controller not connected!")
            return False
            
        try:
            # First send No_Emergency to ensure the arm will respond
            print("Clearing emergency status...")
            self.send_arm_command("No_Emergency")
            time.sleep(1)
            
            # Test each motor individually
            for motor_num in [5, 6, 7]:
                # Forward
                print(f"Moving motor {motor_num} forward...")
                self.send_arm_command("MOVE_MOTOR", str(motor_num), "FWD")
                time.sleep(2)
                
                # Stop
                print(f"Stopping motor {motor_num}...")
                self.send_arm_command("STOP")
                time.sleep(1)
                
                # Backward
                print(f"Moving motor {motor_num} backward...")
                self.send_arm_command("MOVE_MOTOR", str(motor_num), "BACK")
                time.sleep(2)
                
                # Stop
                print(f"Stopping motor {motor_num}...")
                self.send_arm_command("STOP")
                time.sleep(1)
            
            # Test gripper
            print("Opening gripper...")
            self.send_arm_command("RELEASE")
            time.sleep(2)
            
            print("Closing gripper...")
            self.send_arm_command("GRAB")
            time.sleep(2)
            
            print("Arm motor test complete")
            return True
            
        except Exception as e:
            print(f"Error testing arm motors: {e}")
            return False

    def check_web_commands(self):
        """Check for commands from the web server"""
        try:
            import requests
            response = requests.get('http://localhost:5000/api/check_commands', timeout=1)
            if response.status_code == 200:
                data = response.json()
                if data.get('has_commands', False):
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