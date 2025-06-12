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

class RobotMasterController:
    def __init__(self, simulation_mode=False):
        # Simulation mode flag
        self.simulation_mode = simulation_mode
        
        # UART connections to ESP32 controllers with retry mechanism
        self.nav_uart = None
        self.arm_uart = None
        
        # Try to establish UART connections if not in simulation mode
        if not self.simulation_mode:
            self.connect_uart_devices()
        else:
            print("Running in SIMULATION MODE - No hardware connections required")
        
        # Camera setup with fallback options
        self.camera = None
        self.setup_camera()
        
        # Robot state
        self.current_position = (0, 0)  # Grid coordinates
        self.current_orientation = 0    # Degrees
        self.robot_busy = False
        self.obstacle_map = {}
        
        # Grid configuration (adjust to your environment)
        self.grid_size = 20  # 20x20 grid
        self.cell_size = 50  # 50mm per cell
        
        # Shelf positions (QR code -> grid coordinates with shelf sections)
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
        
        # Add a multi-box queue to track boxes being carried
        self.box_queue = []
        self.max_box_capacity = 2  # Maximum boxes the robot can carry at once
        
        # Robot state (update to track multiple box carrying)
        self.carrying_boxes = []  # List of boxes currently being carried
        
        # Command queues
        self.nav_response_queue = queue.Queue()
        self.arm_response_queue = queue.Queue()
        
        # Database setup
        self.init_database()
        
        # Start communication threads if connections established
        if (self.nav_uart and self.arm_uart) or self.simulation_mode:
            if self.simulation_mode:
                self.start_simulation_threads()
            else:
                self.start_communication_threads()
            print("Robot Master Controller Initialized")
        else:
            print("Robot partially initialized - some hardware not connected")
    
    def connect_uart_devices(self):
        """Attempt to connect to ESP32 controllers via UART with retry"""
        # Navigation controller connection
        uart_ports = ['/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/ttyUSB1', 'COM3', 'COM4']
        
        # Try to connect to navigation controller
        for port in uart_ports:
            try:
                print(f"Trying navigation controller on {port}...")
                self.nav_uart = serial.Serial(port, 115200, timeout=1)
                print(f"Connected to navigation controller on {port}")
                break
            except Exception as e:
                print(f"Failed to connect to {port}: {e}")
        
        # Try to connect to arm controller
        arm_ports = [p for p in uart_ports if p != (self.nav_uart.port if self.nav_uart else None)]
        for port in arm_ports:
            try:
                print(f"Trying arm controller on {port}...")
                self.arm_uart = serial.Serial(port, 115200, timeout=1)
                print(f"Connected to arm controller on {port}")
                break
            except Exception as e:
                print(f"Failed to connect to {port}: {e}")
        
        # Check if both connections established
        if not self.nav_uart:
            print("WARNING: Navigation controller not connected!")
        if not self.arm_uart:
            print("WARNING: Arm controller not connected!")
    
    def setup_camera(self):
        """Setup camera with fallback options for USB cameras"""
        # Skip camera setup in simulation mode if requested
        if self.simulation_mode and os.environ.get('SKIP_CAMERA', '0') == '1':
            print("Skipping camera setup in simulation mode (SKIP_CAMERA=1)")
            return
            
        # Try different camera indices
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
                        break
                    else:
                        print(f"Camera at index {camera_index} opened but couldn't read frame")
                        camera.release()
                else:
                    print(f"Failed to open camera at index {camera_index}")
            except Exception as e:
                print(f"Error with camera at index {camera_index}: {e}")
        
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
            if self.simulation_mode:
                print("In simulation mode, QR codes will be simulated.")
            else:
                print("Check camera connections and permissions.")
                print("On Raspberry Pi, ensure the camera is enabled with 'sudo raspi-config'")
                print("Check permissions with 'ls -l /dev/video*'")
                
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
        while True:
            try:
                if self.nav_uart and self.nav_uart.in_waiting:
                    response = self.nav_uart.readline().decode().strip()
                    self.nav_response_queue.put(response)
            except Exception as e:
                print(f"Nav UART error: {e}")
                # Try to reconnect
                try:
                    self.nav_uart.close()
                    self.connect_uart_devices()
                except:
                    pass
            time.sleep(0.01)
    
    def arm_communication_handler(self):
        """Handle arm ESP32 communication"""
        while True:
            try:
                if self.arm_uart and self.arm_uart.in_waiting:
                    response = self.arm_uart.readline().decode().strip()
                    self.arm_response_queue.put(response)
            except Exception as e:
                print(f"Arm UART error: {e}")
                # Try to reconnect
                try:
                    self.arm_uart.close()
                    self.connect_uart_devices()
                except:
                    pass
            time.sleep(0.01)
    
    def start_simulation_threads(self):
        """Start simulation threads for testing without hardware"""
        nav_thread = threading.Thread(target=self.simulation_nav_handler, daemon=True)
        nav_thread.start()
        
        arm_thread = threading.Thread(target=self.simulation_arm_handler, daemon=True)
        arm_thread.start()
        
        print("Simulation threads started")
    
    def simulation_nav_handler(self):
        """Simulate navigation controller responses"""
        while True:
            time.sleep(0.1)
            if not self.nav_response_queue.empty():
                continue
                
            # Simulate occasional position updates
            if random.random() < 0.05:  # 5% chance each cycle
                # Simulate small random movements
                x, y = self.current_position
                dx = random.choice([-1, 0, 1])
                dy = random.choice([-1, 0, 1])
                new_x = max(0, min(self.grid_size-1, x + dx))
                new_y = max(0, min(self.grid_size-1, y + dy))
                self.current_position = (new_x, new_y)
                
                # Simulate orientation changes
                self.current_orientation = (self.current_orientation + random.choice([-10, 0, 10])) % 360
                
                # Log the simulated movement
                self.log_event("SIM_MOVE", f"Simulated movement to {self.current_position}")
    
    def simulation_arm_handler(self):
        """Simulate arm controller responses"""
        while True:
            time.sleep(0.1)
            if not self.arm_response_queue.empty():
                continue
                
            # Simulate occasional arm actions
            if random.random() < 0.02:  # 2% chance each cycle
                action = random.choice(["GRIP", "RELEASE", "HOME"])
                self.log_event("SIM_ARM", f"Simulated arm action: {action}")
    
    def send_nav_command(self, action, param1="", param2="", timeout=10):
        """Send command to navigation ESP32 or simulate response"""
        if self.simulation_mode:
            # Simulate response
            time.sleep(0.2)  # Simulate processing delay
            
            # Log the command
            self.log_event("SIM_NAV_CMD", f"{action}:{param1}:{param2}")
            
            # Generate simulated response
            if action == "MOVE":
                # Update simulated position
                if param1 == "FORWARD":
                    distance = float(param2) if param2 else 50
                    angle_rad = math.radians(self.current_orientation)
                    dx = math.cos(angle_rad) * (distance / self.cell_size)
                    dy = math.sin(angle_rad) * (distance / self.cell_size)
                    
                    x, y = self.current_position
                    new_x = max(0, min(self.grid_size-1, x + round(dx)))
                    new_y = max(0, min(self.grid_size-1, y + round(dy)))
                    self.current_position = (new_x, new_y)
                
                return "NAV:SUCCESS:MOVE"
                
            elif action == "TURN":
                # Update simulated orientation
                if param1 == "LEFT":
                    self.current_orientation = (self.current_orientation + float(param2)) % 360
                elif param1 == "RIGHT":
                    self.current_orientation = (self.current_orientation - float(param2)) % 360
                
                return "NAV:SUCCESS:TURN"
                
            elif action == "STOP":
                return "NAV:SUCCESS:STOP"
                
            elif action == "POSITION":
                return "NAV:SUCCESS:POSITION"
                
            elif action == "SCAN":
                # Randomly decide if obstacle detected
                if random.random() < 0.1:  # 10% chance of obstacle
                    return "NAV:OBSTACLE:FRONT:15"
                else:
                    return "NAV:CLEAR:FRONT:50,LEFT:45,RIGHT:48"
            
            return "NAV:SUCCESS"
            
        elif not self.nav_uart:
            print("Navigation controller not connected!")
            return "ERROR:NOT_CONNECTED"
            
        command = f"NAV:{action}:{param1}:{param2}\n"
        self.nav_uart.write(command.encode())
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                response = self.nav_response_queue.get(timeout=0.1)
                return response
            except queue.Empty:
                continue
        return "TIMEOUT"
    
    def send_arm_command(self, action, param1="", param2="", timeout=15):
        """Send command to arm ESP32 or simulate response"""
        if self.simulation_mode:
            # Simulate response
            time.sleep(0.3)  # Simulate processing delay
            
            # Log the command
            self.log_event("SIM_ARM_CMD", f"{action}:{param1}:{param2}")
            
            # Generate simulated response
            if action == "PICK":
                # Simulate successful pickup most of the time
                if random.random() < 0.9:  # 90% success rate
                    return "ARM:SUCCESS:PICK"
                else:
                    return "ARM:ERROR:PICK:FAILED"
                    
            elif action == "PLACE":
                # Simulate successful placement most of the time
                if random.random() < 0.9:  # 90% success rate
                    return "ARM:SUCCESS:PLACE"
                else:
                    return "ARM:ERROR:PLACE:FAILED"
                    
            elif action == "GRIP":
                if param1 == "STATUS":
                    return "ARM:GRIPPED" if random.random() < 0.9 else "ARM:NOT_GRIPPED"
                elif param1 == "OPEN":
                    return "ARM:SUCCESS:GRIP_OPEN"
                else:
                    return "ARM:SUCCESS:GRIP"
                    
            elif action == "HOME":
                return "ARM:SUCCESS:HOME"
                
            elif action == "STOP":
                return "ARM:SUCCESS:STOP"
                
            elif action == "TRANSPORT":
                return "ARM:SUCCESS:TRANSPORT"
            
            return "ARM:SUCCESS"
            
        elif not self.arm_uart:
            print("Arm controller not connected!")
            return "ERROR:NOT_CONNECTED"
            
        command = f"ARM:{action}:{param1}:{param2}\n"
        self.arm_uart.write(command.encode())
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                response = self.arm_response_queue.get(timeout=0.1)
                return response
            except queue.Empty:
                continue
        return "TIMEOUT"
    
    def scan_qr_codes(self):
        """Scan for QR codes in camera view"""
        if not self.camera:
            # In simulation mode, occasionally return simulated QR codes
            if self.simulation_mode and random.random() < 0.05:  # 5% chance
                simulated_qr = {
                    'data': f"BOX_SIM{random.randint(1000, 9999)}_SHELF_A_SECTION_A1_WEIGHT_1.5",
                    'position': (320, 240),
                    'type': 'box'
                }
                return [simulated_qr]
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
                
                turn_direction = "LEFT" if angle_diff > 0 else "RIGHT"
                response = self.send_nav_command("TURN", turn_direction, str(abs(angle_diff)))
                
                if "SUCCESS" in response:
                    self.current_orientation = target_angle
                else:
                    print(f"Turn failed: {response}")
                    return False
            
            # Move forward
            distance = self.cell_size
            response = self.send_nav_command("MOVE", "FORWARD", str(distance))
            
            if "SUCCESS" in response:
                self.current_position = next_pos
                print(f"Moved to {self.current_position}")
            else:
                print(f"Movement failed: {response}")
                return False
            
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
                if ':' in sensor:
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
        
        # Main loop
        try:
            while True:
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
                elif self.simulation_mode:
                    # In simulation mode, randomly simulate finding a box
                    if random.random() < 0.01:  # 1% chance each cycle
                        box_id = f"SIM_BOX_{random.randint(1000, 9999)}"
                        shelf = random.choice(['A', 'B', 'C'])
                        section = f"{shelf}{random.randint(1, 3)}"
                        weight = round(random.uniform(0.5, 3.0), 1)
                        
                        # Create simulated QR data with correct format
                        qr_data = f"BOX_{box_id}_SHELF_{shelf}_SECTION_{section}_WEIGHT_{weight}"
                        
                        # Log the simulated find
                        self.log_event("SIM_FIND", f"Simulated finding box: {qr_data}")
                        
                        # Process the simulated box
                        self.process_box_task(qr_data)
                
                # Sleep to prevent CPU overuse
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("Main loop interrupted")
            return
    
    def status_update_thread(self):
        """Thread to periodically update web server status"""
        while True:
            try:
                # Get current status
                status = self.get_status_report()
                
                # Send to web server API
                import requests
                try:
                    response = requests.post(
                        'http://localhost:5000/api/update_robot_status',
                        json=status,
                        timeout=1
                    )
                    if response.status_code == 200:
                        data = response.json()
                        # Check for commands
                        if data.get('has_commands', False):
                            cmd = data.get('commands', {})
                            self.execute_web_command(cmd)
                except Exception as e:
                    # Web server might not be running yet
                    pass
            except Exception as e:
                print(f"Status update error: {e}")
                
            # Update every 2 seconds
            time.sleep(2)
    
    def execute_web_command(self, command_data):
        """Execute command received from web interface"""
        if not command_data or 'command' not in command_data:
            return
            
        cmd = command_data['command']
        params = command_data.get('params', {})
        
        self.log_event("WEB_COMMAND", f"Received web command: {cmd}")
        
        if cmd == "move":
            direction = params.get('direction', 'forward')
            if direction == 'forward':
                self.send_nav_command("MOVE", "FORWARD", "50")
            elif direction == 'backward':
                self.send_nav_command("MOVE", "BACKWARD", "50")
        
        elif cmd == "turn":
            direction = params.get('direction', 'left')
            if direction == 'left':
                self.send_nav_command("TURN", "LEFT", "45")
            elif direction == 'right':
                self.send_nav_command("TURN", "RIGHT", "45")
        
        elif cmd == "stop":
            self.send_nav_command("STOP")
            self.send_arm_command("STOP")
        
        elif cmd == "home":
            # Navigate to home position
            home_pos = self.shelf_positions.get('HOME')
            if home_pos:
                self.navigate_to_position(home_pos)
        
        elif cmd == "emergency_stop":
            self.emergency_stop()
            
        # Send result back to web server
        try:
            import requests
            requests.post(
                'http://localhost:5000/api/command_result',
                json={'command_id': command_data.get('id', 0), 'result': 'executed'},
                timeout=1
            )
        except:
            pass

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

if __name__ == "__main__":
    robot = None
    
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Robot Master Controller')
    parser.add_argument('--simulation', action='store_true', help='Run in simulation mode without hardware')
    args = parser.parse_args()
    
    try:
        robot = RobotMasterController(simulation_mode=args.simulation)
        
        # Check if essential components are connected
        if not robot.camera and not robot.simulation_mode:
            print("WARNING: Running without camera - QR code detection disabled")
            
        if (not robot.nav_uart or not robot.arm_uart) and not robot.simulation_mode:
            print("WARNING: Running with limited functionality")
            user_input = input("Do you want to continue anyway? (y/n): ")
            if user_input.lower() != 'y':
                print("Exiting by user request")
                robot.shutdown()
                exit(0)
        
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